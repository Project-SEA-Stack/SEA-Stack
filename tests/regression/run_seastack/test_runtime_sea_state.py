#!/usr/bin/env python3
"""
Live wave period and direction changes through run_seastack --schedule
(PR-5, second slice).

A change samples the sea again (same seed and discretisation) and crossfades
to it over two wave periods, keeping each component's phase at the origin.
Every excitation path weights the old and new component sets.

Checks, on copies of the demo cases:
  1. Same value (period set to its own value) adds no component set, so the
     frequency-domain, IRF and nonlinear Froude-Krylov runs are bit-identical
     to the run without a schedule.
  2. Direction change on multi-heading BEM data (OSWEC, headings 0-90 deg):
     with the phases kept, the new component set is the one a run started at
     the new direction uses, so once the transition (and the IRF kernel) has
     passed, the excitation force equals that run's (frequency domain and IRF).
  3. Regular period change: the steady response matches a fresh run started
     with the new period (the phase differs, the amplitude does not).
  4. Determinism and /events for a real change.
  5. Refused changes stop a scheduled run with the reason: a direction outside
     the BEM headings (one heading; outside 0-90 deg), a period outside the
     BEM frequencies, a period change with constant-coefficient radiation, and
     a change while the previous one still fades in.
  6. Fixed frequency band (frequency_min/max): a period change inside the band
     runs, and one that moves energy above the BEM range is refused.
  7. Ten alternating period changes (IRF, NLFK): deterministic, all logged;
     with --reference-exe, equal to another build's results.
  8. Height, period and direction changes in one run: /events/waves records
     each change and the incident elevation as applied, and a rerun from the
     schedule rebuilt from /events is bit-identical.

Needs numpy and h5py (exit code 77, skipped, if missing). Usage:
    python test_runtime_sea_state.py --exe <path/to/run_seastack> [--reference-exe <other build>]
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    import h5py  # noqa: F401  (used through test_runtime_waves)
    import numpy as np
except ImportError as exc:
    print(f"SKIP: {exc}")
    sys.exit(77)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_runtime_waves import (_run_env, datasets, first_differing_row, identical,  # noqa: E402
                                prepare, run, write_schedule)

CHANGE_STEP = 1000


def set_wave(case: Path, field: str, value: float) -> None:
    """Set waves.<field> in the case's hydro YAML (first occurrence)."""
    hydro = next(case.glob("*.hydro.yaml"))
    text = hydro.read_text(encoding="utf-8")
    text, n = re.subn(rf"^(\s*){field}:\s*[0-9.eE+-]+", rf"\g<1>{field}: {value!r}", text, count=1,
                      flags=re.MULTILINE)
    if n != 1:
        raise RuntimeError(f"{hydro}: no {field}")
    hydro.write_text(text, encoding="utf-8")


def set_frequency_band(case: Path, f_min_hz: float, f_max_hz: float) -> None:
    """Pin the sampled frequency band (waves.frequency_min/max) in the hydro YAML."""
    hydro = next(case.glob("*.hydro.yaml"))
    text = hydro.read_text(encoding="utf-8")
    text, n = re.subn(r"^(\s*)seed:(.*)$",
                      rf"\g<1>seed:\g<2>\n\g<1>frequency_min: {f_min_hz!r}\n\g<1>frequency_max: {f_max_hz!r}",
                      text, count=1, flags=re.MULTILINE)
    if n != 1:
        raise RuntimeError(f"{hydro}: no waves.seed to anchor the frequency band")
    hydro.write_text(text, encoding="utf-8")


def time_step(case: Path) -> float:
    sim = next(case.glob("*.simulation.yaml")).read_text(encoding="utf-8")
    return float(re.search(r"time_step:\s*([0-9.eE+-]+)", sim).group(1))


def set_end_time(case: Path, end_time: float) -> None:
    sim = next(case.glob("*.simulation.yaml"))
    text = re.sub(r"end_time:\s*[0-9.eE+-]+", f"end_time: {end_time}", sim.read_text(encoding="utf-8"), count=1)
    sim.write_text(text, encoding="utf-8")


def series(results: dict, suffixes: tuple[str, ...]) -> dict:
    """Time series of moving bodies whose name ends with one of `suffixes`, by
    dataset name, without the columns (directions) that carry no motion or
    force: below 1e-4 of the largest of their kind, or below 1e-6 (solver noise)."""
    groups: dict[str, list[tuple[str, np.ndarray]]] = {}
    for k, v in results.items():
        if "/ground/" in k or not k.endswith(suffixes):
            continue
        groups.setdefault(k.rsplit("/", 1)[-1], []).append((k, np.asarray(v)))
    out = {}
    for kind in groups.values():
        kind_max = max(np.max(np.abs(v)) for _, v in kind)
        for k, v in kind:
            for col in range(v.shape[1]):
                if np.max(np.abs(v[:, col])) >= max(1e-6, 1e-4 * kind_max):
                    out[f"{k}[{col}]"] = v[:, col]
    return out


def max_relative_difference(a: dict, b: dict, suffixes: tuple[str, ...], t_from: float = -1.0) -> float:
    """max |b - a| / max |a| after t_from, worst over the series of `a`."""
    late = np.asarray(a["time/time"]) > t_from
    sa, sb = series(a, suffixes), series(b, suffixes)
    worst = 0.0
    for k, va in sa.items():
        if k not in sb:
            return float("inf")
        worst = max(worst, np.max(np.abs(sb[k][late] - va[late])) / np.max(np.abs(va[late])))
    return worst


VELOCITY = ("velocity",)
EXCITATION = ("force", "moment")


def rms_ratios(a: dict, b: dict, t_from: float) -> list[tuple[str, float]]:
    late = np.asarray(a["time/time"]) > t_from
    sb = series(b, VELOCITY)
    return [(k.split("/", 2)[2], np.sqrt(np.mean(sb[k][late] ** 2)) / np.sqrt(np.mean(va[late] ** 2)))
            for k, va in series(a, VELOCITY).items() if k in sb]


def excitation(path: Path) -> dict:
    """/results/hydro/excitation per body, with the time vector."""
    res = datasets(path, "results")
    out = {k: v for k, v in res.items() if k.startswith("hydro/excitation/")}
    out["time/time"] = res["time/time"]
    return out


def run_refused(exe: str, case: Path, schedule: Path) -> str:
    """Run expecting failure; return the combined output."""
    shutil.rmtree(case / "outputs", ignore_errors=True)
    proc = subprocess.run([exe, "--nogui", "--quiet", str(case), "--schedule", str(schedule)],
                          capture_output=True, text=True, encoding="utf-8", errors="replace",
                          env=_run_env(), timeout=1800)
    return ("" if proc.returncode != 0 else "<run succeeded>") + proc.stdout + proc.stderr


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--exe", required=True, help="path to run_seastack")
    parser.add_argument("--reference-exe", help="optional second build; check 7 compares its results with --exe")
    args = parser.parse_args()
    exe = args.exe

    failures = []

    def check(cond: bool, msg: str) -> None:
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        if not cond:
            failures.append(msg)

    root = Path(tempfile.mkdtemp(prefix="seastack_seastate_"))
    try:
        # 1 + 4. Same value through every layered path; a real change is deterministic.
        for name, case, period, new_period, end_time in [
            ("sphere_fd", "iea_sphere/regular_waves_lin", 8.0, 10.0, 40.0),
            ("rm3_irf", "rm3/irregular_waves", 8.0, 10.0, 60.0),
            ("sphere_nlfk", "iea_sphere/regular_waves_nlfk", 8.0, 10.0, 30.0),
        ]:
            print(f"{name}: {case}, waves.period")
            d = prepare(root, name, case, end_time)
            dt = time_step(d)
            same = d / "same.schedule.yaml"
            write_schedule(same, [(CHANGE_STEP, "waves.period", period)], dt)
            changed = d / "changed.schedule.yaml"
            write_schedule(changed, [(CHANGE_STEP, "waves.period", new_period)], dt)
            h5 = [run(exe, d, None), run(exe, d, same), run(exe, d, changed), run(exe, d, changed)]
            check(None not in h5, "all runs succeed")
            if None in h5:
                continue
            a, s, b, c = (datasets(h, "results") for h in h5)
            # A change to the value in effect adds no component set.
            ok_same, why_same = identical(a, s)
            check(ok_same, f"same value: bit-identical to the run without a schedule {why_same}")
            ok, why = identical(b, c)
            ok_ev, why_ev = identical(datasets(h5[2], "events"), datasets(h5[3], "events"))
            check(ok and ok_ev, f"period change: rerun is bit-identical {why} {why_ev}")
            ev = datasets(h5[2], "events")
            check(len(ev.get("step", [])) == 1 and ev["step"][0] == CHANGE_STEP
                  and ev["parameter"][0].decode() == "waves.period"
                  and ev["value"][0] == new_period and ev["previous_value"][0] == period,
                  f"/events records the period change: "
                  f"{dict((k, v[:1]) for k, v in ev.items() if not k.startswith('waves/'))}")
            row = first_differing_row(a, b)
            check(CHANGE_STEP <= row <= CHANGE_STEP + 5, f"period change: first differing row {row}")

        # 2. Direction change on multi-heading data: once the crossfade (and, for
        #    IRF, the kernel length) has passed, the excitation is that of a run
        #    started at the new direction. The body response converges to it
        #    only as slowly as the OSWEC flap's free motion decays, so the
        #    excitation is compared. Bodies barely yaw, so the world-frame
        #    rotation of the excitation adds only ~1e-7.
        detailed = ["--output-level", "detailed"]
        for name, case, end_time, late in [
            ("oswec_fd", "oswec/regular_waves", 90.0, 50.0),
            ("oswec_irf", "oswec/irregular_waves", 150.0, 110.0),
        ]:
            print(f"{name}: {case}, waves.direction 0 -> 30 deg")
            d = prepare(root, name, case, end_time)
            sched = d / "turn.schedule.yaml"
            write_schedule(sched, [(CHANGE_STEP, "waves.direction", 30.0)], time_step(d))
            h5_turn = run(exe, d, sched, detailed)
            fresh = prepare(root, name + "_fresh", case, end_time)
            set_wave(fresh, "direction", 30.0)
            h5_fresh = run(exe, fresh, None, detailed)
            h5_base = run(exe, prepare(root, name + "_base", case, end_time), None, detailed)
            check(None not in (h5_turn, h5_fresh, h5_base), "all runs succeed")
            if None in (h5_turn, h5_fresh, h5_base):
                continue
            ex_turn, ex_fresh, ex_base = excitation(h5_turn), excitation(h5_fresh), excitation(h5_base)
            diff = max_relative_difference(ex_fresh, ex_turn, EXCITATION, late)
            check(diff < 1e-5, f"after t = {late:g} s the excitation is that of a run started at 30 deg "
                               f"({diff:.2e})")
            ref = max_relative_difference(ex_base, ex_fresh, EXCITATION, late)
            check(ref > 1e-2, f"... and differs from the 0 deg run ({ref:.2e}), so the check has teeth")

        # 3. Regular period change: steady amplitude of a run started with the new period.
        print("sphere_fd: iea_sphere/regular_waves_lin, waves.period 8 -> 10 s")
        d = prepare(root, "sphere_period", "iea_sphere/regular_waves_lin", 120.0)
        sched = d / "period.schedule.yaml"
        write_schedule(sched, [(CHANGE_STEP, "waves.period", 10.0)], time_step(d))
        h5_change = run(exe, d, sched)
        fresh = prepare(root, "sphere_period_fresh", "iea_sphere/regular_waves_lin", 120.0)
        set_wave(fresh, "period", 10.0)
        h5_fresh = run(exe, fresh, None)
        check(None not in (h5_change, h5_fresh), "both runs succeed")
        if None not in (h5_change, h5_fresh):
            # t in (60, 120] s: six periods of 10 s.
            ratios = rms_ratios(datasets(h5_fresh, "results"), datasets(h5_change, "results"), 60.0)
            worst = max((abs(r - 1.0) for _, r in ratios), default=float("inf"))
            check(bool(ratios) and worst < 1e-3,
                  "steady response matches the 10 s run: " + ", ".join(f"{k} {r:.6f}" for k, r in ratios))

        # 5. Refused changes stop a scheduled run with the reason.
        print("refused changes")
        refusals = [
            ("sphere_dir", "iea_sphere/regular_waves_lin", [(CHANGE_STEP, "waves.direction", 30.0)],
             "one BEM heading"),
            ("oswec_dir", "oswec/regular_waves", [(CHANGE_STEP, "waves.direction", 120.0)],
             "outside the BEM headings 0 to 90 deg"),
            ("sphere_band", "iea_sphere/regular_waves_lin", [(CHANGE_STEP, "waves.period", 200.0)],
             "outside the BEM frequency range"),
            ("rm3_cc", "rm3/regular_waves_cc", [(CHANGE_STEP, "waves.period", 9.0)],
             "constant-coefficient radiation"),
            ("sphere_busy", "iea_sphere/regular_waves_lin",
             [(CHANGE_STEP, "waves.period", 9.0), (CHANGE_STEP + 1, "waves.period", 10.0)],
             "still fading in"),
        ]
        for name, case, events, reason in refusals:
            d = prepare(root, name, case, 15.0)
            dt = time_step(d)
            set_end_time(d, (CHANGE_STEP + 10) * dt)
            sched = d / "refused.schedule.yaml"
            write_schedule(sched, events, dt)
            out = run_refused(exe, d, sched)
            check("<run succeeded>" not in out and reason in out and "refused" in out,
                  f"{name}: run stops with '{reason}'"
                  + ("" if reason in out else f" (output: {out.strip().splitlines()[-3:]})"))

        # 6. Fixed frequency band (frequency_min/max): a period change keeps the
        #    frequencies or changes the pruned count. It used to be refused as
        #    "a different number of wave components", or to skip the BEM band
        #    check. RM3 BEM data: 0.02-5.2 rad/s (up to 0.83 Hz).
        print("rm3_band: rm3/irregular_waves, frequency band 0.04-1.2 Hz")
        d = prepare(root, "rm3_band", "rm3/irregular_waves", 30.0)
        set_frequency_band(d, 0.04, 1.2)
        dt = time_step(d)
        sched = d / "band.schedule.yaml"
        write_schedule(sched, [(CHANGE_STEP, "waves.period", 10.0)], dt)
        h5_base, h5_band = run(exe, d, None), run(exe, d, sched)
        check(None not in (h5_base, h5_band), "fixed band: period change 8 -> 10 s inside the band runs")
        if None not in (h5_base, h5_band):
            a, b = datasets(h5_base, "results"), datasets(h5_band, "results")
            row = first_differing_row(a, b)
            check(CHANGE_STEP <= row <= CHANGE_STEP + 5, f"fixed band: the change takes effect (first differing row {row})")
        set_end_time(d, (CHANGE_STEP + 10) * dt)
        write_schedule(sched, [(CHANGE_STEP, "waves.period", 2.0)], dt)
        out = run_refused(exe, d, sched)
        reason = "outside the BEM frequency range"
        check("<run succeeded>" not in out and reason in out,
              f"fixed band: Tp 2 s (energy above 0.83 Hz) stops with '{reason}'"
              + ("" if reason in out else f" (output: {out.strip().splitlines()[-3:]})"))

        # 7. Ten alternating period changes. IRF excitation frees the sets its
        #    kernel no longer reaches, and the NLFK pressure evaluates only sets
        #    with weight; neither may change the result. With --reference-exe
        #    the results are compared with another build (e.g. one without
        #    that freeing): IRF bit-identical; NLFK sums fewer zero terms, a
        #    round-off change that HHT's Newton tolerance turns into ~1e-8.
        for name, case, period, step_gap, tol in [
            ("rm3_irf_many", "rm3/irregular_waves", 8.0, 2500, 0.0),
            ("sphere_nlfk_many", "iea_sphere/regular_waves_nlfk", 8.0, 2500, 1e-6),
        ]:
            print(f"{name}: {case}, 10 alternating waves.period changes")
            d = prepare(root, name, case, 1.0)
            dt = time_step(d)
            set_end_time(d, (CHANGE_STEP + 10 * step_gap + 3000) * dt)
            sched = d / "many.schedule.yaml"
            write_schedule(sched, [(CHANGE_STEP + i * step_gap, "waves.period", period + (2.0 if i % 2 == 0 else 0.0))
                                   for i in range(10)], dt)
            h5 = [run(exe, d, sched), run(exe, d, sched)]
            if args.reference_exe:
                h5.append(run(args.reference_exe, d, sched))
            check(None not in h5, "all runs succeed")
            if None in h5:
                continue
            res = [datasets(h, "results") for h in h5]
            ok, why = identical(res[0], res[1])
            check(ok, f"rerun is bit-identical {why}")
            ev = datasets(h5[0], "events")
            check(len(ev.get("step", [])) == 10, f"/events records 10 changes ({len(ev.get('step', []))})")
            if args.reference_exe:
                if tol == 0.0:
                    ok, why = identical(res[2], res[0])
                    check(ok, f"bit-identical to --reference-exe {why}")
                else:
                    diff = max_relative_difference(res[2], res[0], VELOCITY)
                    check(diff <= tol, f"matches --reference-exe to {tol:g} ({diff:.2e})")

        # 8. Height, period and direction changes in one run: /events/waves holds
        #    each change and the incident elevation as applied, and a rerun from
        #    the schedule rebuilt from /events (what the GUI exports) is
        #    bit-identical. OSWEC: Hs 2 m, Tp 8 s, BEM headings 0-90 deg, dt 0.03 s.
        print("oswec_mixed: oswec/irregular_waves, height, period and direction changes")
        d = prepare(root, "oswec_mixed", "oswec/irregular_waves", 1.0)
        dt = time_step(d)
        set_end_time(d, 3300 * dt)
        set_wave(d, "ramp_duration", 10.0)  # demo: 60 s; ends before the first change at 30 s
        sched = d / "mixed.schedule.yaml"
        write_schedule(sched, [(1000, "waves.height", 2.5), (1700, "waves.period", 9.0),
                               (2500, "waves.direction", 20.0)], dt)
        h5_mixed = run(exe, d, sched)
        h5_plain = run(exe, d, None)
        check(None not in (h5_mixed, h5_plain), "both runs succeed")
        if None not in (h5_mixed, h5_plain):
            ev = datasets(h5_mixed, "events")
            rebuilt = d / "rebuilt.schedule.yaml"
            write_schedule(rebuilt, [(int(s), p.decode(), float(v))
                                     for s, p, v in zip(ev["step"], ev["parameter"], ev["value"])], dt)
            h5_rerun = run(exe, d, rebuilt)
            check(h5_rerun is not None, "rerun from the schedule rebuilt from /events succeeds")
            if h5_rerun is not None:
                ok, why = identical(datasets(h5_mixed, "results"), datasets(h5_rerun, "results"))
                ok_ev, why_ev = identical(ev, datasets(h5_rerun, "events"))
                check(ok and ok_ev, f"rerun is bit-identical in /results and /events {why} {why_ev}")
            with h5py.File(h5_mixed, "r") as f, h5py.File(h5_plain, "r") as g:
                w = f["events/waves"]
                kinds = [w[f"change_{i}"].attrs["kind"] for i in range(int(w.attrs["count"]))]
                kinds = [k.decode() if isinstance(k, bytes) else str(k) for k in kinds]
                starts = [float(w[f"change_{i}"].attrs["start_time"]) for i in range(len(kinds))]
                check(kinds == ["height", "period", "direction"] and np.allclose(starts, ev["time"], rtol=0, atol=0),
                      f"/events/waves: kinds {kinds}, start times {starts} = /events/time")
                check("components" not in w["change_0"] and
                      all(w[f"change_{i}/components/omega"].shape[0] > 0 for i in (1, 2)),
                      "component sets recorded for the period and direction changes")
                check(float(f["inputs/simulation/waves"].attrs["changed_during_run"]) == 1.0
                      and float(g["inputs/simulation/waves"].attrs["changed_during_run"]) == 0.0
                      and "waves" not in g.get("events", {}),
                      "changed_during_run 1 with changes, 0 (and no /events/waves) without")
                t = f["results/time/time"][()]
                eta = w["free_surface_eta"][()]
                init_t = f["inputs/simulation/waves/irregular/free_surface_time"][()]
                init_eta = f["inputs/simulation/waves/irregular/free_surface_eta"][()]
                ramp = float(f["visualization"].attrs["wave_ramp_duration"])
                idx = np.rint(t / dt).astype(int)
                check(eta.shape == t.shape and np.allclose(init_t[idx], t, rtol=0, atol=1e-9),
                      f"free_surface_eta sampled at /results/time/time ({eta.shape[0]} samples)")
                scale = max(np.max(np.abs(init_eta)), 1e-12)
                before = (t > ramp) & (t < 1000 * dt)
                after_height = (t > 1000 * dt + 18.0) & (t < 1700 * dt)  # ramp: 2 x the peak period
                err_before = np.max(np.abs(eta[before] - init_eta[idx[before]])) / scale
                err_height = np.max(np.abs(eta[after_height] - 1.25 * init_eta[idx[after_height]])) / scale
                check(before.any() and err_before < 1e-8,
                      f"before the first change it is the initial elevation ({err_before:.1e})")
                check(after_height.any() and err_height < 1e-8,
                      f"after the height ramp it is 2.5/2 x the initial elevation ({err_height:.1e})")
                late = t > 2500 * dt + 18.0
                err_late = np.max(np.abs(eta[late] - init_eta[idx[late]])) / scale
                check(late.any() and err_late > 1e-2,
                      f"after the period and direction changes it differs from the initial sea ({err_late:.1e})")
    finally:
        shutil.rmtree(root, ignore_errors=True)

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        return 1
    print("\nRuntime sea-state checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
