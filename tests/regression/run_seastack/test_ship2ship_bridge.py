#!/usr/bin/env python3
"""
Ship-to-ship bridge end to end: two coupled Wigley hulls (two-body BEM) in a
regular beam sea, with a Chrono::FEA aluminium ladder-frame bridge on four
pinned bearings across the 8 m gap, through run_seastack.

The ship2ship folder is copied to a temporary location and three cases run as
authored:

  transfer_hulls  hulls only. Peak relative heave |z2 - z1| after the 20 s
                  ramp is 1.21 m (H = 2 m, T = 8 s) within 5 %.
  bridge_static   still water. The run succeeds, which includes the
                  structure_static_load_path check at 10 s (2 %); the logged
                  bridge weight matches the hand calculation from the section
                  geometry within 0.1 %; the girder mid-span sag (relative to
                  the end chord) matches the simply supported hand calculation
                  within 2 %.
  bridge          beam sea. All diagnostics finite; relative heave within 2 %
                  of transfer_hulls (the 3.2 t bridge must not change the
                  hull motion); every bearing stays in compression (vertical
                  reaction on the hull < 0, z up); peak girder |Mz| in a
                  plausible band.

Needs numpy and h5py (exit code 77, skipped, if missing). Usage:
    python test_ship2ship_bridge.py --exe <path/to/run_seastack>
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

try:
    import h5py
    import numpy as np
except ImportError as exc:
    print(f"SKIP: {exc}")
    sys.exit(77)

_REPO = Path(__file__).resolve().parents[3]
_SHIP2SHIP = _REPO / "data" / "demos" / "run_seastack" / "ship2ship"

_G = 9.81             # m/s^2
_RHO_AL = 2700.0      # kg/m^3
_E_AL = 7.0e10        # Pa
_SPAN = 12.0          # m, girder length between bearings
_GAP = 4.0            # m, cross-beam length (girders at x = +/-2 m)
_PLANK_KG = 270.0     # kg per plank, one per girder node station
_STATIONS = 9         # girder node stations (8 elements)
_T_RAMP = 20.0        # s, wave ramp; motion statistics after this

_HULLS_REL_HEAVE_M = 1.21


def _box(width: float, depth: float, wall: float) -> tuple[float, float]:
    """Area [m^2] and strong-axis second moment [m^4] of a rectangular hollow section."""
    area = width * depth - (width - 2 * wall) * (depth - 2 * wall)
    i_strong = (width * depth**3 - (width - 2 * wall) * (depth - 2 * wall) ** 3) / 12.0
    return area, i_strong


def _hand_calc() -> tuple[float, float]:
    """Bridge weight [N] and girder mid-span sag [m] (simply supported, z up: sag > 0 down)."""
    a_g, i_g = _box(0.150, 0.300, 0.008)
    a_c, _ = _box(0.100, 0.150, 0.006)
    m_girder = a_g * _SPAN * _RHO_AL
    m_station = a_c * _GAP * _RHO_AL + _PLANK_KG
    weight = (2 * m_girder + _STATIONS * m_station) * _G
    ei = _E_AL * i_g
    w = a_g * _RHO_AL * _G                    # girder self-weight [N/m]
    p = 0.5 * m_station * _G                  # per girder per interior station [N]
    sag = 5 * w * _SPAN**4 / (384 * ei)
    pitch = _SPAN / (_STATIONS - 1)
    for k in range(1, _STATIONS - 1):         # end stations sit on the bearings
        a = min(k * pitch, _SPAN - k * pitch)
        sag += p * a * (3 * _SPAN**2 - 4 * a**2) / (48 * ei)
    return weight, sag


def _run_env() -> dict:
    env = dict(os.environ)
    extra = os.environ.get("SEASTACK_RUN_DLL_DIRS")
    if extra:
        dirs = [d for d in extra.split("|") if d]
        env["PATH"] = os.pathsep.join(dirs + [env.get("PATH", "")])
    return env


def _run(exe: str, case: Path) -> tuple[int, str]:
    t0 = time.perf_counter()
    proc = subprocess.run([exe, "--nogui", str(case)], capture_output=True, text=True,
                          encoding="utf-8", errors="replace", env=_run_env(), timeout=900)
    print(f"{case.name}: exit {proc.returncode}, {time.perf_counter() - t0:.1f} s wall")
    return proc.returncode, proc.stdout + proc.stderr


def _rel_heave(h5: Path) -> float:
    with h5py.File(h5, "r") as f:
        t = f["results/time/time"][:]
        z1 = f["results/model/bodies/body1/position"][:, 2]
        z2 = f["results/model/bodies/body2/position"][:, 2]
    return float(np.abs(z2 - z1)[t >= _T_RAMP].max())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--exe", required=True, help="path to run_seastack")
    parser.add_argument("--keep", action="store_true", help="keep the temporary folder")
    args = parser.parse_args()

    failures = []

    def check(cond: bool, msg: str) -> None:
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        if not cond:
            failures.append(msg)

    weight_hand, sag_hand = _hand_calc()
    root = Path(tempfile.mkdtemp(prefix="seastack_ship2ship_"))
    logs = []
    try:
        s2s = root / "ship2ship"
        shutil.copytree(_SHIP2SHIP, s2s, ignore=shutil.ignore_patterns(
            "outputs", "__pycache__", "*.log", ".venv"))

        # Hulls only.
        rc, log = _run(args.exe, s2s / "transfer_hulls")
        logs.append(log)
        check(rc == 0, "transfer_hulls: run_seastack succeeds")
        h5s = list((s2s / "transfer_hulls" / "outputs").glob("*.h5"))
        rel_hulls = _rel_heave(h5s[0]) if h5s else float("nan")
        check(abs(rel_hulls - _HULLS_REL_HEAVE_M) < 0.05 * _HULLS_REL_HEAVE_M,
              f"transfer_hulls: peak relative heave {rel_hulls:.4f} m vs "
              f"{_HULLS_REL_HEAVE_M} m (5 %)")

        # Bridge in still water.
        case = s2s / "bridge_static"
        rc, log = _run(args.exe, case)
        logs.append(log)
        check(rc == 0, "bridge_static: run_seastack succeeds")
        check("structure_static_load_path check PASSED" in log,
              "bridge_static: load-path check evaluated and passed")
        m = re.search(r"Structure weight:\s*([0-9.]+) kN", log)
        weight_log = float(m.group(1)) * 1e3 if m else float("nan")
        check(abs(weight_log - weight_hand) < 1e-3 * weight_hand,
              f"bridge_static: weight {weight_log / 1e3:.3f} kN vs hand calc "
              f"{weight_hand / 1e3:.3f} kN")
        h5s = list((case / "outputs").glob("*.h5"))
        check(bool(h5s), "bridge_static: HDF5 written")
        if h5s:
            with h5py.File(h5s[0], "r") as f:
                t = f["results/time/time"][:]
                i = int(np.argmin(np.abs(t - 10.0)))
                for beam in ("girder_a", "girder_b"):
                    pos = f[f"results/structure/{beam}/node_positions"][i]
                    und = f[f"inputs/structure/{beam}/undeformed_node_positions"][:]
                    mid = pos.shape[0] // 2
                    sag = -((pos[mid, 2] - 0.5 * (pos[0, 2] + pos[-1, 2]))
                            - (und[mid, 2] - 0.5 * (und[0, 2] + und[-1, 2])))
                    check(abs(sag - sag_hand) < 0.02 * sag_hand,
                          f"bridge_static: {beam} mid-span sag {sag * 1e3:.2f} mm vs hand calc "
                          f"{sag_hand * 1e3:.2f} mm (2 %)")

        # Bridge in the beam sea.
        case = s2s / "bridge"
        rc, log = _run(args.exe, case)
        logs.append(log)
        check(rc == 0, "bridge: run_seastack succeeds")
        h5s = list((case / "outputs").glob("*.h5"))
        rel_bridge = _rel_heave(h5s[0]) if h5s else float("nan")
        check(abs(rel_bridge - rel_hulls) < 0.02 * rel_hulls,
              f"bridge: relative heave {rel_bridge:.4f} m vs hulls only {rel_hulls:.4f} m (2 %)")
        csv_path = case / "outputs" / "structure_diagnostics.csv"
        check(csv_path.is_file(), "bridge: structure diagnostics CSV written")
        if csv_path.is_file():
            with csv_path.open(newline="", encoding="utf-8") as fh:
                rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(fh)]
            check(all(math.isfinite(v) for r in rows for v in r.values()),
                  "bridge: all diagnostics values finite")
            late = [r for r in rows if r["time_s"] >= _T_RAMP]
            for b in ("stbd_a", "stbd_b", "port_a", "port_b"):
                fz_max = max(r[f"bearing_{b}_Fz_N"] for r in late)
                check(fz_max < 0.0,
                      f"bridge: bearing_{b} stays in compression (max Fz on hull "
                      f"{fz_max / 1e3:+.3f} kN)")
            peak_mz = max(r["girder_a_max_abs_Mz_Nm"] for r in late)
            check(1.0e4 < peak_mz < 5.0e4,
                  f"bridge: peak girder |Mz| {peak_mz / 1e3:.2f} kN*m in (10, 50)")
        if failures:
            for log in logs:
                print("\n".join(log.strip().splitlines()[-15:]))
    finally:
        if args.keep:
            print(f"kept {root}")
        else:
            shutil.rmtree(root, ignore_errors=True)

    if failures:
        print(f"\nFAILED: {len(failures)} check(s)")
        return 1
    print("\nShip-to-ship bridge passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
