"""Summarise crane lift diagnostics for the 3-panel control ladder."""

from __future__ import annotations

import csv
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CASES = ("lift_baseline", "lift_3dmc")
SETTLE_T0 = 28.0  # settle hover window start (move_end)
SETTLE_T1 = 34.0
LOWER_END = 42.0  # end of flight / start of set-down
STATIC_T = 3000.0 * 9.81
Z_REST = 8.6


def _f(row: dict, key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default) or default)
    except (TypeError, ValueError):
        return default


def summarise(name: str) -> None:
    p = ROOT / name / "outputs" / "crane_diagnostics.csv"
    if not p.is_file():
        print(f"{name}: MISSING {p}")
        return
    rows = list(csv.DictReader(p.open(encoding="utf-8")))
    if not rows:
        print(f"{name}: empty csv")
        return

    settle = [r for r in rows if SETTLE_T0 <= _f(r, "time_s") < SETTLE_T1] or rows[-100:]
    flight = [r for r in rows if _f(r, "time_s") < LOWER_END]
    mid = [r for r in rows if 10.0 <= _f(r, "time_s") <= 48.0]
    n_s = len(settle)
    n_f = max(len(flight), 1)
    n_m = max(len(mid), 1)

    def mean(rs: list, key: str) -> float:
        return sum(_f(r, key) for r in rs) / max(len(rs), 1)

    tip_err = mean(mid, "tip_err_3d_m")
    pay_xy = mean(settle, "err_xy_m")
    pay_xy_max = max(abs(_f(r, "err_xy_m")) for r in settle)
    swing_s = [math.degrees(_f(r, "swing_angle_rad")) for r in settle]
    swing_mean = sum(swing_s) / n_s
    swing_max = max(swing_s) if swing_s else 0.0
    swing_flight_late = [
        math.degrees(_f(r, "swing_angle_rad"))
        for r in rows
        if 30.0 <= _f(r, "time_s") < LOWER_END
    ]
    swing_max_late = max(swing_flight_late) if swing_flight_late else 0.0

    peak_t = max((_f(r, "tension_N") for r in flight), default=0.0)
    slack_frac = sum(1 for r in flight if int(_f(r, "slack")) == 1) / n_f
    sat_s = sum(int(_f(r, "sat_slew")) for r in mid) / n_m
    sat_l = sum(int(_f(r, "sat_luff")) for r in mid) / n_m
    sat_k = sum(int(_f(r, "sat_knuckle")) for r in mid) / n_m
    mode = settle[-1].get("mode", "?")

    # Touchdown: first sample with payload at/below resting centre + 5 cm.
    td_rows = [r for r in rows if _f(r, "payload_z_m") <= Z_REST + 0.05]
    td_t = td_vz = td_xy = float("nan")
    if td_rows:
        td_t = _f(td_rows[0], "time_s")
        td_xy = _f(td_rows[0], "err_xy_m")
        near = [r for r in rows if abs(_f(r, "time_s") - td_t) <= 0.05]
        if len(near) >= 2:
            z0, z1 = _f(near[0], "payload_z_m"), _f(near[-1], "payload_z_m")
            t0, t1 = _f(near[0], "time_s"), _f(near[-1], "time_s")
            td_vz = (z1 - z0) / max(t1 - t0, 1e-6)

    print(f"{name} (mode={mode})")
    print(
        f"  settle {SETTLE_T0:.0f}-{SETTLE_T1:.0f}s: payload XY mean={pay_xy:.3f} "
        f"max={pay_xy_max:.3f} m | swing mean={swing_mean:.1f} max={swing_max:.1f} deg"
    )
    print(
        f"  flight: tip 3d err (10-48s)={tip_err:.3f} m | swing max t>30={swing_max_late:.1f} deg | "
        f"slack_frac={slack_frac:.3f} | peak_T/static={peak_t / STATIC_T:.2f}"
    )
    print(
        f"  sat slew/luff/knuckle (10-48s)={sat_s:.2f}/{sat_l:.2f}/{sat_k:.2f} | "
        f"touchdown t={td_t:.2f}s vz={td_vz:.3f} m/s XY={td_xy:.3f} m"
    )


def main() -> None:
    for name in CASES:
        summarise(name)


if __name__ == "__main__":
    main()
