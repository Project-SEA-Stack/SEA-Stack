#!/usr/bin/env python3
"""
C3 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): SEA-Stack nonlinear
Froude-Krylov pressure force under prescribed heave vs the Giorgi & Ringwood
(2017) model, evaluated with nlfk_reference.py ("gr": deep-water Airy
pressure, waterline flat at the incident elevation on the body axis).

Pass criterion: RMS heave-force difference <= RMS_TOL_PCT of the force range,
for every case with kR <= KR_MAX (sphere and cylinder, a = 0.5 and 1.5 m).

Also reported (information): the difference from nlfk_reference.py's own
model ("airy_wheeler", exact geometry), which isolates mesh error.
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.append(str(HERE.parent / "utilities"))
sys.path.append(str(HERE.parent.parent / "verification" / "utilities"))
from compare_template_internal import write_comparison_status

import numpy as np

TEST_NAME = "compare_nlfk_prescribed_heave_sphere"
RMS_TOL_PCT = 2.0
KR_MAX = 0.15
RHO, G = 1000.0, 9.81


def read_header(path):
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.startswith("#"):
                break
            parts = line.lstrip("#").split()
            if len(parts) == 2:
                try:
                    out[parts[0]] = float(parts[1])
                except ValueError:
                    out[parts[0]] = parts[1]
    return out


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)
    base_dir = Path(sys.argv[1])
    try:
        import nlfk_reference as ref
    except ImportError as e:
        print(f"Cannot import nlfk_reference: {e}")
        sys.exit(77)

    bodies = {
        "sphere": ref.Sphere(radius=5.0, center=(0.0, 0.0, 2.0)),
        "cylinder": ref.VerticalCylinder(radius=3.0, z_bottom=-4.0, z_top=3.0),
    }
    metrics, failures = {}, []
    lines = ["| Body | T (s) | kR | a (m) | RMS vs G&R (% range) | RMS vs reference Airy (% range) | Checked |",
             "|---|---|---|---|---|---|---|"]
    files = sorted(base_dir.glob("*_T*_a*.txt"))
    if len(files) != 12:
        failures.append(f"expected 12 result files, found {len(files)}")
    for path in files:
        h = read_header(path)
        body = bodies[h["body"]]
        d = np.loadtxt(path, comments="#")
        t, z_cg, fz = d[:, 0], d[:, 1], d[:, 2]
        wave = ref.IncidentWave.regular(h["wave_amplitude_m"], h["period_s"], g=G, rho=RHO)
        f_gr = np.array([ref.pressure_force(body, (0.0, 0.0, z), wave, ti, "gr")[2] for ti, z in zip(t, z_cg)])
        f_aw = np.array([ref.pressure_force(body, (0.0, 0.0, z), wave, ti, "airy_wheeler")[2]
                         for ti, z in zip(t, z_cg)])
        rng = f_gr.max() - f_gr.min()
        rms_gr = 100.0 * np.sqrt(np.mean((fz - f_gr) ** 2)) / rng
        rms_aw = 100.0 * np.sqrt(np.mean((fz - f_aw) ** 2)) / rng
        kr = wave.k[0] * body.radius
        checked = kr <= KR_MAX
        key = path.stem
        metrics[f"{key}_rms_vs_gr_pct"] = rms_gr
        metrics[f"{key}_rms_vs_reference_airy_pct"] = rms_aw
        lines.append(f"| {h['body']} | {h['period_s']:g} | {kr:.3f} | {h['wave_amplitude_m']:g} | {rms_gr:.2f} | "
                     f"{rms_aw:.2f} | {'yes' if checked else 'info'} |")
        if checked and rms_gr > RMS_TOL_PCT:
            failures.append(f"{key}: RMS vs G&R {rms_gr:.2f}% > {RMS_TOL_PCT}%")

    summary = "\n".join(lines)
    print(summary)
    (base_dir / "summary.md").write_text(summary + "\n", encoding="utf-8")
    passed = not failures
    payload = write_comparison_status(str(base_dir), TEST_NAME, metrics, passed=passed)
    if failures:
        payload["note"] = "; ".join(failures)
        (base_dir / "status.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print("\nFailures:\n  " + "\n  ".join(failures))
    print(f"\n{TEST_NAME}: {'PASS' if passed else 'FAIL'}")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
