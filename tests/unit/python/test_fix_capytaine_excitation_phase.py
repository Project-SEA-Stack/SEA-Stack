#!/usr/bin/env python3
"""scripts/fix_capytaine_excitation_phase.py converts a file once.

On a small synthetic BEMIO file: the first run negates the excitation phase,
time-reverses the excitation IRF and writes the marker; a second run is
refused (exit code 1) and leaves the file byte-identical; --force converts
back and removes the marker. Exit code 77 (skipped) without numpy or h5py.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

try:
    import h5py
    import numpy as np
except ImportError as exc:
    print(f"SKIP: {exc}")
    sys.exit(77)

SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "fix_capytaine_excitation_phase.py"


def make_file(path: Path) -> tuple[np.ndarray, np.ndarray]:
    phase = np.array([[[0.3, -2.0, 3.0]]])
    f_irf = np.array([[[1.0, 2.0, 3.0, 4.0, 5.0]]])
    with h5py.File(path, "w") as f:
        exc = f.create_group("body1/hydro_coeffs/excitation")
        exc["phase"] = phase
        exc["mag"] = np.ones_like(phase)
        irf = exc.create_group("impulse_response_fun")
        irf["t"] = np.linspace(-2.0, 2.0, 5)
        irf["f"] = f_irf
    return phase, f_irf


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True)


def main() -> int:
    failures = []

    def check(cond: bool, msg: str) -> None:
        print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
        if not cond:
            failures.append(msg)

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "bem.h5"
        phase, f_irf = make_file(path)

        first = run(str(path))
        check(first.returncode == 0, f"first run converts (exit {first.returncode}) {first.stderr.strip()}")
        with h5py.File(path, "r") as f:
            exc = f["body1/hydro_coeffs/excitation"]
            check(np.allclose(exc["phase"][...], -phase), "phase negated")
            check(np.array_equal(exc["impulse_response_fun/f"][...], f_irf[..., ::-1]), "IRF time-reversed")
            marker = f.attrs.get("excitation_phase_convention")
            check(marker in ("exp(+iwt)", b"exp(+iwt)"), f"marker written ({marker!r})")

        before = path.read_bytes()
        second = run(str(path))
        check(second.returncode == 1 and "already converted" in second.stderr,
              f"second run refused (exit {second.returncode}): {second.stderr.strip()}")
        check(path.read_bytes() == before, "refused file is byte-identical")

        forced = run(str(path), "--force")
        check(forced.returncode == 0, f"--force converts again (exit {forced.returncode})")
        with h5py.File(path, "r") as f:
            exc = f["body1/hydro_coeffs/excitation"]
            check(np.allclose(exc["phase"][...], phase) and
                  np.array_equal(exc["impulse_response_fun/f"][...], f_irf),
                  "--force restores the original data")
            check("excitation_phase_convention" not in f.attrs, "--force removes the marker")

    if failures:
        print(f"FAILED: {len(failures)} check(s)")
        return 1
    print("fix_capytaine_excitation_phase checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
