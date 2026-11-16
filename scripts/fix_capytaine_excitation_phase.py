"""Convert the excitation data in a Capytaine-derived BEMIO H5 file to exp(+i*omega*t).

Older ``run_bem.py`` writers stored Capytaine's excitation force without
conjugating it.  Capytaine uses exp(-i*omega*t) whereas BEMIO and SEA-Stack
use exp(+i*omega*t), so those files carry conjugated excitation phases and a
time-reversed excitation impulse response.

For F' = conj(F) the stored quantities transform exactly as:
    mag'   = mag                  (unchanged)
    phase' = -phase               (wrapped to (-pi, pi])
    f'(t)  = f(-t)                (requires a symmetric IRF time grid)

The file is modified in place.  Run once per file; running twice restores the
original.

Usage:
    python fix_capytaine_excitation_phase.py path/to/file.h5 [more.h5 ...]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import h5py
import numpy as np


def fix_file(path: Path) -> None:
    with h5py.File(str(path), "r+") as f:
        bodies = sorted(k for k in f.keys() if k.startswith("body"))
        if not bodies:
            raise ValueError(f"{path}: no body groups found")
        # Check every body before writing so a failure leaves the file untouched.
        for name in bodies:
            irf = f[f"{name}/hydro_coeffs/excitation"].get("impulse_response_fun")
            if irf is not None:
                t = irf["t"][...].ravel()
                tol = 1e-9 * max(1.0, float(np.max(np.abs(t))))
                if np.max(np.abs(t + t[::-1])) > tol:
                    raise ValueError(f"{path}:{name}: excitation IRF time grid is not symmetric")

        for name in bodies:
            exc = f[f"{name}/hydro_coeffs/excitation"]
            phase = exc["phase"][...]
            exc["phase"][...] = np.angle(np.exp(-1j * phase))

            irf = exc.get("impulse_response_fun")
            if irf is not None:
                f_irf = irf["f"][...]
                irf["f"][...] = f_irf[..., ::-1]
            print(f"  {path.name}:{name}: phase negated"
                  + (", excitation IRF time-reversed" if irf is not None else ""))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", type=Path, nargs="+")
    for path in ap.parse_args().files:
        fix_file(path)


if __name__ == "__main__":
    main()
