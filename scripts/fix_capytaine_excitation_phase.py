"""Convert the excitation data in a Capytaine-derived BEMIO H5 file to exp(+i*omega*t).

Older ``run_bem.py`` writers stored Capytaine's excitation force without
conjugating it.  Capytaine uses exp(-i*omega*t) whereas BEMIO and SEA-Stack
use exp(+i*omega*t), so those files carry conjugated excitation phases and a
time-reversed excitation impulse response.

For F' = conj(F) the stored quantities transform exactly as:
    mag'   = mag                  (unchanged)
    phase' = -phase               (wrapped to (-pi, pi])
    f'(t)  = f(-t)                (requires a symmetric IRF time grid)

The file is modified in place and marked with the root attribute
``excitation_phase_convention = "exp(+iwt)"``.  A file that already carries
the marker is refused and left untouched, since converting it again would
restore the old, wrong convention; ``--force`` overrides this.  Files written
by the current ``run_bem.py`` already use exp(+i*omega*t) but carry no
marker: do not run this script on them.

Usage:
    python fix_capytaine_excitation_phase.py path/to/file.h5 [more.h5 ...] [--force]

Exit code 0 if every file was converted, 1 if any was refused or failed.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import h5py
import numpy as np

MARKER = "excitation_phase_convention"
CONVENTION = "exp(+iwt)"


def fix_file(path: Path, force: bool = False) -> None:
    """Convert one file in place. Raises ValueError (file untouched) if it cannot."""
    with h5py.File(str(path), "r+") as f:
        if MARKER in f.attrs and not force:
            raise ValueError(f"{path}: already converted ({MARKER} = {f.attrs[MARKER]!r}); "
                             "left untouched (use --force to convert again)")
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
        if MARKER in f.attrs:
            # --force on a marked file: the conversion just restored the other convention.
            del f.attrs[MARKER]
        else:
            f.attrs[MARKER] = CONVENTION


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", type=Path, nargs="+")
    ap.add_argument("--force", action="store_true",
                    help=f"convert even if the file carries {MARKER} (undoes an earlier run)")
    args = ap.parse_args()
    failed = 0
    for path in args.files:
        try:
            fix_file(path, args.force)
        except (OSError, ValueError, KeyError) as e:
            print(f"ERROR: {e}", file=sys.stderr)
            failed += 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
