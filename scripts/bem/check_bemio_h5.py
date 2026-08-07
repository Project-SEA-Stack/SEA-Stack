"""Sanity-check a BEMIO HDF5 file written by ``bemio_from_dataset.py``.

Hull-agnostic structural and convention checks, plus optional comparisons
against analytic hydrostatics when the expected values are supplied.

All stored coefficients are BEMIO-normalised, so the printed quantities are:

    linear_restoring_stiffness[2,2]  = C33/(rho g)  -> waterplane area   [m^2]
    added_mass/inf_freq[i,i]         = Ainf/rho                          [m^3]
    excitation/mag[2,h,0]            = |F3|/(rho g) -> waterplane area   [m^2]

The excitation phase check is the one that catches a mirrored sign
convention: heave follows wave elevation (phase ~ 0), while surge follows the
water-particle acceleration and therefore *leads* elevation by +90 deg under
the BEMIO e^{+iwt} convention. A stored -90 deg means the Capytaine
imaginary-sign flip was skipped.

Usage:
    python check_bemio_h5.py path/to/model.h5
    python check_bemio_h5.py model.h5 --waterplane-area 400.0 --disp-vol 933.3
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import h5py
import numpy as np

DOF_LABELS = ("Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw")

REQUIRED_BODY_DATASETS = (
    "properties/cg",
    "properties/cb",
    "properties/disp_vol",
    "hydro_coeffs/linear_restoring_stiffness",
    "hydro_coeffs/added_mass/inf_freq",
    "hydro_coeffs/added_mass/all",
    "hydro_coeffs/radiation_damping/all",
    "hydro_coeffs/radiation_damping/impulse_response_fun/K",
    "hydro_coeffs/radiation_damping/impulse_response_fun/t",
    "hydro_coeffs/excitation/mag",
    "hydro_coeffs/excitation/phase",
    "hydro_coeffs/excitation/re",
    "hydro_coeffs/excitation/im",
    "hydro_coeffs/excitation/impulse_response_fun/f",
    "hydro_coeffs/excitation/impulse_response_fun/t",
)


def _flat(node) -> np.ndarray:
    return np.asarray(node[()], dtype=float).ravel()


def _text(node) -> str:
    raw = node[()]
    if isinstance(raw, bytes):
        return raw.decode()
    return str(raw)


def natural_periods(
    w: np.ndarray,
    a_all: np.ndarray,
    a_inf: np.ndarray,
    stiffness: np.ndarray,
    generalised_mass: np.ndarray,
    rho: float,
    g: float,
    dof_start: int,
) -> dict[str, float]:
    """Undamped natural periods of the uncoupled DOFs [s].

    Solves ``T = 2 pi sqrt((M + A(w_n)) / C)`` for each diagonal DOF. Added mass
    is frequency dependent, so the frequency is iterated: start from the
    infinite-frequency value and re-interpolate ``A`` at the current estimate
    until it settles. Using ``Ainf`` alone would overestimate every frequency.

    Cross-body and cross-DOF coupling is ignored, so these are the single-body
    uncoupled estimates a free-decay test should reproduce to within the damping
    correction.

    Parameters
    ----------
    a_all, a_inf, stiffness
        As stored in the H5, i.e. ``A/rho`` and ``C/(rho g)``.
    generalised_mass
        Physical mass for DOFs 0-2 and moment of inertia for DOFs 3-5 [kg, kg m^2].
    """
    out: dict[str, float] = {}
    for i, label in enumerate(DOF_LABELS):
        c = rho * g * stiffness[i, i]
        m = generalised_mass[i]
        if c <= 0.0 or m <= 0.0:
            continue  # no restoring force (surge, sway, yaw) or unset inertia
        a = rho * a_inf[i, dof_start + i]
        omega = float("nan")
        for _ in range(50):
            omega_new = np.sqrt(c / (m + a))
            if np.isfinite(omega) and abs(omega_new - omega) < 1e-9:
                omega = omega_new
                break
            omega = omega_new
            # A is stored on the H5 frequency grid; clamp outside it.
            a = rho * np.interp(omega, w, a_all[i, dof_start + i, :])
        out[label] = 2.0 * np.pi / omega
    return out


def check(
    h5_path: Path,
    waterplane_area: float | None,
    disp_vol: float | None,
    tol_pct: float,
    body_mass: float | None = None,
    body_inertia: tuple[float, float, float] | None = None,
) -> bool:
    failures: list[str] = []
    warnings: list[str] = []

    with h5py.File(h5_path, "r") as f:
        w = _flat(f["simulation_parameters/w"])
        rho = _flat(f["simulation_parameters/rho"])[0]
        g = _flat(f["simulation_parameters/g"])[0]
        headings = _flat(f["simulation_parameters/wave_dir"])
        depth_node = f["simulation_parameters/water_depth"][()]
        if isinstance(depth_node, bytes):
            depth_s = depth_node.decode()
        else:
            depth_s = f"{np.asarray(depth_node, dtype=float).ravel()[0]:.4g} m"
        code = _text(f["bem_data/code"]) if "bem_data/code" in f else "(missing)"

        body_names = sorted(k for k in f.keys() if k.startswith("body"))

        print(f"File: {h5_path.name}")
        print(f"  code={code}  rho={rho:g}  g={g:g}  depth={depth_s}")
        print(f"  Nf={w.size}  omega=[{w[0]:.3f}, {w[-1]:.3f}] rad/s")
        print(f"  Nh={headings.size}  headings={np.round(headings, 2)} deg")
        print(f"  bodies: {', '.join(body_names)}")

        if not np.all(np.diff(w) > 0):
            failures.append("frequencies are not strictly ascending")

        for body in body_names:
            print(f"\n  --- {body} ---")
            missing = [d for d in REQUIRED_BODY_DATASETS if f"{body}/{d}" not in f]
            if missing:
                failures.append(f"{body}: missing datasets: {', '.join(missing)}")
                continue

            name = _text(f[f"{body}/properties/name"]) if f"{body}/properties/name" in f else "?"
            cg = _flat(f[f"{body}/properties/cg"])
            cb = _flat(f[f"{body}/properties/cb"])
            vol = _flat(f[f"{body}/properties/disp_vol"])[0]
            C = np.asarray(f[f"{body}/hydro_coeffs/linear_restoring_stiffness"][()], dtype=float)
            Ainf = np.asarray(f[f"{body}/hydro_coeffs/added_mass/inf_freq"][()], dtype=float)
            A = np.asarray(f[f"{body}/hydro_coeffs/added_mass/all"][()], dtype=float)
            B = np.asarray(f[f"{body}/hydro_coeffs/radiation_damping/all"][()], dtype=float)
            mag = np.asarray(f[f"{body}/hydro_coeffs/excitation/mag"][()], dtype=float)
            phase = np.asarray(f[f"{body}/hydro_coeffs/excitation/phase"][()], dtype=float)

            print(f"  name={name}")
            print(f"  cg={np.round(cg, 4)}  cb={np.round(cb, 4)}")
            print(f"  disp_vol={vol:.4f} m^3  (neutral mass {rho * vol / 1000.0:.3f} t)")
            print(f"  C33/(rho g)={C[2, 2]:.4f} m^2   C44/(rho g)={C[3, 3]:.4f} m^4"
                  f"   C55/(rho g)={C[4, 4]:.4f} m^4")

            # The self-coupling 6x6 block sits in columns [dof_start, dof_end).
            start = int(_flat(f[f"{body}/properties/dof_start"])[0]) - 1
            self_block = Ainf[:, start:start + 6]
            print("  Ainf/rho diag [m^3]: " + "  ".join(
                f"{lbl}={self_block[i, i]:.4g}" for i, lbl in enumerate(DOF_LABELS)))

            if not np.isfinite(A).all() or not np.isfinite(Ainf).all():
                failures.append(f"{body}: NaN/Inf in added mass")
            if not np.isfinite(B).all():
                failures.append(f"{body}: NaN/Inf in radiation damping")
            if np.any(np.diag(self_block) <= 0.0):
                failures.append(f"{body}: non-positive diagonal in Ainf self-block")
            # Radiation damping must be non-negative on the diagonal (energy out).
            diag_b = np.array([B[i, start + i, :].min() for i in range(6)])
            if np.any(diag_b < -1e-6 * max(1.0, abs(diag_b).max())):
                warnings.append(f"{body}: negative diagonal radiation damping "
                                f"(min {diag_b.min():.3e})")
            if C[2, 2] <= 0.0:
                failures.append(f"{body}: non-positive heave restoring stiffness")

            # Convention check: heave follows wave elevation, surge follows the
            # water-particle acceleration and so leads elevation by 90 deg. The
            # sign of that lead flips with the direction of propagation, so the
            # test is anchored on the 0 deg heading (waves travelling towards +x),
            # where the stored phase must be +90 deg.
            i_lo = 0
            for h in range(mag.shape[1]):
                heave_deg = np.degrees(phase[2, h, i_lo])
                surge_deg = np.degrees(phase[0, h, i_lo])
                print(f"  heading {headings[h]:6.1f} deg, omega={w[i_lo]:.3f}: "
                      f"|F3|/(rho g)={mag[2, h, i_lo]:.4f} m^2, "
                      f"phase3={heave_deg:+.1f} deg, phase1={surge_deg:+.1f} deg")
                following_x = abs(headings[h] % 360.0) < 1.0
                if following_x and mag[0, h, i_lo] > 1e-9 and surge_deg < 0.0:
                    failures.append(
                        f"{body}: surge excitation phase {surge_deg:+.1f} deg at "
                        f"heading 0 deg; expected ~+90 deg. The Capytaine e^-iwt "
                        f"-> BEMIO e^+iwt imaginary-sign flip looks to be missing.")

            if waterplane_area is not None:
                # Hydrostatic heave stiffness is frequency-independent, so this is
                # a strict check on the panel mesh.
                err = 100.0 * (C[2, 2] / waterplane_area - 1.0)
                flag = "WARN" if abs(err) > tol_pct else "ok"
                print(f"  C33/(rho g) vs Awp={waterplane_area:.3f} m^2: "
                      f"{C[2, 2]:.4f} ({err:+.2f} %) [{flag}]")
                if abs(err) > tol_pct:
                    warnings.append(f"{body}: C33/(rho g) differs from the expected "
                                    f"waterplane area by {err:+.2f} %")

                # Heave excitation only approaches rho g Awp in the long-wave
                # limit, and always from below (the wave crest does not span the
                # whole waterplane, and pressure decays over the draft). So this
                # is a scaling check, not an equality: flag values above Awp or
                # far below it, and otherwise just report the ratio.
                exc_lo = mag[2, 0, 0]
                ratio = exc_lo / waterplane_area
                print(f"  |F3|/(rho g) at omega={w[0]:.3f} rad/s: {exc_lo:.4f} m^2 "
                      f"({100.0 * ratio:.1f} % of Awp; -> 100 % as omega -> 0)")
                if ratio > 1.0 + tol_pct / 100.0 or ratio < 0.5:
                    warnings.append(f"{body}: long-wave heave excitation is "
                                    f"{100.0 * ratio:.1f} % of rho g Awp, which "
                                    f"suggests an excitation scaling error")

            if body_mass is not None:
                gen_mass = np.array([
                    body_mass, body_mass, body_mass,
                    body_inertia[0] if body_inertia else 0.0,
                    body_inertia[1] if body_inertia else 0.0,
                    body_inertia[2] if body_inertia else 0.0,
                ])
                periods = natural_periods(w, A, Ainf, C, gen_mass, rho, g, start)
                print("  Undamped natural periods [s]: " + "  ".join(
                    f"{k}={v:.2f}" for k, v in periods.items()))

            if disp_vol is not None:
                err = 100.0 * (vol / disp_vol - 1.0)
                flag = "WARN" if abs(err) > tol_pct else "ok"
                print(f"  disp_vol vs expected {disp_vol:.4f} m^3: "
                      f"{vol:.4f} ({err:+.2f} %) [{flag}]")
                if abs(err) > tol_pct:
                    warnings.append(f"{body}: disp_vol differs from expected by {err:+.2f} %")

    print()
    for msg in warnings:
        print(f"WARN: {msg}")
    for msg in failures:
        print(f"FAIL: {msg}")
    print("OK" if not failures else "FAILED")
    return not failures


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("h5", type=Path, help="BEMIO HDF5 file to check")
    ap.add_argument("--waterplane-area", type=float, default=None,
                    help="Expected waterplane area [m^2]; compared with C33/(rho g) "
                         "and the long-wave heave excitation")
    ap.add_argument("--disp-vol", type=float, default=None,
                    help="Expected displaced volume [m^3]")
    ap.add_argument("--tol-pct", type=float, default=3.0,
                    help="Tolerance for the analytic comparisons [%%] (default: 3)")
    ap.add_argument("--body-mass", type=float, default=None,
                    help="Body mass [kg]; enables the natural-period report")
    ap.add_argument("--body-inertia", type=float, nargs=3, default=None,
                    metavar=("IXX", "IYY", "IZZ"),
                    help="Moments of inertia about the CoG [kg m^2], for the roll "
                         "and pitch natural periods")
    args = ap.parse_args(argv)

    if not args.h5.is_file():
        print(f"Missing {args.h5}")
        return 1
    inertia = tuple(args.body_inertia) if args.body_inertia else None
    ok = check(args.h5, args.waterplane_area, args.disp_vol, args.tol_pct,
               args.body_mass, inertia)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
