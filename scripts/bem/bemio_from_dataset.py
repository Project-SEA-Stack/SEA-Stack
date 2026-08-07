"""Write a BEMIO-convention HDF5 file from an in-memory Capytaine dataset.

Mirrors WEC-Sim BEMIO (readCAPYTAINE → normalizeBEM → radiationIRF →
excitationIRF → writeBEMIOH5) using ``capytaine.BEMSolver.fill_dataset``
output plus explicit per-body hydrostatics.

Conventions:
  - Wave directions in degrees
  - Capytaine e^{-iωt} → WAMIT/BEMIO e^{+iωt}: negate Im of scattering and FK
  - Normalisation: C/(ρg), A/ρ, B/(ρω), forces/(ρg)
  - Radiation state-space (ss_*) omitted

This is the reference implementation for SEA-Stack demo assets; new BEM
scripts should import from here rather than writing their own HDF5 writer.
The imaginary-sign flip in ``_forces_from_dataset`` is the piece most easily
got wrong: omitting it mirrors every excitation phase and time-reverses the
excitation IRF. Older per-demo writers (wigley, trimaran) predate this module
and do not apply it.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import h5py
import numpy as np
from scipy.integrate import trapezoid

logger = logging.getLogger(__name__)

STD_DOFS = ("Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw")


@dataclass
class BodyHydrostatics:
    """Hydrostatics for one body in SI units (not BEMIO-normalised).

    Attributes
    ----------
    name
        Body name (``properties/name``).
    disp_volume
        Displaced volume [m^3].
    center_of_gravity
        Centre of mass, shape ``(3,)`` [m].
    center_of_buoyancy
        Centre of buoyancy, shape ``(3,)`` [m].
    hydrostatic_stiffness
        6×6 restoring stiffness in physical units.
    """

    name: str
    disp_volume: float
    center_of_gravity: np.ndarray
    center_of_buoyancy: np.ndarray
    hydrostatic_stiffness: np.ndarray


# ---------------------------------------------------------------------------
# Build the (normalised) hydro dict from an in-memory Capytaine dataset
# ---------------------------------------------------------------------------

def _as_influenced_radiating_freq(dataset, var_name: str) -> np.ndarray:
    """Return ``var[influenced, radiating, omega]`` for a real BEM matrix.

    Capytaine stores added mass / radiation damping with dims
    ``(omega, radiating_dof, influenced_dof)``; ``readCAPYTAINE.m`` uses
    ``(influenced, radiating, omega)``.
    """
    da = dataset[var_name]
    da = da.transpose("influenced_dof", "radiating_dof", "omega")
    return np.asarray(da.values, dtype=float)


def _forces_from_dataset(dataset, var_name: str) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(re, im)`` of a complex force as ``[influenced, heading, omega]``.

    Applies the Capytaine -> BEMIO imaginary-sign flip: the stored imaginary
    part is the *negative* of Capytaine's imaginary part, matching
    ``fk_im = -FK(:,:,:,i_im)`` / ``sc_im = -DF(:,:,:,i_im)`` in
    ``readCAPYTAINE.m``.
    """
    da = dataset[var_name]
    da = da.transpose("influenced_dof", "wave_direction", "omega")
    z = np.asarray(da.values)
    re = np.real(z).astype(float)
    im = -np.imag(z).astype(float)
    return re, im


def build_hydro_from_dataset(
    dataset,
    body_hydrostatics: list[BodyHydrostatics],
) -> dict[str, Any]:
    """Assemble a normalised ``hydro`` dict mirroring the WEC-Sim struct.

    Parameters
    ----------
    dataset
        Capytaine dataset from ``BEMSolver.fill_dataset`` with complex forces
        intact. Must contain ``added_mass``, ``radiation_damping``,
        ``diffraction_force`` and ``Froude_Krylov_force`` and the coords
        ``omega`` and ``wave_direction`` (radians).
    body_hydrostatics
        One :class:`BodyHydrostatics` per body, in body order. The 6x6
        hydrostatic-stiffness blocks are placed on the diagonal of the combined
        stiffness; cross-body hydrostatic coupling is zero by construction.

    Returns
    -------
    dict
        Normalised hydro data with WAMIT-convention keys (``A``, ``B``,
        ``Ainf``, ``Khs``, ``ex_*``, ``sc_*``, ``fk_*`` ...).
    """
    rho = float(np.asarray(dataset["rho"].values).item())
    g = float(np.asarray(dataset["g"].values).item())
    depth = float(np.asarray(dataset["water_depth"].values).item())

    omega = np.asarray(dataset["omega"].values, dtype=float)
    theta_rad = np.asarray(dataset["wave_direction"].values, dtype=float)

    nb = len(body_hydrostatics)
    n_dof_total = 6 * nb

    # --- radiation matrices, (influenced, radiating, omega) --------------
    A = _as_influenced_radiating_freq(dataset, "added_mass")
    B = _as_influenced_radiating_freq(dataset, "radiation_damping")
    if A.shape[0] != n_dof_total:
        raise ValueError(
            f"Dataset has {A.shape[0]} influenced DOFs but {nb} bodies "
            f"({n_dof_total} DOFs) were supplied via body_hydrostatics."
        )

    # --- complex forces, (influenced, heading, omega) --------------------
    sc_re, sc_im = _forces_from_dataset(dataset, "diffraction_force")
    fk_re, fk_im = _forces_from_dataset(dataset, "Froude_Krylov_force")
    ex_re = sc_re + fk_re
    ex_im = sc_im + fk_im

    nf = omega.size
    nh = theta_rad.size

    hydro: dict[str, Any] = {
        "code": "CAPYTAINE",
        "body": [bh.name for bh in body_hydrostatics],
        "Nb": nb,
        "rho": rho,
        "g": g,
        "h": depth,
        "Nf": nf,
        "Nh": nh,
        "w": omega,
        "T": 2.0 * np.pi / omega,
        "theta": np.rad2deg(theta_rad),
        "dof": np.array([6] * nb, dtype=int),
    }

    # --- hydrostatics ----------------------------------------------------
    cg = np.zeros((3, nb))
    cb = np.zeros((3, nb))
    vo = np.zeros(nb)
    khs = np.zeros((6, 6, nb))
    for m, bh in enumerate(body_hydrostatics):
        cg[:, m] = np.asarray(bh.center_of_gravity, dtype=float).ravel()[:3]
        cb[:, m] = np.asarray(bh.center_of_buoyancy, dtype=float).ravel()[:3]
        vo[m] = float(bh.disp_volume)
        khs[:, :, m] = np.asarray(bh.hydrostatic_stiffness, dtype=float)
    hydro["cg"] = cg
    hydro["cb"] = cb
    hydro["Vo"] = vo
    hydro["Khs"] = khs

    # --- magnitude/phase from signed components (pre-normalisation) ------
    for key, re_, im_ in (("ex", ex_re, ex_im), ("sc", sc_re, sc_im), ("fk", fk_re, fk_im)):
        hydro[f"{key}_re"] = re_
        hydro[f"{key}_im"] = im_
        hydro[f"{key}_ma"] = np.sqrt(re_**2 + im_**2)
        hydro[f"{key}_ph"] = np.arctan2(im_, re_)
    hydro["A"] = A
    hydro["B"] = B

    _drop_nan_frequencies(hydro)
    _normalize(hydro)
    return hydro


def _drop_nan_frequencies(hydro: dict[str, Any]) -> None:
    """Remove frequencies where the BEM returned all-NaN data.

    Mirrors the ``lowFrequencyNanMask`` logic at the end of
    ``readCAPYTAINE.m``: Capytaine can return NaN added mass / damping /
    excitation at the lowest requested frequency. A frequency is dropped when
    *every* entry of the added mass (or damping, or excitation real/imag part)
    is NaN at that frequency.
    """
    A = hydro["A"]
    B = hydro["B"]
    ex_re = hydro["ex_re"]
    ex_im = hydro["ex_im"]
    remove = (
        np.all(np.isnan(A), axis=(0, 1))
        | np.all(np.isnan(B), axis=(0, 1))
        | np.all(np.isnan(ex_re), axis=(0, 1))
        | np.all(np.isnan(ex_im), axis=(0, 1))
    )
    if not np.any(remove):
        return
    keep = ~remove
    dropped = hydro["w"][remove]
    logger.warning(
        "BEM results contain all-NaN data at %d frequency(ies) (%s rad/s); dropping them.",
        int(remove.sum()),
        ", ".join(f"{x:.3f}" for x in dropped),
    )
    hydro["w"] = hydro["w"][keep]
    hydro["T"] = hydro["T"][keep]
    hydro["Nf"] = int(keep.sum())
    for key in ("A", "B"):
        hydro[key] = hydro[key][:, :, keep]
    for base in ("ex", "sc", "fk"):
        for suf in ("re", "im", "ma", "ph"):
            hydro[f"{base}_{suf}"] = hydro[f"{base}_{suf}"][:, :, keep]


def _normalize(hydro: dict[str, Any]) -> None:
    """Normalise in place, following ``normalizeBEM.m`` (non-WAMIT branch).

    Also sorts all frequency-indexed arrays into ascending frequency.
    """
    w = hydro["w"]
    if not np.all(np.diff(w) >= 0):
        order = np.argsort(w)
        hydro["w"] = w[order]
        hydro["T"] = hydro["T"][order]
        for key in ("A", "B"):
            hydro[key] = hydro[key][:, :, order]
        for base in ("ex", "sc", "fk"):
            for suf in ("re", "im", "ma", "ph"):
                hydro[f"{base}_{suf}"] = hydro[f"{base}_{suf}"][:, :, order]
        w = hydro["w"]

    rho = hydro["rho"]
    g = hydro["g"]

    hydro["Khs"] = hydro["Khs"] / (rho * g)
    hydro["A"] = hydro["A"] / rho
    hydro["Ainf"] = hydro["A"][:, :, -1].copy()  # overwritten by radiation_irf
    hydro["B"] = hydro["B"] / (rho * w[None, None, :])
    for base in ("ex", "sc", "fk"):
        for suf in ("re", "im", "ma"):  # phase is not normalised
            hydro[f"{base}_{suf}"] = hydro[f"{base}_{suf}"] / (rho * g)


# ---------------------------------------------------------------------------
# Impulse response functions (mirror radiationIRF.m / excitationIRF.m)
# ---------------------------------------------------------------------------

def radiation_irf(
    hydro: dict[str, Any],
    t_end: float = 100.0,
    n_dt: int = 1001,
    n_dw: int = 1001,
    w_min: float | None = None,
    w_max: float | None = None,
) -> None:
    """Compute the normalised radiation IRF ``K(t)`` and infinite-freq added mass.

    Mirrors ``radiationIRF.m``:

        K_{ij}(t) = (2/pi) * integral B_{ij}(w) cos(w t) dw   (B already / rho)

    The infinite-frequency added mass uses the Ogilvie relation

        Ainf_{ij} = mean_k [ A_{ij}(w_k) + (1/w_k) * integral K_{ij}(t) sin(w_k t) dt ]

    Note: the official MATLAB averages over ``1:length(hydro.w)`` while indexing
    the *interpolated* (length ``n_dw``) arrays, which truncates the average to
    the lowest ``Nf`` of ``n_dw`` frequencies. We deliberately average over the
    full interpolated grid instead (a documented, more correct deviation); the
    direct ``omega=inf`` solve is reported separately as the physical anchor.
    """
    w_data = hydro["w"]
    w_min = float(np.min(w_data)) if w_min is None else float(w_min)
    w_max = float(np.max(w_data)) if w_max is None else float(w_max)
    t = np.linspace(0.0, t_end, n_dt)
    w = np.linspace(w_min, w_max, n_dw)

    B = hydro["B"]
    A = hydro["A"]
    n_dof = B.shape[0]
    cos_wt = np.cos(np.outer(t, w))  # (n_dt, n_dw)
    sin_wt = np.sin(np.outer(w, t))  # (n_dw, n_dt)

    ra_K = np.zeros((n_dof, n_dof, t.size))
    ainf = np.zeros((n_dof, n_dof))
    for i in range(n_dof):
        for j in range(n_dof):
            ra_B = np.interp(w, w_data, B[i, j, :])
            k_ij = (2.0 / np.pi) * trapezoid(ra_B[None, :] * cos_wt * w[None, :], w, axis=1)
            ra_K[i, j, :] = k_ij
            ra_A = np.interp(w, w_data, A[i, j, :])
            # Ogilvie estimate at every interpolated frequency, then average.
            integ = trapezoid(k_ij[None, :] * sin_wt, t, axis=1)  # (n_dw,)
            with np.errstate(divide="ignore", invalid="ignore"):
                ainf_per_w = ra_A + integ / w
            ainf[i, j] = float(np.mean(ainf_per_w[np.isfinite(ainf_per_w)]))

    hydro["ra_K"] = ra_K
    hydro["ra_t"] = t
    hydro["ra_w"] = w
    hydro["Ainf"] = ainf


def excitation_irf(
    hydro: dict[str, Any],
    t_end: float = 100.0,
    n_dt: int = 1001,
    n_dw: int = 1001,
    w_min: float | None = None,
    w_max: float | None = None,
) -> None:
    """Compute the normalised excitation IRF ``f(t)`` (mirrors ``excitationIRF.m``).

        f_{i,h}(t) = (1/pi) * integral [ex_re cos(w t) - ex_im sin(w t)] dw
    """
    w_data = hydro["w"]
    w_min = float(np.min(w_data)) if w_min is None else float(w_min)
    w_max = float(np.max(w_data)) if w_max is None else float(w_max)
    t = np.linspace(-t_end, t_end, n_dt)
    w = np.linspace(w_min, w_max, n_dw)

    ex_re = hydro["ex_re"]
    ex_im = hydro["ex_im"]
    n_dof, n_h, _ = ex_re.shape
    cos_wt = np.cos(np.outer(t, w))  # (n_dt, n_dw)
    sin_wt = np.sin(np.outer(t, w))  # (n_dt, n_dw)

    ex_K = np.zeros((n_dof, n_h, t.size))
    for i in range(n_dof):
        for h in range(n_h):
            re_i = np.interp(w, w_data, ex_re[i, h, :])
            im_i = np.interp(w, w_data, ex_im[i, h, :])
            ex_K[i, h, :] = (1.0 / np.pi) * trapezoid(
                re_i[None, :] * cos_wt - im_i[None, :] * sin_wt, w, axis=1
            )
    hydro["ex_K"] = ex_K
    hydro["ex_t"] = t
    hydro["ex_w"] = w


# ---------------------------------------------------------------------------
# Write BEMIO HDF5 (mirror writeBEMIOH5.m)
# ---------------------------------------------------------------------------

def _write_param(group: h5py.Group, name: str, values: Any, description: str, units: str) -> None:
    ds = group.create_dataset(name, data=np.asarray(values, dtype=np.float64))
    ds.attrs["description"] = description
    ds.attrs["units"] = units


def _scalar(v: float) -> np.ndarray:
    return np.array([[float(v)]], dtype=np.float64)


def _col(v: np.ndarray) -> np.ndarray:
    return np.asarray(v, dtype=np.float64).reshape(-1, 1)


def write_bemio_h5(hydro: dict[str, Any], filename: str | Path) -> Path:
    """Serialise a (normalised, IRF-populated) ``hydro`` dict to a BEMIO ``.h5``.

    ``radiation_irf`` and ``excitation_irf`` must have been called first.
    """
    for required in ("ra_K", "ex_K", "Ainf"):
        if required not in hydro:
            raise ValueError(f"hydro['{required}'] missing; call the IRF functions first.")

    out = Path(filename)
    out.parent.mkdir(parents=True, exist_ok=True)

    nb = hydro["Nb"]
    dof = hydro["dof"]

    with h5py.File(str(out), "w") as f:
        f.create_group("bem_data").create_dataset("code", data=np.bytes_(hydro["code"]))

        sim = f.create_group("simulation_parameters")
        _write_param(sim, "scaled", _scalar(0.0), "", "")
        _write_param(sim, "g", _scalar(hydro["g"]), "Gravitational acceleration", "m/s^2")
        _write_param(sim, "rho", _scalar(hydro["rho"]), "Water density", "kg/m^3")
        _write_param(sim, "T", _col(hydro["T"]), "Wave periods", "s")
        _write_param(sim, "w", _col(hydro["w"]), "Wave frequencies", "rad/s")
        if np.isinf(hydro["h"]):
            sim.create_dataset("water_depth", data=np.bytes_("infinite"))
        else:
            _write_param(sim, "water_depth", _scalar(hydro["h"]), "Water depth", "m")
        _write_param(sim, "wave_dir", _col(hydro["theta"]), "Wave direction", "deg")

        n = 0  # running DOF offset into the combined 6*Nb matrices
        for i in range(nb):
            m = int(dof[i])
            rows = slice(n, n + m)
            bnum = i + 1
            bg = f.create_group(f"body{bnum}")

            pg = bg.create_group("properties")
            pg.create_dataset("name", data=np.bytes_(hydro["body"][i]))
            _write_param(pg, "body_number", _scalar(bnum),
                         "Number of rigid body from the BEM simulation", "")
            _write_param(pg, "cb", _col(hydro["cb"][:, i]), "Center of buoyancy", "m")
            _write_param(pg, "cg", _col(hydro["cg"][:, i]), "Center of gravity", "m")
            _write_param(pg, "disp_vol", _scalar(hydro["Vo"][i]), "Displaced volume", "m^3")
            _write_param(pg, "dof", _scalar(m), "Degrees of freedom", "")
            _write_param(pg, "dof_start", _scalar(n + 1), "Degrees of freedom", "")
            _write_param(pg, "dof_end", _scalar(n + m), "Degrees of freedom", "")

            hc = bg.create_group("hydro_coeffs")
            _write_param(hc, "linear_restoring_stiffness", hydro["Khs"][:, :, i],
                         "Hydrostatic stiffness matrix (normalized by rho*g)", "N/m")

            am = hc.create_group("added_mass")
            _write_param(am, "inf_freq", hydro["Ainf"][rows, :],
                         "Infinite frequency added mass (normalized by rho)", "kg")
            _write_param(am, "all", hydro["A"][rows, :, :],
                         "Added mass (normalized by rho)", "kg")

            exc = hc.create_group("excitation")
            _write_param(exc, "re", hydro["ex_re"][rows, :, :],
                         "Real component of excitation force (normalized by rho*g)", "N")
            _write_param(exc, "im", hydro["ex_im"][rows, :, :],
                         "Imaginary component of excitation force (normalized by rho*g)", "N")
            _write_param(exc, "mag", hydro["ex_ma"][rows, :, :],
                         "Magnitude of excitation force (normalized by rho*g)", "N")
            _write_param(exc, "phase", hydro["ex_ph"][rows, :, :],
                         "Phase angle of excitation force", "rad")

            sca = exc.create_group("scattering")
            _write_param(sca, "re", hydro["sc_re"][rows, :, :],
                         "Real component of scattering force (normalized by rho*g)", "N")
            _write_param(sca, "im", hydro["sc_im"][rows, :, :],
                         "Imaginary component of scattering force (normalized by rho*g)", "N")
            _write_param(sca, "mag", hydro["sc_ma"][rows, :, :],
                         "Magnitude of scattering force (normalized by rho*g)", "N")
            _write_param(sca, "phase", hydro["sc_ph"][rows, :, :],
                         "Phase angle of scattering force", "rad")

            fkg = exc.create_group("froude-krylov")
            _write_param(fkg, "re", hydro["fk_re"][rows, :, :],
                         "Real component of Froude-Krylov force (normalized by rho*g)", "N")
            _write_param(fkg, "im", hydro["fk_im"][rows, :, :],
                         "Imaginary component of Froude-Krylov force (normalized by rho*g)", "N")
            _write_param(fkg, "mag", hydro["fk_ma"][rows, :, :],
                         "Magnitude of Froude-Krylov force (normalized by rho*g)", "N")
            _write_param(fkg, "phase", hydro["fk_ph"][rows, :, :],
                         "Phase angle of Froude-Krylov force", "rad")

            ei = exc.create_group("impulse_response_fun")
            _write_param(ei, "f", hydro["ex_K"][rows, :, :], "Impulse response function", "")
            _write_param(ei, "t", _col(hydro["ex_t"]),
                         "Time vector for the impulse resonse function", "s")
            _write_param(ei, "w", _col(hydro["ex_w"]),
                         "Interpolated frequencies used to compute the impulse response function",
                         "rad/s")

            rad = hc.create_group("radiation_damping")
            _write_param(rad, "all", hydro["B"][rows, :, :],
                         "Radiation damping (normalized by rho*w)", "N-s/m")
            ri = rad.create_group("impulse_response_fun")
            _write_param(ri, "K", hydro["ra_K"][rows, :, :], "Impulse response function", "")
            _write_param(ri, "t", _col(hydro["ra_t"]),
                         "Time vector for the impulse resonse function", "s")
            _write_param(ri, "w", _col(hydro["ra_w"]),
                         "Interpolated frequencies used to compute the impulse response function",
                         "rad/s")

            n += m

    logger.info("Wrote BEMIO HDF5 to %s", out)
    return out
