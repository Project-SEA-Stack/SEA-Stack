#!/usr/bin/env python3
"""
Human-visible verification / comparison plots for the three external-PTO demos
(RM3 TSDA or OSWEC RSDA) under a shared irregular sea state
(JONSWAP Hs=2 m, Tp=8 s, seed=42).

Produces a *small curated set* of figures in the same visual style as the
SEA-Stack regression / comparison / verification reports
(``tests/utilities/plot_helpers.py``):

  01_cross_case_overview.png   motion, relative rate, actuator, energy
  02_linear_vs_native.png      external vs native damper (confidence plot)
  03_adaptive_controller.png   actuator, adapted c(t), energy (+ saturation)
  04_hydraulic_energy_balance.png  E_abs = dE_gas + E_motor + E_relief
  summary.csv / summary.txt
  external_pto_verification.pdf  (or oswec_external_pto_verification.pdf)

Usage:
  python plot_verification.py --auto --output-dir external_pto_verification
  python plot_verification.py --platform oswec --auto \\
      --output-dir oswec_external_pto_verification
  python plot_verification.py \\
      --linear <h5> --adaptive <h5> --hydraulic <h5> --native <h5> \\
      --output-dir external_pto_verification
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    import h5py  # type: ignore
    import numpy as np
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
except Exception as exc:  # noqa: BLE001
    sys.stderr.write(f"plot_verification requires h5py, numpy, matplotlib: {exc}\n")
    sys.exit(2)

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parents[1]
_DEMOS = _REPO / "data" / "demos" / "run_seastack"
_UTIL = _REPO / "tests" / "utilities"
if str(_UTIL) not in sys.path:
    sys.path.insert(0, str(_UTIL))

from plot_helpers import (  # noqa: E402
    AXIS_STYLE,
    EXTRA_PALETTE,
    FIGURE_DPI,
    SERIES_STYLES,
    apply_modern_style,
)

TIME = "/results/time/time"


@dataclass(frozen=True)
class PlatformSpec:
    """HDF5 paths and axis labels for TSDA (RM3) vs RSDA (OSWEC)."""

    key: str
    title: str
    demo_root: Path
    force: str
    speed: str
    extension: str
    power: str
    energy: str
    motion: str
    motion_col: int
    motion_ylabel: str
    speed_ylabel: str
    force_ylabel: str
    damping_ylabel: str
    native_label: str
    force_diff_ylabel: str
    motion_diff_ylabel: str
    motion_summary_key: str
    pdf_name: str
    twin_dirname: str
    model_stem: str
    default_damping: float
    power_sign_note: str
    motion_tol: float
    actuator_tol: float


PLATFORMS: Dict[str, PlatformSpec] = {
    "rm3": PlatformSpec(
        key="rm3",
        title="RM3",
        demo_root=_DEMOS / "rm3",
        force="/results/model/tsdas/PTO/force_mag",
        speed="/results/model/tsdas/PTO/speed",
        extension="/results/model/tsdas/PTO/extension",
        power="/results/model/tsdas/PTO/absorbed_power",
        energy="/results/model/tsdas/PTO/absorbed_energy",
        motion="/results/model/bodies/body1/position",
        motion_col=2,
        motion_ylabel="Float heave (m)",
        speed_ylabel="PTO relative velocity (m/s)",
        force_ylabel="PTO force (N)",
        damping_ylabel="Damping c (N·s/m)",
        native_label="Native LinearPTO",
        force_diff_ylabel="Force difference (N)",
        motion_diff_ylabel="Heave difference (m)",
        motion_summary_key="heave",
        pdf_name="external_pto_verification.pdf",
        twin_dirname="_visual_native_linpto",
        model_stem="rm3_external_pto",
        default_damping=1.2e6,
        power_sign_note="(-force_mag * speed)",
        motion_tol=2.0e-3,
        actuator_tol=2.0e-2,
    ),
    "oswec": PlatformSpec(
        key="oswec",
        title="OSWEC",
        demo_root=_DEMOS / "oswec",
        force="/results/model/rsdas/PTO/torque_mag",
        speed="/results/model/rsdas/PTO/ang_speed",
        extension="/results/model/rsdas/PTO/angle",
        power="/results/model/rsdas/PTO/absorbed_power",
        energy="/results/model/rsdas/PTO/absorbed_energy",
        motion="/results/model/bodies/body1/orientation_xyz",
        motion_col=1,
        motion_ylabel="Flap pitch (rad)",
        speed_ylabel="PTO angular velocity (rad/s)",
        force_ylabel="PTO torque (N·m)",
        damping_ylabel="Damping c (N·m·s/rad)",
        native_label="Native Chrono RSDA",
        force_diff_ylabel="Torque difference (N·m)",
        motion_diff_ylabel="Pitch difference (rad)",
        motion_summary_key="pitch",
        pdf_name="oswec_external_pto_verification.pdf",
        twin_dirname="_visual_native_rsda",
        model_stem="oswec_external_pto",
        default_damping=1.2e7,
        power_sign_note="(-(T*w))",
        # Larger dt (0.03) than RM3 (0.01) → slightly more explicit-within-step drift.
        motion_tol=1.0e-2,
        actuator_tol=5.0e-2,
    ),
}

CASE_LABELS = {
    "linear": "Linear damper",
    "adaptive": "Adaptive damping",
    "hydraulic": "Hydraulic accumulator PTO",
}
# Medium line weight + distinct linestyles so overlapping irregular-wave
# traces stay readable without looking sparse or ink-heavy.
CASE_STYLE = {
    "linear": {
        "color": SERIES_STYLES["primary"]["color"],
        "linewidth": 1.4,
        "linestyle": "-",
        "alpha": 0.85,
        "label": CASE_LABELS["linear"],
    },
    "adaptive": {
        "color": EXTRA_PALETTE[1],
        "linewidth": 1.4,
        "linestyle": "--",
        "alpha": 0.9,
        "label": CASE_LABELS["adaptive"],
    },
    "hydraulic": {
        "color": EXTRA_PALETTE[0],
        "linewidth": 1.4,
        "linestyle": "-.",
        "alpha": 0.9,
        "label": CASE_LABELS["hydraulic"],
    },
}

# Local overrides of report SERIES_STYLES for this dense irregular-wave PDF.
_LINE = {"linewidth": 1.4, "alpha": 0.85}
_STYLE_EXTERNAL = {
    **SERIES_STYLES["primary"], **_LINE, "linestyle": "-",
}
_STYLE_NATIVE = {
    **SERIES_STYLES["secondary"], **_LINE, "linestyle": "--", "alpha": 0.9,
}
_STYLE_ERROR = {
    **SERIES_STYLES["error"], "linewidth": 1.1, "alpha": 0.8, "linestyle": "-",
}

# Excitation ramp in the shared irregular-wave YAML (seconds).
RAMP_DURATION_S = 60.0


@dataclass
class CaseData:
    name: str
    label: str
    h5: Path
    t: np.ndarray
    motion: np.ndarray  # heave (RM3) or flap pitch (OSWEC)
    extension: np.ndarray
    speed: np.ndarray
    force: np.ndarray  # force (TSDA) or torque (RSDA)
    power: np.ndarray
    energy: np.ndarray
    diag: Optional[Dict[str, np.ndarray]] = None
    notes: List[str] = field(default_factory=list)


@dataclass
class SummaryRow:
    case: str
    peak_force: float
    rms_force: float
    net_energy: float
    mean_power: float
    peak_power: float
    heave_rms: float
    heave_peak: float
    saturation_events: Optional[int]
    relief_events: Optional[int]
    status: str
    extra: str = ""


def read_h5(path: Path, dset: str, col: Optional[int] = None) -> np.ndarray:
    with h5py.File(path, "r") as f:
        if dset not in f:
            raise KeyError(f"{path}: missing dataset {dset}")
        arr = np.asarray(f[dset], dtype=float)
    if col is not None and arr.ndim == 2:
        return arr[:, col]
    return np.asarray(arr).reshape(-1)


def find_results_h5(case_dir: Path) -> Path:
    outs = list((case_dir / "outputs").glob("results.*.h5"))
    if not outs:
        raise FileNotFoundError(f"no results.*.h5 under {case_dir / 'outputs'}")
    # Prefer the newest file (avoids picking a stale results.still.h5).
    return max(outs, key=lambda p: p.stat().st_mtime)


def load_csv(path: Path) -> Dict[str, np.ndarray]:
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)
    if not rows:
        return {}
    return {k: np.asarray([float(r[k]) for r in rows], dtype=float)
            for k in rows[0].keys()}


def discover_diag(h5: Path) -> Optional[Path]:
    cand = h5.parent / "pto_diagnostics.csv"
    return cand if cand.is_file() else None


def load_case(
    name: str,
    h5: Path,
    plat: PlatformSpec,
    diag_path: Optional[Path] = None,
) -> CaseData:
    notes: List[str] = []
    diag = None
    csv_path = diag_path or discover_diag(h5)
    if csv_path is not None and csv_path.is_file():
        diag = load_csv(csv_path)
    elif name in ("adaptive", "hydraulic"):
        notes.append(f"diagnostic CSV not found next to {h5.name}")
    return CaseData(
        name=name, label=CASE_LABELS[name], h5=h5,
        t=read_h5(h5, TIME),
        motion=read_h5(h5, plat.motion, plat.motion_col),
        extension=read_h5(h5, plat.extension),
        speed=read_h5(h5, plat.speed),
        force=read_h5(h5, plat.force),
        power=read_h5(h5, plat.power),
        energy=read_h5(h5, plat.energy),
        diag=diag, notes=notes,
    )


def rms(a: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(a))))


def rms_rel(ref: np.ndarray, sim: np.ndarray) -> float:
    ref_rms = rms(ref)
    if ref_rms == 0.0:
        return rms(sim)
    return rms(sim - ref) / ref_rms


def _label_axis(ax: plt.Axes, ylabel: str, xlabel: Optional[str] = None,
                title: Optional[str] = None) -> None:
    ax.set_ylabel(ylabel, **AXIS_STYLE["ylabel"])
    if xlabel is not None:
        ax.set_xlabel(xlabel, **AXIS_STYLE["xlabel"])
    if title is not None:
        ax.set_title(title, **AXIS_STYLE["title_error"])
    apply_modern_style(ax)


def _plot_kw(style: Dict[str, Any], **overrides: Any) -> Dict[str, Any]:
    kw = {
        "color": style["color"],
        "linewidth": style["linewidth"],
        "linestyle": style["linestyle"],
        "alpha": style["alpha"],
        "label": style.get("label"),
    }
    kw.update(overrides)
    return kw


def _info_box(ax: plt.Axes, text: str, loc: str = "lower right") -> None:
    anchors = {
        "lower right": (0.98, 0.05, "right", "bottom"),
        "lower left": (0.02, 0.05, "left", "bottom"),
        "upper right": (0.98, 0.95, "right", "top"),
        "upper left": (0.02, 0.95, "left", "top"),
    }
    x, y, ha, va = anchors[loc]
    ax.text(
        x, y, text, transform=ax.transAxes, ha=ha, va=va, fontsize=8,
        family="monospace", color="#212529",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#f8f9fa",
                  edgecolor="#e9ecef", linewidth=1.5, alpha=0.95),
    )


def _shade_ramp(ax: plt.Axes, t: np.ndarray) -> None:
    """Shade the excitation-ramp window so post-ramp performance is obvious."""
    t_end = float(t[-1]) if len(t) else RAMP_DURATION_S
    ramp_end = min(RAMP_DURATION_S, t_end)
    if ramp_end <= 0.0:
        return
    ax.axvspan(0.0, ramp_end, color="#adb5bd", alpha=0.18, zorder=0)
    if ramp_end < t_end:
        ax.axvline(ramp_end, color="#adb5bd", linewidth=0.9, linestyle=":",
                   zorder=1)


def _post_ramp_mask(t: np.ndarray) -> np.ndarray:
    return t >= RAMP_DURATION_S


def _mean_power_post_ramp(case: CaseData) -> float:
    mask = _post_ramp_mask(case.t)
    if not np.any(mask):
        return float(np.mean(case.power))
    return float(np.mean(case.power[mask]))


def save_fig(fig: plt.Figure, out_dir: Path, stem: str, pdf: PdfPages) -> Path:
    png = out_dir / f"{stem}.png"
    fig.savefig(png, dpi=FIGURE_DPI, facecolor="white")
    pdf.savefig(fig, dpi=FIGURE_DPI, facecolor="white")
    plt.close(fig)
    return png


# ---------------------------------------------------------------------------
# Curated figures
# ---------------------------------------------------------------------------

def plot_cross_case(cases: Sequence[CaseData], out_dir: Path,
                    pdf: PdfPages, plat: PlatformSpec) -> Path:
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True,
                             facecolor="white")
    fig.suptitle(
        f"External PTO comparison — {plat.title} irregular sea "
        f"(JONSWAP Hs=2 m, Tp=8 s, seed=42)",
        fontsize=13, fontweight="bold", color="#212529", y=0.995,
    )
    panels = [
        (axes[0], plat.motion_ylabel, lambda c: c.motion),
        (axes[1], plat.speed_ylabel, lambda c: c.speed),
        (axes[2], plat.force_ylabel, lambda c: c.force),
        (axes[3], "Absorbed energy (J)", lambda c: c.energy),
    ]
    t0 = cases[0].t if cases else np.array([0.0])
    # Draw solid first, then dashed / dash-dot on top so covered data stays visible.
    draw_order = sorted(cases, key=lambda c: {"linear": 0, "adaptive": 1,
                                              "hydraulic": 2}.get(c.name, 9))
    for ax, ylabel, getter in panels:
        _shade_ramp(ax, t0)
        for i, c in enumerate(draw_order):
            ax.plot(c.t, getter(c), **_plot_kw(CASE_STYLE[c.name]),
                    zorder=2 + i)
        _label_axis(ax, ylabel)
        ax.legend(fontsize=9, framealpha=0.9, loc="upper right")
    axes[3].set_xlabel("Time (s)", **AXIS_STYLE["xlabel"])
    # Compact mean-power callout (post-ramp) for performance comparison.
    lines = ["Mean absorbed power after ramp:"]
    for c in cases:
        lines.append(f"  {c.label}: {_mean_power_post_ramp(c):.3g} W")
    _info_box(axes[3], "\n".join(lines), loc="lower right")
    fig.text(
        0.5, 0.005,
        f"Grey band = excitation ramp (0–{RAMP_DURATION_S:.0f} s). "
        "Same sea state for all three PTO modules.",
        ha="center", va="bottom", fontsize=8, style="italic", color="#495057",
    )
    fig.subplots_adjust(left=0.10, right=0.97, top=0.95, bottom=0.05, hspace=0.16)
    return save_fig(fig, out_dir, "01_cross_case_overview", pdf)


def plot_linear_vs_native(
    linear: CaseData,
    native: Optional[CaseData],
    out_dir: Path,
    pdf: PdfPages,
    plat: PlatformSpec,
) -> Tuple[Path, SummaryRow]:
    motion_err = float("nan")
    force_err = float("nan")
    status = "PASS"
    extra = ""
    mkey = plat.motion_summary_key

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, facecolor="white")
    fig.suptitle(
        f"Linear verification — external damper vs {plat.native_label} "
        "(irregular waves)",
        fontsize=13, fontweight="bold", color="#212529", y=0.98,
    )
    for ax in axes:
        _shade_ramp(ax, linear.t)

    if native is None:
        status = "INCOMPLETE"
        extra = f"{plat.native_label} reference H5 not provided"
        axes[0].plot(linear.t, linear.motion,
                     **_plot_kw(_STYLE_EXTERNAL, label="External"))
        axes[1].plot(linear.t, linear.force,
                     **_plot_kw(_STYLE_EXTERNAL, label="External"))
        axes[2].text(0.5, 0.5, extra, transform=axes[2].transAxes,
                     ha="center", va="center", color="#dc3545")
    else:
        t = linear.t
        n_force = np.interp(t, native.t, native.force)
        n_motion = np.interp(t, native.t, native.motion)
        d_force = linear.force - n_force
        d_motion = linear.motion - n_motion
        force_err = rms_rel(n_force, linear.force)
        motion_err = rms_rel(n_motion, linear.motion)
        if motion_err > plat.motion_tol or force_err > plat.actuator_tol:
            status = "FAIL"
        extra = (f"{mkey}_rms_rel={motion_err:.3e}; "
                 f"actuator_rms_rel={force_err:.3e}")

        axes[0].plot(t, linear.motion,
                     **_plot_kw(_STYLE_EXTERNAL, label="External", zorder=2))
        axes[0].plot(t, n_motion,
                     **_plot_kw(_STYLE_NATIVE, label=plat.native_label,
                                zorder=3))
        axes[1].plot(t, linear.force,
                     **_plot_kw(_STYLE_EXTERNAL, label="External", zorder=2))
        axes[1].plot(t, n_force,
                     **_plot_kw(_STYLE_NATIVE, label=plat.native_label,
                                zorder=3))

        ax_h, ax_f = axes[2], axes[2].twinx()
        h1, = ax_h.plot(t, d_motion,
                        **_plot_kw(_STYLE_ERROR, label=f"{mkey.capitalize()} diff"))
        h2, = ax_f.plot(t, d_force, color="#6f42c1", linewidth=1.1,
                        alpha=0.8, label="Actuator diff")
        ax_h.axhline(0.0, color="#adb5bd", linewidth=0.8)
        ax_h.set_ylabel(plat.motion_diff_ylabel, **AXIS_STYLE["ylabel"])
        ax_f.set_ylabel(plat.force_diff_ylabel, **AXIS_STYLE["ylabel"])
        ax_h.tick_params(labelsize=9, colors="#495057")
        ax_f.tick_params(labelsize=9, colors="#495057")
        apply_modern_style(ax_h)
        ax_f.spines["top"].set_color("#dee2e6")
        ax_f.spines["right"].set_color("#dee2e6")
        ax_h.legend(handles=[h1, h2], fontsize=10, framealpha=0.9,
                    loc="lower right")
        _info_box(
            axes[0],
            f"{mkey.capitalize()} RMS-rel = {motion_err:.3e}  "
            f"(tol {plat.motion_tol:.0e})\n"
            f"Actuator RMS-rel = {force_err:.3e}  "
            f"(tol {plat.actuator_tol:.0e})",
        )

    _label_axis(axes[0], plat.motion_ylabel)
    _label_axis(axes[1], plat.force_ylabel)
    axes[0].legend(fontsize=10, framealpha=0.9, loc="upper right")
    axes[1].legend(fontsize=10, framealpha=0.9, loc="upper right")
    axes[2].set_xlabel("Time (s)", **AXIS_STYLE["xlabel"])
    link_kind = "RSDA" if plat.key == "oswec" else "TSDA"
    fig.text(
        0.5, 0.01,
        "Actuator need not match instantaneously: external module is frozen per "
        f"accepted time level (explicit-within-step HHT); native {link_kind} is "
        "re-evaluated every Newton iteration.",
        ha="center", va="bottom", fontsize=8, style="italic", color="#495057",
    )
    fig.subplots_adjust(left=0.10, right=0.90, top=0.93, bottom=0.08, hspace=0.18)
    path = save_fig(fig, out_dir, "02_linear_vs_native", pdf)

    row = SummaryRow(
        case=linear.label,
        peak_force=float(np.max(np.abs(linear.force))),
        rms_force=rms(linear.force),
        net_energy=float(linear.energy[-1]),
        mean_power=_mean_power_post_ramp(linear),
        peak_power=float(np.max(linear.power)),
        heave_rms=rms(linear.motion),
        heave_peak=float(np.max(np.abs(linear.motion))),
        saturation_events=None, relief_events=None,
        status=status, extra=extra,
    )
    return path, row


def plot_adaptive(
    adaptive: CaseData, out_dir: Path, pdf: PdfPages, plat: PlatformSpec,
) -> Tuple[Path, SummaryRow]:
    diag = adaptive.diag
    sat_events: Optional[int] = None
    status = "PASS"
    extra = ""
    act_short = "torque" if plat.key == "oswec" else "force"

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, facecolor="white")
    fig.suptitle(
        f"Adaptive damping — controller under irregular waves ({plat.title})",
        fontsize=13, fontweight="bold", color="#212529", y=0.98,
    )

    for ax in axes:
        _shade_ramp(ax, adaptive.t)

    if diag is None:
        status = "INCOMPLETE"
        extra = "missing controller CSV"
        axes[0].plot(adaptive.t, adaptive.force,
                     **_plot_kw(CASE_STYLE["adaptive"]), zorder=2)
        axes[1].plot(adaptive.t, adaptive.speed,
                     **_plot_kw(CASE_STYLE["adaptive"], label="Velocity"),
                     zorder=2)
        axes[2].plot(adaptive.t, adaptive.energy,
                     **_plot_kw(CASE_STYLE["adaptive"]), zorder=2)
    else:
        t = diag["time"]
        sat_events = int(diag["saturation_events"][-1])
        axes[0].plot(t, diag["force"],
                     **_plot_kw(CASE_STYLE["adaptive"],
                                label=f"PTO {act_short}"),
                     zorder=2)
        axes[0].plot(t, diag["force_max"], color="#dc3545", linewidth=1.5,
                     linestyle="--", alpha=0.8, label="+limit", zorder=2)
        axes[0].plot(t, -diag["force_max"], color="#dc3545", linewidth=1.5,
                     linestyle="--", alpha=0.8, label="-limit", zorder=2)
        sat = diag["saturated"] > 0.5
        if np.any(sat):
            axes[0].fill_between(
                t, -diag["force_max"], diag["force_max"], where=sat,
                color="#dc3545", alpha=0.12, label="Saturation", zorder=1,
            )
        axes[1].plot(t, diag["damping_c"],
                     color="#6f42c1", linewidth=1.4, alpha=0.9,
                     label="Adapted c(t)", zorder=2)
        axes[2].plot(adaptive.t, adaptive.energy,
                     **_plot_kw(CASE_STYLE["adaptive"]), zorder=2)
        extra = (f"sat_events={sat_events}; "
                 f"mean_P_post_ramp={_mean_power_post_ramp(adaptive):.3g} W")
        if float(adaptive.energy[-1]) <= 0.0:
            status = "FAIL"

    _label_axis(axes[0], plat.force_ylabel)
    _label_axis(axes[1], plat.damping_ylabel if diag is not None
                else plat.speed_ylabel)
    _label_axis(axes[2], "Absorbed energy (J)")
    for ax in axes:
        ax.legend(fontsize=10, framealpha=0.9, loc="best")
    axes[2].set_xlabel("Time (s)", **AXIS_STYLE["xlabel"])
    fig.subplots_adjust(left=0.10, right=0.97, top=0.93, bottom=0.07, hspace=0.18)
    path = save_fig(fig, out_dir, "03_adaptive_controller", pdf)

    row = SummaryRow(
        case=adaptive.label,
        peak_force=float(np.max(np.abs(adaptive.force))),
        rms_force=rms(adaptive.force),
        net_energy=float(adaptive.energy[-1]),
        mean_power=_mean_power_post_ramp(adaptive),
        peak_power=float(np.max(adaptive.power)),
        heave_rms=rms(adaptive.motion),
        heave_peak=float(np.max(np.abs(adaptive.motion))),
        saturation_events=sat_events, relief_events=None,
        status=status, extra=extra,
    )
    return path, row


def plot_hydraulic(
    hydraulic: CaseData, out_dir: Path, pdf: PdfPages, plat: PlatformSpec,
) -> Tuple[Path, SummaryRow]:
    diag = hydraulic.diag
    relief_events: Optional[int] = None
    status = "PASS"
    extra = ""
    max_abs_res = float("nan")
    max_rel_res = float("nan")

    fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=False, facecolor="white")
    fig.suptitle(
        f"Hydraulic accumulator PTO — energy balance "
        f"({plat.title}, irregular waves)",
        fontsize=13, fontweight="bold", color="#212529", y=0.98,
    )

    if diag is None:
        status = "INCOMPLETE"
        extra = "missing hydraulic CSV"
        _shade_ramp(axes[0], hydraulic.t)
        _shade_ramp(axes[1], hydraulic.t)
        axes[0].plot(hydraulic.t, hydraulic.force,
                     **_plot_kw(CASE_STYLE["hydraulic"]), zorder=2)
        axes[1].plot(hydraulic.t, hydraulic.energy,
                     **_plot_kw(CASE_STYLE["hydraulic"]), zorder=2)
        _label_axis(axes[0], plat.force_ylabel)
        _label_axis(axes[1], "Absorbed energy (J)", xlabel="Time (s)")
    else:
        t = diag["time"]
        relief_events = int(diag["relief_events"][-1])
        residual = diag["residual"]
        e_abs = diag["E_abs"]
        e_sum = diag["E_gas"] + diag["E_motor"] + diag["E_relief"]
        max_abs_res = float(np.max(np.abs(residual)))
        scale = max(float(np.max(np.abs(e_abs))), 1.0)
        max_rel_res = max_abs_res / scale
        extra = (f"max_abs_residual={max_abs_res:.3e}; "
                 f"max_rel_residual={max_rel_res:.3e}; "
                 f"mean_P_post_ramp={_mean_power_post_ramp(hydraulic):.3g} W")

        _shade_ramp(axes[0], t)
        _shade_ramp(axes[1], t)
        axes[0].plot(t, e_abs, **_plot_kw(CASE_STYLE["hydraulic"],
                                          label="E_abs (mechanical)"), zorder=2)
        axes[0].plot(t, diag["E_gas"], color=EXTRA_PALETTE[1], linewidth=1.4,
                     linestyle="--", alpha=0.9, label="ΔE_gas (stored)",
                     zorder=3)
        axes[0].plot(t, diag["E_motor"], color=EXTRA_PALETTE[2], linewidth=1.4,
                     linestyle=":", alpha=0.9, label="E_motor", zorder=3)
        axes[0].plot(t, diag["E_relief"], color=EXTRA_PALETTE[3], linewidth=1.4,
                     linestyle="-.", alpha=0.9, label="E_relief", zorder=3)
        axes[0].plot(t, e_sum, color="#212529", linewidth=1.2, linestyle=":",
                     alpha=0.75, label="ΔE_gas+E_motor+E_relief", zorder=4)
        _label_axis(axes[0], "Energy (J)",
                    title="E_abs = ΔE_gas + E_motor + E_relief")
        axes[0].legend(fontsize=9, framealpha=0.9, loc="best")
        _info_box(axes[0],
                  f"max |residual| = {max_abs_res:.3e} J\n"
                  f"max |residual|/max|E_abs| = {max_rel_res:.3e}")

        axes[1].plot(t, residual, **_plot_kw(_STYLE_ERROR, label="Residual"),
                     zorder=2)
        axes[1].axhline(0.0, color="#adb5bd", linewidth=0.8)
        _label_axis(axes[1], "Energy residual (J)", xlabel="Time (s)")
        axes[1].legend(fontsize=10, framealpha=0.9)

        if max_rel_res > 1.0e-8 or float(hydraulic.energy[-1]) <= 0.0:
            status = "FAIL"

    fig.subplots_adjust(left=0.10, right=0.97, top=0.90, bottom=0.08, hspace=0.28)
    path = save_fig(fig, out_dir, "04_hydraulic_energy_balance", pdf)

    row = SummaryRow(
        case=hydraulic.label,
        peak_force=float(np.max(np.abs(hydraulic.force))),
        rms_force=rms(hydraulic.force),
        net_energy=float(hydraulic.energy[-1]),
        mean_power=_mean_power_post_ramp(hydraulic),
        peak_power=float(np.max(hydraulic.power)),
        heave_rms=rms(hydraulic.motion),
        heave_peak=float(np.max(np.abs(hydraulic.motion))),
        saturation_events=None, relief_events=relief_events,
        status=status, extra=extra,
    )
    return path, row


def write_summary(
    rows: Sequence[SummaryRow], out_dir: Path, plat: PlatformSpec,
) -> Tuple[Path, Path]:
    csv_path = out_dir / "summary.csv"
    txt_path = out_dir / "summary.txt"
    mkey = plat.motion_summary_key
    act_unit = "Nm" if plat.key == "oswec" else "N"
    motion_unit = "rad" if plat.key == "oswec" else "m"
    fields = [
        "case", f"peak_actuator_{act_unit}", f"rms_actuator_{act_unit}",
        "net_energy_J", "mean_power_post_ramp_W", "peak_power_W",
        f"{mkey}_rms_{motion_unit}", f"{mkey}_peak_{motion_unit}",
        "saturation_events", "relief_events", "status", "extra",
    ]
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(fields)
        for r in rows:
            w.writerow([
                r.case, f"{r.peak_force:.6g}", f"{r.rms_force:.6g}",
                f"{r.net_energy:.6g}", f"{r.mean_power:.6g}",
                f"{r.peak_power:.6g}",
                f"{r.heave_rms:.6g}", f"{r.heave_peak:.6g}",
                "" if r.saturation_events is None else r.saturation_events,
                "" if r.relief_events is None else r.relief_events,
                r.status, r.extra,
            ])

    motion_hdr = "pitch_rms" if plat.key == "oswec" else "z_rms"
    lines = [
        f"SEA-Stack external PTO visual verification summary ({plat.title})",
        "Sea state: irregular JONSWAP Hs=2 m, Tp=8 s, seed=42, ramp=60 s",
        f"Generated: {datetime.now().isoformat(timespec='seconds')}",
        "=" * 78,
        f"{'Case':<28} {'Peak':>10} {'RMS':>10} {'E_net':>10} "
        f"{'P_mean':>10} {motion_hdr:>9} {'status':>10}",
        "-" * 78,
    ]
    for r in rows:
        lines.append(
            f"{r.case:<28} {r.peak_force:10.3g} {r.rms_force:10.3g} "
            f"{r.net_energy:10.3g} {r.mean_power:10.3g} {r.heave_rms:9.3g} "
            f"{r.status:>10}"
        )
        if r.saturation_events is not None:
            lines.append(f"  saturation_events = {r.saturation_events}")
        if r.relief_events is not None:
            lines.append(f"  relief_events = {r.relief_events}")
        if r.extra:
            lines.append(f"  {r.extra}")
    lines.append("-" * 78)
    lines.append(
        "P_mean is time-mean absorbed power after the excitation ramp "
        f"({RAMP_DURATION_S:.0f} s)."
    )
    lines.append(
        "Absorbed power sign convention (SEA-Stack HDF5): positive = absorbing "
        f"{plat.power_sign_note}."
    )
    lines.append(
        "Plots use the same demo YAML configs as the automated regression tests."
    )
    lines.append(
        "Figure style matches tests/utilities/plot_helpers.py "
        "(regression / comparison / verification reports)."
    )
    txt = "\n".join(lines) + "\n"
    txt_path.write_text(txt, encoding="utf-8")
    print(txt)
    return csv_path, txt_path


def resolve_inputs(
    args: argparse.Namespace, plat: PlatformSpec,
) -> Dict[str, Any]:
    root = plat.demo_root
    if args.auto:
        twin = root / plat.twin_dirname
        return {
            "linear": find_results_h5(root / "external_pto"),
            "adaptive": find_results_h5(root / "external_pto_adaptive"),
            "hydraulic": find_results_h5(root / "external_pto_hydraulic"),
            "native": (
                find_results_h5(twin)
                if (twin / "outputs").exists()
                else None
            ),
            "adaptive_csv": None,
            "hydraulic_csv": None,
        }
    missing = [n for n in ("linear", "adaptive", "hydraulic")
               if getattr(args, n) is None]
    if missing:
        raise SystemExit(
            f"missing required H5 paths: {', '.join(missing)} "
            "(or pass --auto after running the demos)"
        )
    return {
        "linear": Path(args.linear),
        "adaptive": Path(args.adaptive),
        "hydraulic": Path(args.hydraulic),
        "native": Path(args.native) if args.native else None,
        "adaptive_csv": Path(args.adaptive_csv) if args.adaptive_csv else None,
        "hydraulic_csv": Path(args.hydraulic_csv) if args.hydraulic_csv else None,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--platform", choices=sorted(PLATFORMS), default="rm3",
        help="Demo platform: rm3 (TSDA) or oswec (RSDA)",
    )
    parser.add_argument("--linear", type=Path)
    parser.add_argument("--adaptive", type=Path)
    parser.add_argument("--hydraulic", type=Path)
    parser.add_argument("--native", type=Path)
    parser.add_argument("--adaptive-csv", type=Path, default=None)
    parser.add_argument("--hydraulic-csv", type=Path, default=None)
    parser.add_argument("--auto", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args(argv)

    plat = PLATFORMS[args.platform]
    if args.output_dir is None:
        args.output_dir = Path(
            "oswec_external_pto_verification" if plat.key == "oswec"
            else "external_pto_verification"
        )

    paths = resolve_inputs(args, plat)
    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    # Drop stale figures from older plotter versions.
    for old in out_dir.glob("*.png"):
        old.unlink()

    linear = load_case("linear", paths["linear"], plat)
    adaptive = load_case(
        "adaptive", paths["adaptive"], plat, paths["adaptive_csv"],
    )
    hydraulic = load_case(
        "hydraulic", paths["hydraulic"], plat, paths["hydraulic_csv"],
    )
    native = None
    if paths["native"] is not None and Path(paths["native"]).is_file():
        native = load_case("linear", paths["native"], plat)
        native.name = "native"
        native.label = plat.native_label

    pdf_path = out_dir / plat.pdf_name
    written: List[Path] = []
    rows: List[SummaryRow] = []

    with PdfPages(pdf_path) as pdf:
        written.append(
            plot_cross_case([linear, adaptive, hydraulic], out_dir, pdf, plat),
        )
        p, row = plot_linear_vs_native(linear, native, out_dir, pdf, plat)
        written.append(p)
        rows.append(row)
        p, row = plot_adaptive(adaptive, out_dir, pdf, plat)
        written.append(p)
        rows.append(row)
        p, row = plot_hydraulic(hydraulic, out_dir, pdf, plat)
        written.append(p)
        rows.append(row)

    csv_path, txt_path = write_summary(rows, out_dir, plat)
    print(f"Wrote {len(written)} PNG figures ({FIGURE_DPI} DPI) and PDF:")
    for p in written:
        print(f"  {p}")
    print(f"  {pdf_path}")
    print(f"  {csv_path}")
    print(f"  {txt_path}")
    for note in linear.notes + adaptive.notes + hydraulic.notes:
        print(f"NOTE: {note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
