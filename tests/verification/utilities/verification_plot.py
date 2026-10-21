#!/usr/bin/env python3
"""
Polished verification plot utilities for SEA-Stack.

Provides RAO-specific verification plotting with a metadata-rich report-page
layout matching the time-series comparison template in compare_template.py.

  - create_verification_rao_plot: RAO frequency-domain (amplitude + error +
        metadata panels + extra-source overlays)

Time-series verification plots use create_comparison_plot from
tests/regression/utilities/compare_template.py instead.
"""

import platform
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# Try importing the shared plot helpers (tests/utilities/plot_helpers.py); fall
# back to local implementations so the module remains self-contained.
try:
    _helpers_dir = str(Path(__file__).resolve().parent.parent.parent / 'utilities')
    if _helpers_dir not in sys.path:
        sys.path.insert(0, _helpers_dir)
    from plot_helpers import (
        get_seastack_version,
        get_chrono_version,
        format_path,
        EXTRA_PALETTE as _EXTRA_PALETTE,
    )
except ImportError:
    def get_seastack_version():
        return 'Unknown'
    def get_chrono_version():
        return 'Unknown'
    def format_path(path):
        return str(path) if path else 'Not specified'
    _EXTRA_PALETTE = [
        '#2ca02c', '#ff7f0e', '#9467bd', '#8c564b',
        '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
    ]

# ---------------------------------------------------------------------------
# Shared style constants (matching regression compare_template.py aesthetic)
# ---------------------------------------------------------------------------
_REF_COLOR = '#007bff'
_SIM_COLOR = '#dc3545'
_GRID_COLOR = '#6c757d'
_SPINE_COLOR = '#dee2e6'
_PANEL_BG = '#f8f9fa'
_PANEL_BORDER = '#e9ecef'
_TEXT_COLOR = '#212529'
_TEXT_MUTED = '#495057'

_EXTRA_MARKERS = ['v', 'D', '^', 'P', 'X', '<', '>']


def _apply_style(ax):
    """Apply consistent modern styling to an axis."""
    ax.grid(True, alpha=0.2, color=_GRID_COLOR, linewidth=0.5)
    ax.tick_params(labelsize=9, colors=_TEXT_MUTED)
    for spine in ['top', 'right', 'left', 'bottom']:
        ax.spines[spine].set_visible(True)
        ax.spines[spine].set_color(_SPINE_COLOR)
        ax.spines[spine].set_linewidth(1.0)
    ax.set_facecolor('#ffffff')


def _add_panel(fig, pos, text, *, text_color=_TEXT_COLOR, font_size=9):
    """Add a text panel with a rounded box at absolute figure position."""
    ax = fig.add_axes(pos)
    ax.axis('off')
    ax.text(
        0.05, 0.95, text,
        transform=ax.transAxes, verticalalignment='top',
        bbox=dict(boxstyle='round,pad=0.6', facecolor=_PANEL_BG,
                  edgecolor=_PANEL_BORDER, linewidth=1.5, alpha=0.95),
        fontsize=font_size, family='monospace', fontweight='normal',
        color=text_color,
    )


# ---------------------------------------------------------------------------
# RAO verification plot -- full report page
# ---------------------------------------------------------------------------

def create_verification_rao_plot(
    ref_omega, ref_amp, ref_phase,
    sim_omega, sim_amp, sim_phase, *,
    title, amp_label, output_path,
    ref_label='Reference', sim_label='SEA-Stack',
    amp_errors=None, phase_errors=None,
    ref_file_path=None, test_file_path=None,
    extra_sources=None,
    error_thresholds=(0.10, 0.25),
    legend_kwargs=None,
    quantity=None,
    connect_points=True,
    show_extra_errors=False,
):
    """Create a polished RAO verification figure with full report-page layout.

    The layout mirrors the time-series comparison page from compare_template.py:
    header panels (test info + system info), two data panels (amplitude + error),
    and right-side metric/detail panels.

    Args:
        ref_omega:      Reference frequency array (rad/s).
        ref_amp:        Reference amplitude array.
        ref_phase:      Reference phase array (rad), or None.
        sim_omega:      Simulation frequency array (rad/s).
        sim_amp:        Simulation amplitude array.
        sim_phase:      Simulation phase array (rad), or None.
        title:          Plot title string.
        amp_label:      Y-axis label for amplitude panel.
        output_path:    File path for the saved figure.
        ref_label:      Legend label for reference data.
        sim_label:      Legend label for simulation data.
        amp_errors:     Per-frequency relative amplitude errors (0-1 scale).
        phase_errors:   Per-frequency absolute phase errors (rad).
        ref_file_path:  Path to reference data file (for test info panel).
        test_file_path: Path to simulation results file (for test info panel).
        extra_sources:  Optional dict mapping source name to
                        ``(omega, amp, phase)`` tuple.  ``phase`` may be None.
        error_thresholds: ``(pass, warn)`` relative amplitude errors drawn
                        as dotted lines and used for the per-point ticks.
        legend_kwargs:  Optional overrides for the amplitude-panel legend
                        (e.g. ``loc``, ``ncol``) when the default covers data.
        quantity:       Name of the plotted quantity when it is not an RAO
                        amplitude (e.g. ``'Mean PTO Power'``); used in the
                        panel titles and metric labels.
        connect_points: If False, reference and simulation points are drawn
                        as markers only, so a dense curve in
                        ``extra_sources`` shows the shape between them.
        show_extra_errors: If True, the error panel also shows the relative
                        error of each sparse ``extra_sources`` series that is
                        sampled at the reference frequencies.

    Returns:
        dict with summary metrics (max_amp_rel_err, mean_amp_rel_err,
        max_phase_err_rad).
    """
    if not HAS_MPL:
        print(f"  Warning: matplotlib not available, skipping {output_path}")
        return {}

    if quantity is None:
        main_title, err_name, metrics_title, err_abbr = (
            'RAO Amplitude', 'Amplitude', 'RAO Metrics', 'amp')
    else:
        main_title, err_name, metrics_title, err_abbr = (
            quantity, quantity, 'Metrics', '')
    err_word = f"{err_abbr} err" if err_abbr else "err"

    fig = plt.figure(figsize=(12, 9), facecolor='white')

    # ── Test Information panel (matches compare_template.py layout) ───────
    info_content = (
        f"Test Information\n\n"
        f"Test: {title}\n"
        f"Reference File: {format_path(ref_file_path)}\n"
        f"Results File: {format_path(test_file_path)}\n"
        f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
    )
    _add_panel(fig, [0.02, 0.82, 0.8, 0.12], info_content, font_size=11)

    # ── System Information panel ───────────────────────────────────────────
    platform_name = {"Windows": "Windows", "Darwin": "macOS"}.get(
        platform.system(), "Linux")
    python_version = (f"{sys.version_info.major}.{sys.version_info.minor}"
                      f".{sys.version_info.micro}")
    seastack_version = get_seastack_version()
    chrono_version = get_chrono_version()

    sysinfo_content = (
        f"System Information\n\n"
        f"Platform: {platform_name}\n"
        f"Python: {python_version}\n"
        f"SEA-Stack: {seastack_version}\n"
        f"Chrono: {chrono_version}"
    )
    _add_panel(fig, [0.85, 0.82, 0.22, 0.12], sysinfo_content, font_size=8)

    # ── Convert frequency to wave period for x-axis ────────────────────
    ref_period = 2.0 * np.pi / ref_omega
    sim_period = 2.0 * np.pi / sim_omega

    # ── Marker heuristic ──────────────────────────────────────────────────
    use_markers_ref = len(ref_omega) <= 30
    use_markers_sim = len(sim_omega) <= 30

    # One entry per plotted series:
    # (label, period, value, colour, marker, line style, draw markers)
    series = [
        (ref_label, ref_period, np.asarray(ref_amp, dtype=float), _REF_COLOR, 'o',
         '-' if connect_points else 'none', use_markers_ref),
        (sim_label, sim_period, np.asarray(sim_amp, dtype=float), _SIM_COLOR, 's',
         '--' if connect_points else 'none', use_markers_sim),
    ]
    extra_marker_style = {}
    if extra_sources:
        for i, (name, (w, a, _p)) in enumerate(extra_sources.items()):
            use_m = len(w) <= 30
            color = _EXTRA_PALETTE[i % len(_EXTRA_PALETTE)]
            marker = _EXTRA_MARKERS[i % len(_EXTRA_MARKERS)]
            extra_marker_style[name] = (color, marker)
            series.append((name, 2.0 * np.pi / np.asarray(w, dtype=float),
                           np.asarray(a, dtype=float), color, marker,
                           '-' if (connect_points or not use_m) else 'none', use_m))

    def _markers_coincide(sets, tol=0.02):
        if len(sets) < 2:
            return False
        x0 = np.asarray(sets[0][0], dtype=float)
        y0 = np.asarray(sets[0][1], dtype=float)
        scale = np.maximum(np.abs(y0), 1e-9)
        for x, y in sets[1:]:
            x = np.asarray(x, dtype=float)
            y = np.asarray(y, dtype=float)
            order = np.argsort(x)
            y_i = np.interp(x0, x[order], y[order])
            if np.any(np.abs(y_i - y0) / scale > tol):
                return False
        return True

    # When the series agree closely, their markers land on the same spot and
    # hide each other. Offset each series' markers sideways by a small, fixed
    # step (lines stay at the true period) so every method stays visible.
    marker_series = [s for s in series if s[6]]
    dodge = _markers_coincide([(s[1], s[2]) for s in marker_series])
    offsets = {}
    if dodge:
        all_periods = np.concatenate([s[1] for s in marker_series])
        step = 0.012 * max(np.ptp(all_periods), 1e-9)
        n = len(marker_series)
        offsets = {s[0]: (k - 0.5 * (n - 1)) * step for k, s in enumerate(marker_series)}

    # ── Amplitude panel (positions mirror compare_template.py main/error) ─
    ax_amp = fig.add_axes([0.05, 0.45, 0.78, 0.28])
    handles = []
    for label, x, y, color, marker, ls, use_m in series:
        order = np.argsort(x)
        x, y = x[order], y[order]
        if ls != 'none':
            ax_amp.plot(x, y, color=color, linestyle=ls, alpha=0.9, zorder=2,
                        linewidth=1.4 if use_m else 1.8)
        if use_m:
            ax_amp.plot(x + offsets.get(label, 0.0), y, linestyle='none',
                        marker=marker, color=color, alpha=0.95, zorder=3,
                        markersize=6 if dodge else 5,
                        markeredgecolor='white' if dodge else color,
                        markeredgewidth=0.7 if dodge else 1.0)
        handles.append(Line2D([], [], color=color, linestyle=ls, linewidth=1.4,
                              marker=marker if use_m else None, markersize=6))
    ax_amp.set_ylabel(amp_label, fontsize=11, color=_TEXT_MUTED)
    ax_amp.set_title(main_title, fontsize=13, fontweight='bold',
                     color=_TEXT_COLOR, pad=8)
    ncol = 1
    if extra_sources:
        ncol = min(2 + len(extra_sources), 4)
    legend_args = dict(fontsize=9, framealpha=0.9, loc='best', ncol=ncol)
    legend_args.update(legend_kwargs or {})
    ax_amp.legend(handles, [s[0] for s in series], **legend_args)
    if dodge:
        ax_amp.text(1.0, 1.015, 'Markers offset sideways for clarity',
                    transform=ax_amp.transAxes, ha='right', va='bottom',
                    fontsize=8, color=_TEXT_MUTED, style='italic')
    _apply_style(ax_amp)

    # ── Error panel (relative amplitude error vs wave period) ─────────────
    ax_err = fig.add_axes([0.05, 0.05, 0.78, 0.28])
    if amp_errors is not None and len(amp_errors) > 0:
        amp_errors_arr = np.asarray(amp_errors)
        ax_err.plot(ref_period + offsets.get(sim_label, 0.0), amp_errors_arr,
                    color=_SIM_COLOR, marker='s', markersize=5, linewidth=1.4,
                    linestyle='--', alpha=0.9, label=sim_label)
        if show_extra_errors and extra_sources:
            ref_amp_arr = np.asarray(ref_amp, dtype=float)
            for name, (w, a, _p) in extra_sources.items():
                w = np.asarray(w, dtype=float)
                if len(w) > 30:
                    continue
                idx = [int(np.argmin(np.abs(w - w_r))) for w_r in ref_omega]
                if not np.allclose(w[idx], ref_omega, rtol=1e-6):
                    continue
                color, marker = extra_marker_style[name]
                err = np.abs(np.asarray(a, dtype=float)[idx] / ref_amp_arr - 1.0)
                ax_err.plot(ref_period + offsets.get(name, 0.0), err, color=color,
                            marker=marker, markersize=5, linewidth=1.2,
                            linestyle='--', alpha=0.9, label=name)
            ax_err.legend(fontsize=8, framealpha=0.9, loc='upper right')
        ax_err.axhline(y=error_thresholds[0], color=_GRID_COLOR, linestyle=':', linewidth=1, alpha=0.7)
        ax_err.axhline(y=error_thresholds[1], color=_GRID_COLOR, linestyle=':', linewidth=1, alpha=0.5)
    ax_err.set_xlabel('Wave Period (s)', fontsize=11, color=_TEXT_MUTED)
    err_ylabel = 'Relative amplitude error' if quantity is None else 'Relative error'
    ax_err.set_ylabel(err_ylabel, fontsize=11, color=_TEXT_MUTED)
    ax_err.set_title(f'{err_name} Error (SEA-Stack vs reference)', fontsize=13, fontweight='bold',
                     color=_TEXT_COLOR, pad=8)
    ax_err.set_ylim(bottom=0)
    _apply_style(ax_err)

    # ── RAO Metrics panel ─────────────────────────────────────────────────
    out_metrics = {}
    lines = [f"{metrics_title}\n"]

    if amp_errors is not None and len(amp_errors) > 0:
        amp_errors = np.asarray(amp_errors)
        max_ae = float(np.max(amp_errors))
        mean_ae = float(np.mean(amp_errors))
        out_metrics['max_amp_rel_err'] = max_ae
        out_metrics['mean_amp_rel_err'] = mean_ae
        lines.append(f"Max rel {err_word}:  {max_ae:.1%}")
        lines.append(f"Mean rel {err_word}: {mean_ae:.1%}")

    if phase_errors is not None and len(phase_errors) > 0:
        phase_errors = np.asarray(phase_errors)
        max_pe = float(np.max(phase_errors))
        out_metrics['max_phase_err_rad'] = max_pe
        lines.append(f"Max phase err:    {max_pe:.3f} rad")
        lines.append(f"  ({np.degrees(max_pe):.1f}\u00b0)")

    if len(ref_omega) > 0 and len(sim_omega) > 0:
        lines.append(f"\nRef points:  {len(ref_omega)}")
        lines.append(f"Sim points:  {len(sim_omega)}")

    _add_panel(fig, [0.85, 0.63, 0.22, 0.15], '\n'.join(lines),
               text_color=_TEXT_COLOR, font_size=11)

    # ── Per-Point Error detail panel ──────────────────────────────────────
    if amp_errors is not None and len(amp_errors) > 0:
        detail = [f"Per-Point {err_name} Error\n"]
        for i, ae in enumerate(amp_errors):
            tick = ('\u2713' if ae <= error_thresholds[0]
                    else ('\u26a0' if ae <= error_thresholds[1] else '\u2717'))
            detail.append(f"  {tick} {ae:.1%}")
            if i >= 15:
                detail.append(f"  ... ({len(amp_errors) - i - 1} more)")
                break
        _add_panel(fig, [0.85, 0.07, 0.22, 0.53], '\n'.join(detail),
                   font_size=8)

    # ── Save ──────────────────────────────────────────────────────────────
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.close(fig)
    print(f"  Saved verification RAO plot: {output_path}")

    return out_metrics
