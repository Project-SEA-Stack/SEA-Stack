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
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# Try importing helpers from the regression compare_template; fall back to
# local implementations so the module remains self-contained.
try:
    _ct_dir = str(Path(__file__).resolve().parent.parent.parent / 'regression' / 'utilities')
    if _ct_dir not in sys.path:
        sys.path.insert(0, _ct_dir)
    from compare_template import (
        get_seastack_version,
        get_chrono_version,
        format_path,
        _EXTRA_PALETTE,
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

    Returns:
        dict with summary metrics (max_amp_rel_err, mean_amp_rel_err,
        max_phase_err_rad).
    """
    if not HAS_MPL:
        print(f"  Warning: matplotlib not available, skipping {output_path}")
        return {}

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

    def _plot_series(ax, x, values, color, label, use_markers,
                     ls='-', marker='o'):
        if use_markers:
            ax.plot(x, values, color=color, marker=marker, markersize=5,
                    linewidth=1.4, linestyle=ls, label=label, alpha=0.9)
        else:
            ax.plot(x, values, color=color, linewidth=1.8,
                    label=label, alpha=0.9, linestyle=ls)

    # ── Amplitude panel (positions mirror compare_template.py main/error) ─
    ax_amp = fig.add_axes([0.05, 0.45, 0.78, 0.28])
    _plot_series(ax_amp, ref_period, ref_amp, _REF_COLOR, ref_label,
                 use_markers_ref, ls='-', marker='o')
    _plot_series(ax_amp, sim_period, sim_amp, _SIM_COLOR, sim_label,
                 use_markers_sim, ls='--', marker='s')
    if extra_sources:
        for i, (name, (w, a, _p)) in enumerate(extra_sources.items()):
            color = _EXTRA_PALETTE[i % len(_EXTRA_PALETTE)]
            marker = _EXTRA_MARKERS[i % len(_EXTRA_MARKERS)]
            use_m = len(w) <= 30
            x_period = 2.0 * np.pi / np.asarray(w)
            ax_amp.plot(x_period, a, color=color,
                        marker=marker if use_m else None,
                        markersize=4, linewidth=1.0, alpha=0.7,
                        linestyle='-', label=name)
    ax_amp.set_ylabel(amp_label, fontsize=11, color=_TEXT_MUTED)
    ax_amp.set_title('RAO Amplitude', fontsize=13, fontweight='bold',
                     color=_TEXT_COLOR, pad=8)
    ncol = 1
    if extra_sources:
        ncol = min(2 + len(extra_sources), 4)
    ax_amp.legend(fontsize=9, framealpha=0.9, loc='best', ncol=ncol)
    _apply_style(ax_amp)

    # ── Error panel (relative amplitude error vs wave period) ─────────────
    ax_err = fig.add_axes([0.05, 0.05, 0.78, 0.28])
    if amp_errors is not None and len(amp_errors) > 0:
        amp_errors_arr = np.asarray(amp_errors)
        ax_err.plot(ref_period, amp_errors_arr, color=_SIM_COLOR, marker='s',
                    markersize=5, linewidth=1.4, linestyle='--', alpha=0.9)
        ax_err.axhline(y=0.1, color=_GRID_COLOR, linestyle=':', linewidth=1, alpha=0.7)
        ax_err.axhline(y=0.25, color=_GRID_COLOR, linestyle=':', linewidth=1, alpha=0.5)
    ax_err.set_xlabel('Wave Period (s)', fontsize=11, color=_TEXT_MUTED)
    ax_err.set_ylabel('Relative amplitude error', fontsize=11, color=_TEXT_MUTED)
    ax_err.set_title('Amplitude Error (SEA-Stack vs reference)', fontsize=13, fontweight='bold',
                     color=_TEXT_COLOR, pad=8)
    ax_err.set_ylim(bottom=0)
    _apply_style(ax_err)

    # ── RAO Metrics panel ─────────────────────────────────────────────────
    out_metrics = {}
    lines = ["RAO Metrics\n"]

    if amp_errors is not None and len(amp_errors) > 0:
        amp_errors = np.asarray(amp_errors)
        max_ae = float(np.max(amp_errors))
        mean_ae = float(np.mean(amp_errors))
        out_metrics['max_amp_rel_err'] = max_ae
        out_metrics['mean_amp_rel_err'] = mean_ae
        lines.append(f"Max rel amp err:  {max_ae:.1%}")
        lines.append(f"Mean rel amp err: {mean_ae:.1%}")

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
        detail = ["Per-Point Amplitude Error\n"]
        for i, ae in enumerate(amp_errors):
            tick = '\u2713' if ae <= 0.10 else ('\u26a0' if ae <= 0.25 else '\u2717')
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
