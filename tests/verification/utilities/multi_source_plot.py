#!/usr/bin/env python3
"""
Multi-source overlay plotting for SEA-Stack verification.

Provides two main functions:
  - plot_multi_source_overlay: time-series overlay of all sources
  - plot_multi_source_rao: RAO amplitude + phase overlay of all sources

SEA-Stack is always drawn in black/bold; other sources use tab10 colours.
"""

import os
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


_SEASTACK_STYLE = dict(color='black', linewidth=2.0, zorder=10)
_PALETTE = [
    '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
    '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
]


def _source_style(idx):
    return dict(color=_PALETTE[idx % len(_PALETTE)], linewidth=1.2, alpha=0.85)


def plot_multi_source_overlay(
    time_series,
    *,
    title,
    y_label,
    output_path,
    baseline_key=None,
    time_range=None,
):
    """Plot overlay of multiple time-series sources on one figure.

    Args:
        time_series: dict mapping source name -> (t_array, y_array).
        title:       Figure title.
        y_label:     Y-axis label (e.g. "Heave (m)").
        output_path: Path for the output PNG.
        baseline_key: If set, add a lower subplot with differences vs this source.
        time_range:  Optional (t_min, t_max) to clip display range.
    """
    if not HAS_MPL:
        print(f"  Warning: matplotlib not available, skipping plot {output_path}")
        return

    has_baseline = baseline_key and baseline_key in time_series
    n_panels = 2 if has_baseline else 1
    fig, axes = plt.subplots(n_panels, 1, figsize=(12, 4 * n_panels),
                             sharex=True, squeeze=False)
    ax_main = axes[0, 0]

    ref_idx = 0
    for name, (t, y) in time_series.items():
        if time_range:
            t_lo = time_range[0] if time_range[0] is not None else t.min()
            t_hi = time_range[1] if time_range[1] is not None else t.max()
            mask = (t >= t_lo) & (t <= t_hi)
            t, y = t[mask], y[mask]
        if name == 'SEA-Stack':
            ax_main.plot(t, y, label=name, **_SEASTACK_STYLE)
        else:
            ax_main.plot(t, y, label=name, **_source_style(ref_idx))
            ref_idx += 1

    ax_main.set_ylabel(y_label)
    ax_main.set_title(title)
    ax_main.legend(loc='best', fontsize=8, ncol=min(len(time_series), 4))
    ax_main.grid(True, alpha=0.3)

    if has_baseline:
        ax_diff = axes[1, 0]
        t_base, y_base = time_series[baseline_key]
        ref_idx = 0
        for name, (t, y) in time_series.items():
            if name == baseline_key:
                continue
            t_min = max(t[0], t_base[0])
            t_max = min(t[-1], t_base[-1])
            if time_range:
                if time_range[0] is not None:
                    t_min = max(t_min, time_range[0])
                if time_range[1] is not None:
                    t_max = min(t_max, time_range[1])
            n_pts = min(len(t), len(t_base), 2000)
            tc = np.linspace(t_min, t_max, n_pts)
            yi = np.interp(tc, t, y)
            yb = np.interp(tc, t_base, y_base)
            style = _SEASTACK_STYLE if name == 'SEA-Stack' else _source_style(ref_idx)
            ax_diff.plot(tc, yi - yb, label=f"{name} - {baseline_key}", **style)
            if name != 'SEA-Stack':
                ref_idx += 1

        ax_diff.set_ylabel(f"Difference ({y_label})")
        ax_diff.set_xlabel("Time (s)")
        ax_diff.legend(loc='best', fontsize=8, ncol=min(len(time_series), 4))
        ax_diff.grid(True, alpha=0.3)
    else:
        ax_main.set_xlabel("Time (s)")

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(str(output_path), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved multi-source plot: {output_path}")


def plot_multi_source_rao(
    rao_data,
    *,
    title,
    amp_label,
    output_path,
):
    """Plot RAO amplitude and phase for multiple sources.

    Args:
        rao_data: dict mapping source name -> (omega, amplitude, phase).
                  Phase may be None if not available.
        title:    Figure title.
        amp_label: Y-axis label for amplitude (e.g. "Heave RAO (m/m)").
        output_path: Path for the output PNG.
    """
    if not HAS_MPL:
        print(f"  Warning: matplotlib not available, skipping plot {output_path}")
        return

    has_phase = any(p is not None for _, (_, _, p) in rao_data.items() if len(_) > 0)
    n_panels = 2 if has_phase else 1
    fig, axes = plt.subplots(n_panels, 1, figsize=(10, 4 * n_panels),
                             sharex=True, squeeze=False)
    ax_amp = axes[0, 0]

    ref_idx = 0
    for name, (omega, amp, phase) in rao_data.items():
        if name == 'SEA-Stack':
            style = _SEASTACK_STYLE
            marker_style = dict(color='black', markersize=6, zorder=10)
        else:
            style = _source_style(ref_idx)
            marker_style = dict(color=style['color'], markersize=5)
            ref_idx += 1

        if len(omega) > 30:
            ax_amp.plot(omega, amp, label=name, **style)
        else:
            ax_amp.plot(omega, amp, 'o-', label=name, **marker_style, linewidth=style.get('linewidth', 1.2))

        if has_phase and phase is not None and n_panels > 1:
            ax_phase = axes[1, 0]
            if len(omega) > 30:
                ax_phase.plot(omega, phase, **style)
            else:
                ax_phase.plot(omega, phase, 'o-', **marker_style, linewidth=style.get('linewidth', 1.2))

    ax_amp.set_ylabel(amp_label)
    ax_amp.set_title(title)
    ax_amp.legend(loc='best', fontsize=8)
    ax_amp.grid(True, alpha=0.3)

    if n_panels > 1:
        axes[1, 0].set_ylabel("Phase (rad)")
        axes[1, 0].set_xlabel("Frequency (rad/s)")
        axes[1, 0].grid(True, alpha=0.3)
    else:
        ax_amp.set_xlabel("Frequency (rad/s)")

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(str(output_path), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved multi-source RAO plot: {output_path}")
