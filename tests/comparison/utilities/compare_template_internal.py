#!/usr/bin/env python3
"""
SEA-Stack Internal Method Comparison Template

Shared utilities for comparing two SEA-Stack methods against each other
(e.g. IRF convolution vs frequency-domain excitation). These comparisons
do NOT use external reference data; they compare two outputs produced by
different code paths within the same simulation run.

Usage:
    from compare_template_internal import run_internal_comparison
"""

import json
import os
import sys
from pathlib import Path

# CTest SKIP_RETURN_CODE 77 — see tests/*/CMakeLists.txt SEASTACK_SKIP_MISSING_PYTHON_DEPS.
try:
    import numpy as np
except ImportError:
    print("Error: numpy is required but not installed. Please install it with: pip install numpy")
    sys.exit(77)

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Import shared plotting helpers
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "utilities"))
try:
    from plot_helpers import (
        apply_modern_style,
        create_standard_figure,
        build_info_panel,
        build_system_panel,
        build_error_metrics_panel,
        build_data_stats_panel,
        build_simulation_summary_panel,
        compute_error_metrics,
        extract_units_from_label,
        SERIES_STYLES,
        AXIS_STYLE,
        FIGURE_DPI,
    )
    HAS_PLOT_HELPERS = True
except ImportError:
    HAS_PLOT_HELPERS = False


def compute_norms(a, b, skip_time=0.0, time=None):
    """Compute L2 and Linf norms between two signals.

    Args:
        a, b:       1-D arrays of equal length.
        skip_time:  Skip initial transient period (requires time array).
        time:       Optional time array for transient skipping.

    Returns:
        (l2_norm, linf_norm)
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    if time is not None and skip_time > 0:
        mask = np.asarray(time) >= skip_time
        a, b = a[mask], b[mask]

    diff = a - b
    l2 = np.sqrt(np.mean(diff ** 2))
    linf = np.max(np.abs(diff))
    return l2, linf


def compute_phase_error(a, b, omega, time, skip_time=0.0):
    """Estimate phase error between two quasi-sinusoidal signals.

    Uses Fourier projection at the dominant frequency.
    """
    if skip_time > 0:
        mask = time >= skip_time
        a, b, time = a[mask], b[mask], time[mask]

    a_cos = 2.0 * np.mean(a * np.cos(omega * time))
    a_sin = 2.0 * np.mean(a * np.sin(omega * time))
    b_cos = 2.0 * np.mean(b * np.cos(omega * time))
    b_sin = 2.0 * np.mean(b * np.sin(omega * time))

    phase_a = np.arctan2(a_sin, a_cos)
    phase_b = np.arctan2(b_sin, b_cos)
    return phase_b - phase_a


def write_comparison_status(output_dir, test_name, metrics, passed=None):
    """Write a .status.json for an internal comparison.

    Comparison tests are informational by default (no pass/fail) unless
    explicit thresholds are provided.
    """
    status_dir = Path(output_dir)
    status_dir.mkdir(parents=True, exist_ok=True)

    from datetime import datetime

    payload = {
        "test_name": test_name,
        "status": "PASS" if passed else ("FAIL" if passed is False else "INFO"),
        "timestamp": datetime.now().isoformat(),
        "metrics": metrics,
    }

    status_file = status_dir / "status.json"
    with open(status_file, 'w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2)

    return payload


def plot_comparison(time, signal_a, signal_b, *, label_a, label_b,
                    title, y_label, output_path, skip_time=0.0, test_name_full=None):
    """Generate overlay + difference comparison plot with standardized LAYOUT.
    
    Args:
        time: Time array (1D)
        signal_a: Signal array A (1D)
        signal_b: Signal array B (1D)
        label_a: Label for signal A
        label_b: Label for signal B
        title: Plot title
        y_label: Y-axis label (e.g., "Heave (m)")
        output_path: Path to save the plot
        skip_time: Skip initial transient period (seconds)
        test_name_full: Optional full test name for info panel (defaults to extracting from title)
    """
    if not HAS_MATPLOTLIB:
        return
    
    if not HAS_PLOT_HELPERS:
        # Fallback to basic plotting if helpers not available
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True,
                                        gridspec_kw={'height_ratios': [3, 1]})
        
        ax1.plot(time, signal_a, label=label_a, linewidth=1.0)
        ax1.plot(time, signal_b, label=label_b, linewidth=1.0, linestyle='--')
        if skip_time > 0:
            ax1.axvline(skip_time, color='gray', linestyle=':', alpha=0.5, label='Skip boundary')
        ax1.set_ylabel(y_label)
        ax1.set_title(title)
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        diff = np.asarray(signal_b) - np.asarray(signal_a)
        ax2.plot(time, diff, color='red', linewidth=0.8)
        ax2.set_ylabel('Difference')
        ax2.set_xlabel('Time (s)')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(str(output_path), dpi=150)
        plt.close()
        return
    
    # Convert signals to Nx2 arrays for compatibility with shared helpers
    data_a = np.column_stack([time, signal_a])
    data_b = np.column_stack([time, signal_b])
    
    # Clip to common time range if needed
    from plot_helpers import clip_to_common_time
    data_a, data_b = clip_to_common_time(data_a, data_b)
    
    # Extract time and signals after clipping
    time_clipped = data_a[:, 0]
    signal_a_clipped = data_a[:, 1]
    signal_b_clipped = data_b[:, 1]
    
    # Compute error metrics using shared helper
    l2_norm, linf_norm, max_error, mean_error = compute_error_metrics(data_a, data_b)
    
    # Create standard figure with LAYOUT
    fig, ax_main, ax_error = create_standard_figure(title)
    
    # Build metadata panels
    test_name = test_name_full if test_name_full else title.split(':')[0] if ':' in title else title
    build_info_panel(fig, test_name, label_a, label_b)
    build_system_panel(fig)
    
    units = extract_units_from_label(y_label)
    build_data_stats_panel(fig, data_a, data_b, label_a, label_b, units)
    build_simulation_summary_panel(fig, data_a, data_b)
    
    metrics_dict = {
        'l2_norm': l2_norm,
        'linf_norm': linf_norm,
        'max_error': max_error,
        'mean_error': mean_error
    }
    build_error_metrics_panel(fig, metrics_dict)
    
    # Plot main comparison
    style_a = SERIES_STYLES['primary'].copy()
    style_a['label'] = label_a
    ax_main.plot(time_clipped, signal_a_clipped, **style_a)
    
    style_b = SERIES_STYLES['secondary'].copy()
    style_b['label'] = label_b
    ax_main.plot(time_clipped, signal_b_clipped, **style_b)
    
    if skip_time > 0:
        skip_style = SERIES_STYLES['skip_boundary']
        ax_main.axvline(skip_time, **skip_style)
    
    ax_main.set_xlabel('Time (s)', **AXIS_STYLE['xlabel'])
    ax_main.set_ylabel(y_label, **AXIS_STYLE['ylabel'])
    ax_main.set_title(title, **AXIS_STYLE['title'])
    ax_main.legend(fontsize=9, framealpha=0.9, loc='best')
    apply_modern_style(ax_main)
    
    # Plot error (difference)
    diff = signal_b_clipped - signal_a_clipped
    error_style = SERIES_STYLES['error'].copy()
    error_style['label'] = f'Error ({label_b} - {label_a})'
    ax_error.plot(time_clipped, diff, **error_style)
    ax_error.axhline(y=0, color='#6c757d', linestyle='-', alpha=0.4, linewidth=1)
    
    error_y_label = f'Error ({units})' if units else 'Error'
    ax_error.set_xlabel('Time (s)', **AXIS_STYLE['xlabel'])
    ax_error.set_ylabel(error_y_label, **AXIS_STYLE['ylabel'])
    ax_error.set_title('Error Analysis', **AXIS_STYLE['title_error'])
    ax_error.legend(fontsize=9, framealpha=0.9, loc='best')
    apply_modern_style(ax_error)
    
    # Save plot
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(output_path), dpi=FIGURE_DPI, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close()


def run_internal_comparison(results_file_a, results_file_b, *,
                            test_name, label_a, label_b, y_label,
                            skip_time=0.0, thresholds=None):
    """Run a full internal method comparison.

    Args:
        results_file_a, results_file_b: Paths to result text files.
        test_name:    Test identifier for status file and plots.
        label_a:      Legend label for method A.
        label_b:      Legend label for method B.
        y_label:      Y-axis label for comparison plot.
        skip_time:    Skip initial transient (seconds).
        thresholds:   Optional (l2_max, linf_max) tuple for pass/fail.

    Returns:
        (l2_norm, linf_norm, passed_or_none)
    """
    data_a = np.loadtxt(results_file_a, comments='#')
    data_b = np.loadtxt(results_file_b, comments='#')

    time_a, signal_a = data_a[:, 0], data_a[:, 1]
    time_b, signal_b = data_b[:, 0], data_b[:, 1]

    if len(time_a) != len(time_b):
        min_len = min(len(time_a), len(time_b))
        time_a, signal_a = time_a[:min_len], signal_a[:min_len]
        time_b, signal_b = time_b[:min_len], signal_b[:min_len]

    l2, linf = compute_norms(signal_a, signal_b, skip_time=skip_time, time=time_a)

    passed = None
    if thresholds:
        l2_max, linf_max = thresholds
        passed = (l2 <= l2_max) and (linf <= linf_max)

    metrics = {"l2_norm": float(l2), "linf_norm": float(linf)}

    output_dir = Path(results_file_a).parent
    write_comparison_status(output_dir, test_name, metrics, passed)

    plot_path = output_dir / "plots" / "comparison.png"
    plot_comparison(time_a, signal_a, signal_b,
                    label_a=label_a, label_b=label_b,
                    title=f"{test_name}: {label_a} vs {label_b}",
                    y_label=y_label, output_path=str(plot_path),
                    skip_time=skip_time)

    status_str = f"PASS" if passed else (f"FAIL" if passed is False else "INFO")
    print(f"  {test_name}: L2={l2:.2e}, Linf={linf:.2e} [{status_str}]")

    return l2, linf, passed
