#!/usr/bin/env python3
"""
SEA-Stack Regression Test Comparison Template

This template provides a standardized way to compare reference and test data
across different regression test cases. It generates professional comparison
plots with consistent formatting and comprehensive information panels.

Usage:
    python compare_template.py <reference_file> <test_file> [test_name] [y_label]

Example:
    python compare_template.py ref_data.txt test_data.txt "Sphere Decay Test" "Heave (m)"
"""

import sys
import os
from pathlib import Path
from datetime import datetime
import platform

# CTest maps this exit code to "Skipped" when SKIP_RETURN_CODE is set on the test (CMake).
SKIP_EXIT_CODE = 77

# Check for required packages
try:
    import numpy as np
except ImportError:
    print("Error: numpy is required but not installed. Please install it with: pip install numpy")
    sys.exit(SKIP_EXIT_CODE)

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("Error: matplotlib is required but not installed. Please install it with: pip install matplotlib")
    sys.exit(SKIP_EXIT_CODE)

# Import shared plotting helpers
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "utilities"))
from plot_helpers import (
    LAYOUT,
    EXTRA_PALETTE,
    clip_to_common_time,
    format_path,
    get_seastack_version,
    get_chrono_version,
    create_text_panel,
    apply_modern_style,
    extract_units_from_label,
    compute_error_metrics,
    create_standard_figure,
    build_info_panel,
    build_system_panel,
    build_error_metrics_panel,
    build_data_stats_panel,
    build_simulation_summary_panel,
    SERIES_STYLES,
    AXIS_STYLE,
    FIGURE_DPI,
)

_NON_EXECUTABLE_EXTS = {'.txt', '.py', '.csv', '.json', '.md', '.log', '.png',
                        '.jpg', '.svg', '.pdf', '.h5', '.hdf5', '.dat', '.status'}

def find_executable(test_dir, executable_patterns):
    """
    Find executable in test directory or ancestor directories.
    
    Args:
        test_dir: Directory to start searching from
        executable_patterns: List of patterns to search for (e.g., ["sphere_decay_test", "rm3_test"])
    
    Returns:
        Path to executable if found, None otherwise
    """
    # Search upward from the results dir to locate the binary directory
    search_dirs = [test_dir]
    cur = test_dir
    for _ in range(4):
        cur = cur.parent
        search_dirs.append(cur)

    try:
        for s_dir in search_dirs:
            for pattern in executable_patterns:
                possible_names = [pattern, f"{pattern}.exe", f"{pattern}.out"]
                
                for name in possible_names:
                    exe_file = s_dir / name
                    if exe_file.exists() and exe_file.suffix not in _NON_EXECUTABLE_EXTS:
                        return exe_file
                
                for exe_file in s_dir.glob("*"):
                    if not exe_file.is_file() or exe_file.suffix in _NON_EXECUTABLE_EXTS:
                        continue
                    if pattern in exe_file.name:
                        if exe_file.suffix in ['.exe', '.out', '.app'] or (
                            platform.system() != 'Windows' and os.access(exe_file, os.X_OK)):
                            return exe_file
    except Exception as e:
        print(f"Warning: Could not find executable: {e}")
    
    return None

# extract_units_from_label is now imported from shared plot_helpers

def create_comparison_plot(ref_data, test_data, test_name, output_dir, 
                          ref_file_path=None, test_file_path=None, executable_path=None,
                          y_label="Value", executable_patterns=None,
                          extra_sources=None, output_suffix='comparison',
                          ref_label='Reference', sim_label='SEA-Stack'):
    """
    Create comparison plot between reference and test data.
    
    Args:
        ref_data: Reference data array (Nx2, time in col 0)
        test_data: Test data array (Mx2, time in col 0)
        test_name: Name of the test case
        output_dir: Directory to save the plot
        ref_file_path: Path to reference file (for transparency)
        test_file_path: Path to test file (for transparency)
        executable_path: Path to the executable that generated the test data
        y_label: Label for the y-axis (e.g., "Heave (m)", "Surge (m)")
        executable_patterns: List of patterns to search for executable if not provided
        extra_sources: Optional dict mapping source name to Nx2 array (time, value).
                       These are overlaid on the main plot for multi-code context;
                       error metrics remain ref-vs-test only.
        output_suffix: Filename suffix before .png (default 'comparison').
                       Use 'overlay' to produce debug-only artifacts excluded
                       from the verification report.
        ref_label: Legend label for the reference data (default 'Reference').
                   Use 'WEC-Sim' for WEC-Sim reference comparisons.
        sim_label: Legend label for the simulation/test data (default 'SEA-Stack').
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Clip both datasets to the common (overlapping) time range so that
    # a short CI run can be compared against longer reference data.
    ref_data, test_data = clip_to_common_time(ref_data, test_data)
    
    # Calculate error metrics using shared helper
    n1, n2, max_error, mean_error = compute_error_metrics(ref_data, test_data)
    
    # Prepare for plotting (interpolate to common grid for error plot)
    nval = test_data.shape[0]
    x = np.linspace(test_data[0, 0], test_data[nval-1, 0], nval)
    y1 = np.interp(x, ref_data[:,0], ref_data[:,1])
    y2 = np.interp(x, test_data[:,0], test_data[:,1])
    yd = y1 - y2
    
    # Set up the figure with standard configuration
    fig, ax1, ax2 = create_standard_figure(f'{test_name} - Reference vs Simulation Comparison')
    
    # Build metadata panels using shared helpers
    extra_info = {
        'ref_file': ref_file_path,
        'test_file': test_file_path,
        'executable_path': executable_path
    }
    build_info_panel(fig, test_name, ref_label, sim_label, extra_info, show_method_labels=False)
    build_system_panel(fig)
    
    # Plot main comparison using SERIES_STYLES
    style_ref = SERIES_STYLES['primary'].copy()
    style_ref['label'] = ref_label
    ax1.plot(ref_data[:,0], ref_data[:,1], **style_ref)
    
    style_sim = SERIES_STYLES['secondary'].copy()
    style_sim['label'] = sim_label
    ax1.plot(test_data[:,0], test_data[:,1], **style_sim)
    if extra_sources:
        t_lo, t_hi = ref_data[0, 0], ref_data[-1, 0]
        for i, (src_name, src_data) in enumerate(extra_sources.items()):
            mask = (src_data[:, 0] >= t_lo) & (src_data[:, 0] <= t_hi)
            clipped = src_data[mask]
            if len(clipped) > 0:
                color = EXTRA_PALETTE[i % len(EXTRA_PALETTE)]
                ax1.plot(clipped[:, 0], clipped[:, 1], color=color,
                         linewidth=1.2, alpha=0.7, label=src_name)
    ax1.set_xlabel('Time (s)', **AXIS_STYLE['xlabel'])
    ax1.set_ylabel(y_label, **AXIS_STYLE['ylabel'])
    ax1.set_title(f'{test_name} - Reference vs Simulation Comparison', **AXIS_STYLE['title'])
    ncol = 1 if not extra_sources else min(2 + len(extra_sources), 4)
    ax1.legend(fontsize=9, framealpha=0.9, ncol=ncol, loc='best')
    apply_modern_style(ax1)
    
    # Create error plot: show error vs simulation for primary ref and all extra sources
    error_style = SERIES_STYLES['error'].copy()
    error_style['label'] = f'Error ({ref_label} - {sim_label})'
    ax2.plot(x, yd, **error_style)
    if extra_sources:
        for i, (src_name, src_data) in enumerate(extra_sources.items()):
            y_src = np.interp(x, src_data[:, 0], src_data[:, 1])
            err_src = y_src - y2
            color = EXTRA_PALETTE[i % len(EXTRA_PALETTE)]
            ax2.plot(x, err_src, color=color, linewidth=1.2, alpha=0.8,
                     label=f'Error ({src_name} - {sim_label})')
    ax2.axhline(y=0, color='#6c757d', linestyle='-', alpha=0.4, linewidth=1)
    ax2.set_xlabel('Time (s)', **AXIS_STYLE['xlabel'])
    ax2.set_ylabel(f'Error ({extract_units_from_label(y_label)})', **AXIS_STYLE['ylabel'])
    ax2.set_title('Error Analysis', **AXIS_STYLE['title_error'])
    ncol_err = 1 if not extra_sources else min(2 + len(extra_sources), 4)
    ax2.legend(fontsize=9, framealpha=0.9, ncol=ncol_err, loc='best')
    apply_modern_style(ax2)
    
    # Build remaining panels using shared helpers
    metrics_dict = {
        'l2_norm': n1,
        'linf_norm': n2,
        'max_error': max_error,
        'mean_error': mean_error
    }
    build_error_metrics_panel(fig, metrics_dict)
    
    units = extract_units_from_label(y_label)
    build_data_stats_panel(fig, ref_data, test_data, ref_label, sim_label, units)
    build_simulation_summary_panel(fig, ref_data, test_data)
    
    # Save plot
    # Convert test_name to lowercase with underscores instead of spaces
    safe_test_name = test_name.lower().replace(' ', '_').replace('-', '_')
    plot_filename = os.path.join(output_dir, f'{safe_test_name}_{output_suffix}.png')
    plt.savefig(plot_filename, dpi=FIGURE_DPI, bbox_inches='tight', facecolor='white', edgecolor='none')
    print(f"Plot saved: {plot_filename}")
    
    return n1, n2

def write_status_file(output_dir, test_name, status, metrics=None, note=None):
    """Write a persistent status file for a comparison test.
    
    These files survive across ctest runs, unlike LastTest.log which is
    overwritten each invocation.  The report generator reads them as its
    primary source of pass/fail information.
    
    Args:
        output_dir:  Directory containing the test results (e.g. results/tests/rm3)
        test_name:   Canonical test name (e.g. "rm3_decay")
        status:      "PASS", "WARN", or "FAIL"
        metrics:     Optional dict with numeric metrics (l2_norm, linf_norm, …)
        note:        Optional human-readable annotation (e.g. known limitation)
    """
    import json
    status_dir = Path(output_dir)
    status_dir.mkdir(parents=True, exist_ok=True)
    status_file = status_dir / f"{test_name}.status.json"
    
    payload = {
        "test_name": test_name,
        "status": status,
        "timestamp": datetime.now().isoformat(),
    }
    if metrics:
        payload["metrics"] = metrics
    if note:
        payload["note"] = note
    
    try:
        with open(status_file, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2)
    except Exception as e:
        print(f"Warning: could not write status file {status_file}: {e}")


def run_comparison(ref_file, test_file, test_name=None, y_label="Value", 
                  executable_patterns=None, pass_criteria=None, status_name=None):
    """
    Run a complete comparison between reference and test data
    
    Args:
        ref_file: Path to reference data file
        test_file: Path to test data file
        test_name: Name of the test (defaults to test file stem)
        y_label: Label for y-axis
        executable_patterns: List of patterns to search for executable
        pass_criteria: Tuple of (l2_threshold, linf_threshold) for pass/fail
        status_name: Canonical name for the status file (e.g. "sphere_decay").
                     If None, derived from test_name.
    
    Returns:
        Tuple of (l2_norm, linf_norm)
    """
    print(f"Comparing: {ref_file} vs {test_file}")

    # Load data with error handling
    try:
        # Try to detect the number of header lines by looking for the first numeric line
        def find_data_start(filename):
            with open(filename, 'r') as f:
                for i, line in enumerate(f):
                    # Check if line contains numeric data (time value)
                    try:
                        float(line.split()[0])
                        return i
                    except (ValueError, IndexError):
                        continue
            return 0  # Default to 0 if no numeric line found
        
        ref_skiprows = find_data_start(ref_file)
        test_skiprows = find_data_start(test_file)
        
        refData = np.loadtxt(ref_file, skiprows=ref_skiprows)
        testData = np.loadtxt(test_file, skiprows=test_skiprows)
    except (OSError, IOError, ValueError) as e:
        print(f"Error loading data files: {e}")
        sys.exit(1)
    
    # Validate data
    if refData.size == 0 or testData.size == 0:
        print("Error: One or both data files are empty")
        sys.exit(1)
    
    if refData.shape[1] < 2 or testData.shape[1] < 2:
        print("Error: Data files must have at least 2 columns (time and value)")
        sys.exit(1)

    print(f"Reference data shape: {refData.shape}")
    print(f"Test data shape: {testData.shape}")

    # Determine test name from file path if not provided
    if test_name is None:
        test_name = Path(test_file).stem
    
    # Create plots directory in the same location as the test file
    test_file_path = Path(test_file)
    plots_dir = test_file_path.parent / "plots"
    
    # Find the executable path
    executable_path = None
    if executable_patterns:
        executable_path = find_executable(test_file_path.parent, executable_patterns)
    
    # Generate comparison plot
    def rel_to_root(path):
        try:
            project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
            return os.path.relpath(path, project_root)
        except Exception:
            return str(path)
    
    try:
        n1, n2 = create_comparison_plot(
            refData, testData, test_name, plots_dir, 
            ref_file_path=rel_to_root(ref_file), 
            test_file_path=rel_to_root(test_file),
            executable_path=rel_to_root(str(executable_path)) if executable_path else None,
            y_label=y_label,
            executable_patterns=executable_patterns
        )
    except Exception as e:
        print(f"Error creating comparison plot: {e}")
        sys.exit(1)
    
    # Check pass/fail criteria if provided
    if pass_criteria:
        l2_threshold, linf_threshold = pass_criteria
        metrics = {"l2_norm": n1, "linf_norm": n2}
        sname = status_name if status_name else test_name.lower().replace(' ', '_').replace('-', '_')
        if (n1 > l2_threshold or n2 > linf_threshold):
            print(f"TEST FAILED - L2 Norm: {n1:.2e}, L-infinity Norm: {n2:.2e}")
            write_status_file(test_file_path.parent, sname, "FAIL", metrics)
            return n1, n2, False
        else:
            print(f"TEST PASSED - L2 Norm: {n1:.2e}, L-infinity Norm: {n2:.2e}")
            write_status_file(test_file_path.parent, sname, "PASS", metrics)
            return n1, n2, True
    
    return n1, n2

def run_multi_column_comparison(ref_file, test_file, test_configs, executable_patterns=None, pass_criteria=None):
    """
    Run comparison for multiple columns of data, generating separate plots for each
    
    Args:
        ref_file: Path to reference data file
        test_file: Path to test data file
        test_configs: List of dicts, each containing:
            - 'column_index': Column index to plot (1-based)
            - 'test_name': Name for this specific test/plot
            - 'y_label': Label for y-axis
            - 'validation_tolerance': Optional tolerance for validation (defaults to pass_criteria)
            - 'status_name': Optional canonical name for the status file
        executable_patterns: List of patterns to search for executable
        pass_criteria: Tuple of (l2_threshold, linf_threshold) for pass/fail (default)
    
    Returns:
        List of tuples (l2_norm, linf_norm, passed) for each column
    """
    print(f"Comparing multiple columns: {ref_file} vs {test_file}")

    # Load data with error handling
    try:
        refData = np.loadtxt(ref_file, skiprows=1)
        testData = np.loadtxt(test_file, skiprows=1)
    except (OSError, IOError, ValueError) as e:
        print(f"Error loading data files: {e}")
        sys.exit(1)
    
    # Validate data
    if refData.size == 0 or testData.size == 0:
        print("Error: One or both data files are empty")
        sys.exit(1)
    
    if refData.shape[1] < 2 or testData.shape[1] < 2:
        print("Error: Data files must have at least 2 columns (time and value)")
        sys.exit(1)

    print(f"Reference data shape: {refData.shape}")
    print(f"Test data shape: {testData.shape}")

    # Create plots directory in the same location as the test file
    test_file_path = Path(test_file)
    plots_dir = test_file_path.parent / "plots"
    
    # Find the executable path
    executable_path = None
    if executable_patterns:
        executable_path = find_executable(test_file_path.parent, executable_patterns)
    
    results = []
    
    # Generate comparison plot for each column
    def rel_to_root(path):
        try:
            project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
            return os.path.relpath(path, project_root)
        except Exception:
            return str(path)
    
    for config in test_configs:
        column_index = config['column_index']
        test_name = config['test_name']
        y_label = config['y_label']
        validation_tolerance = config.get('validation_tolerance', pass_criteria)
        
        print(f"Generating plot for {test_name} (column {column_index})...")
        
        try:
            # Create a temporary data structure for this column
            # We need to create arrays with time and the specific column data
            ref_col_data = np.column_stack((refData[:, 0], refData[:, column_index]))
            test_col_data = np.column_stack((testData[:, 0], testData[:, column_index]))
            
            n1, n2 = create_comparison_plot(
                ref_col_data, test_col_data, test_name, plots_dir, 
                ref_file_path=rel_to_root(ref_file), 
                test_file_path=rel_to_root(test_file),
                executable_path=rel_to_root(str(executable_path)) if executable_path else None,
                y_label=y_label,
                executable_patterns=executable_patterns
            )
            
            # Check pass/fail criteria if provided
            passed = True
            metrics = {"l2_norm": n1, "linf_norm": n2}
            if validation_tolerance:
                l2_threshold, linf_threshold = validation_tolerance
                if (n1 > l2_threshold or n2 > linf_threshold):
                    print(f"TEST FAILED for {test_name} - L2 Norm: {n1:.2e}, L-infinity Norm: {n2:.2e}")
                    passed = False
                else:
                    print(f"TEST PASSED for {test_name} - L2 Norm: {n1:.2e}, L-infinity Norm: {n2:.2e}")
            
            sname = config.get('status_name') or test_name.lower().replace(' ', '_').replace('-', '_')
            write_status_file(test_file_path.parent, sname,
                              "PASS" if passed else "FAIL", metrics)
            results.append((n1, n2, passed))
            
        except Exception as e:
            print(f"Error creating comparison plot for {test_name}: {e}")
            results.append((float('inf'), float('inf'), False))
    
    return results

if __name__ == '__main__':
    """
    Template usage examples for regression tests
    
    This section shows how to use the comparison template for different types of tests.
    """
    
    # Example 1: Single column comparison (like sphere test)
    # Usage: python compare_template.py <ref_file> <test_file>
    if len(sys.argv) == 3:
        ref_file = sys.argv[1]
        test_file = sys.argv[2]
        
        # Single column configuration
        test_name = "Example Single Column Test"
        y_label = "Example Value (units)"
        executable_patterns = ["example_test", "example_test.exe"]
        pass_criteria = (1e-4, 1e-3)  # (L2_threshold, Linf_threshold)
        
        # Run single column comparison
        l2_norm, linf_norm, passed = run_comparison(
            ref_file, test_file, test_name, y_label, 
            executable_patterns, pass_criteria
        )
        
        sys.exit(0 if passed else 1)
    
    # Example 2: Multi-column comparison (like f3of test)
    # This would be used in a separate compare.py script:
    """
    #!/usr/bin/env python3
    import sys
    import os
    from pathlib import Path
    
    # Import the comparison template
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    from compare_template import run_multi_column_comparison
    
    def main():
        if len(sys.argv) != 3:
            print("Usage: python compare.py <reference_file> <test_file>")
        sys.exit(1)

    ref_file = sys.argv[1]
    test_file = sys.argv[2]
        
        # Test-specific configuration
        test_name = "Example Multi-Column Test"
        executable_patterns = ["example_test", "example_test.exe"]
        
        # Define columns to plot
        test_configs = [
            {
                'column_index': 2,  # First variable
                'test_name': f"{test_name} - Variable 1",
                'y_label': "Variable 1 (units)",
                'validation_tolerance': (1e-4, 1e-3)
            },
            {
                'column_index': 3,  # Second variable
                'test_name': f"{test_name} - Variable 2",
                'y_label': "Variable 2 (units)",
                'validation_tolerance': (1e-5, 1e-4)
            }
        ]
        
        # Run multi-column comparison
        results = run_multi_column_comparison(
            ref_file, test_file, test_configs, 
            executable_patterns=executable_patterns
        )
        
        # Check if all passed
        all_passed = all(result[2] for result in results)
        sys.exit(0 if all_passed else 1)
    
    if __name__ == "__main__":
        main()
    """
    
    print("Template usage examples:")
    print("1. Single column: python compare_template.py <ref_file> <test_file>")
    print("2. Multi-column: Create a compare.py script using the example above")
    print("3. See the sphere/ and f3of/ directories for working examples") 