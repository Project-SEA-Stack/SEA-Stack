#!/usr/bin/env python3
"""
Shared Plotting Helpers

This module provides common plotting utilities extracted from the regression
test template. These are building blocks that can be used by suite-specific
plotting wrappers to maintain consistent styling and layout across test suites.

The design keeps these as lower-level helpers rather than forcing a single
unified plotting API, allowing suite-specific semantics to be preserved.
"""

import os
import platform
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:
    np = None

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


# Layout configuration for text boxes and panels - using absolute positioning
LAYOUT = {
    'figure': {
        'figsize': (12, 9),
        'facecolor': 'white'
    },
    'fonts': {
        'title': 13,
        'heading': 11,
        'body': 11,
        'small': 8
    },
    'panels': {
        'test_info': {
            'pos': (0.02, 0.82, 0.8, 0.12),
            'font_size': 'body',
            'style': {
                'facecolor': '#f8f9fa',
                'edgecolor': '#e9ecef',
                'text_color': '#212529'
            }
        },
        'system_info': {
            'pos': (0.85, 0.82, 0.22, 0.12),
            'font_size': 'small',
            'style': {
                'facecolor': '#f8f9fa',
                'edgecolor': '#e9ecef',
                'text_color': '#212529'
            }
        },
        'simulation_summary': {
            'pos': (0.85, 0.63, 0.22, 0.15),
            'font_size': 'body',
            'style': {
                'facecolor': '#f8f9fa',
                'edgecolor': '#e9ecef',
                'text_color': '#212529'
            }
        },
        'data_stats': {
            'pos': (0.85, 0.18, 0.22, 0.35),
            'font_size': 'body',
            'style': {
                'facecolor': '#f8f9fa',
                'edgecolor': '#e9ecef',
                'text_color': '#212529'
            }
        },
        'error_metrics': {
            'pos': (0.85, 0.07, 0.22, 0.15),
            'font_size': 'body',
            'style': {
                'facecolor': '#f8f9fa',
                'edgecolor': '#e9ecef',
                'text_color': '#dc3545'
            }
        }
    },
    'plots': {
        'main_plot': {
            'pos': (0.05, 0.45, 0.78, 0.28)
        },
        'error_plot': {
            'pos': (0.05, 0.05, 0.78, 0.28)
        }
    }
}

# Color palette for additional data sources in multi-source plots
EXTRA_PALETTE = [
    '#2ca02c', '#ff7f0e', '#9467bd', '#8c564b',
    '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
]

# Canonical series styling for consistent plot appearance
SERIES_STYLES = {
    'primary': {
        'color': '#007bff',
        'linewidth': 2.5,
        'linestyle': '-',
        'alpha': 0.9,
        'label': None  # Set by caller
    },
    'secondary': {
        'color': '#dc3545',
        'linewidth': 2.5,
        'linestyle': '--',
        'alpha': 0.9,
        'label': None  # Set by caller
    },
    'error': {
        'color': '#dc3545',
        'linewidth': 2.0,
        'linestyle': '-',
        'alpha': 0.8,
        'label': None  # Set by caller
    },
    'skip_boundary': {
        'color': 'gray',
        'linewidth': 1.0,
        'linestyle': ':',
        'alpha': 0.5,
        'label': 'Skip boundary'
    }
}

# Canonical axis styling
AXIS_STYLE = {
    'xlabel': {
        'fontsize': 11,
        'color': '#495057',
        'fontweight': '500'
    },
    'ylabel': {
        'fontsize': 11,
        'color': '#495057',
        'fontweight': '500'
    },
    'title': {
        'fontsize': 13,
        'color': '#212529',
        'fontweight': 'bold',
        'pad': 25  # For main plot
    },
    'title_error': {
        'fontsize': 13,
        'color': '#212529',
        'fontweight': 'bold',
        'pad': 15  # For error plot
    }
}

# Standard figure DPI for all plots
FIGURE_DPI = 300


def clip_to_common_time(ref_data, test_data):
    """Clip both datasets to their overlapping time range.

    When the simulation and reference have different durations (e.g. a short
    CI run vs. full-length reference data), this function trims both arrays
    to the time interval covered by *both* series so that the comparison is
    always apples-to-apples.

    Args:
        ref_data:  Nx2+ array with time in column 0
        test_data: Mx2+ array with time in column 0

    Returns:
        (ref_clipped, test_clipped) arrays covering the common time range.
    """
    if np is None:
        raise ImportError("numpy is required for clip_to_common_time")
    
    t_start = max(ref_data[0, 0], test_data[0, 0])
    t_end   = min(ref_data[-1, 0], test_data[-1, 0])

    if t_end <= t_start:
        print("WARNING: No overlapping time range between reference and simulation!")
        return ref_data, test_data

    ref_mask  = (ref_data[:, 0] >= t_start)  & (ref_data[:, 0] <= t_end)
    test_mask = (test_data[:, 0] >= t_start) & (test_data[:, 0] <= t_end)

    ref_clipped  = ref_data[ref_mask]
    test_clipped = test_data[test_mask]

    if len(ref_clipped) < len(ref_data) or len(test_clipped) < len(test_data):
        print(f"Clipped to common time range [{t_start:.2f}, {t_end:.2f}]s  "
              f"(ref {len(ref_data)}->{len(ref_clipped)} pts, "
              f"sim {len(test_data)}->{len(test_clipped)} pts)")

    return ref_clipped, test_clipped


def format_path(path):
    """Format file paths for display by making them relative to current directory and removing SEA-Stack prefix"""
    if path is None:
        return "Not specified"
    try:
        # First make it relative to current directory
        rel_path = os.path.relpath(path, os.getcwd())
        if len(rel_path) >= len(path):
            rel_path = path
        
        # Remove "SEA-Stack" or "sea-stack" from the beginning of the path if present
        for prefix in ("SEA-Stack/", "SEA-Stack\\", "sea-stack/", "sea-stack\\"):
            if rel_path.startswith(prefix):
                rel_path = rel_path[len(prefix):]
                break

        return rel_path
    except (ValueError, OSError):
        # If relative path fails, try to remove SEA-Stack/sea-stack from absolute path
        path_str = str(path)
        for name in ("SEA-Stack", "sea-stack"):
            pos = path_str.find(name)
            if pos != -1:
                remaining = path_str[pos + len(name):]
                if remaining.startswith("/") or remaining.startswith("\\"):
                    remaining = remaining[1:]
                return remaining
        return path_str


def get_cmake_cache_path():
    """Return the canonical path to CMakeCache.txt in the build directory."""
    # 1. Check SEASTACK_BUILD_DIR env var
    build_dir = os.environ.get('SEASTACK_BUILD_DIR')
    if build_dir:
        candidate = os.path.join(build_dir, 'CMakeCache.txt')
        if os.path.exists(candidate):
            return candidate
    
    # 2. Search upwards from current directory
    cur = os.path.abspath(os.getcwd())
    while True:
        candidate = os.path.join(cur, 'CMakeCache.txt')
        if os.path.exists(candidate):
            return candidate
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    
    # 3. Fallback to relative to script location
    possible_paths = [
        os.path.join(os.path.dirname(__file__), '../../../CMakeCache.txt'),
        os.path.join(os.path.dirname(__file__), '../../build/CMakeCache.txt'),
        os.path.join(os.path.dirname(__file__), '../build/CMakeCache.txt'),
    ]
    for path in possible_paths:
        if os.path.exists(path):
            return path
    return None


def get_seastack_version():
    """Get SEA-Stack version from generated version.h header, falling back to CMakeCache.txt"""
    try:
        # First, try to read from the generated version.h header
        cmake_cache_path = get_cmake_cache_path()
        if cmake_cache_path:
            # Build directory is the parent of CMakeCache.txt
            build_dir = os.path.dirname(cmake_cache_path)
            version_h = os.path.join(build_dir, 'seastack', 'version.h')
            if os.path.exists(version_h):
                with open(version_h, 'r', encoding='utf-8') as f:
                    for line in f:
                        if line.startswith('#define SEASTACK_VERSION_FULL'):
                            # Extract quoted string value: #define SEASTACK_VERSION_FULL "1.0.0-beta"
                            parts = line.split('"')
                            if len(parts) >= 2:
                                return parts[1]
            
            # Fallback to numeric version from CMakeCache.txt
            with open(cmake_cache_path, 'r', encoding='utf-8') as f:
                for line in f:
                    if line.startswith('CMAKE_PROJECT_VERSION:STATIC='):
                        return line.split('=')[1].strip()
        return os.environ.get('SEASTACK_VERSION', 'Unknown')
    except (OSError, IOError, UnicodeDecodeError):
        return os.environ.get('SEASTACK_VERSION', 'Unknown')


def _chrono_git_suffix(chrono_root):
    """Return a git-based suffix like ' (branch@abc1234)' if Chrono is a dev build.
    
    Reads git metadata directly from the .git directory so that the git
    executable does not need to be on PATH (common in CTest environments on
    Windows).
    """
    try:
        git_dir = os.path.join(chrono_root, '.git')
        if not os.path.isdir(git_dir):
            return ''

        head_file = os.path.join(git_dir, 'HEAD')
        with open(head_file, 'r', encoding='utf-8') as f:
            head = f.read().strip()

        if head.startswith('ref: '):
            ref = head[5:]  # e.g. 'refs/heads/feature/fsi'
            branch = ref.split('refs/heads/', 1)[-1] if 'refs/heads/' in ref else ref

            # Resolve the commit hash from the ref
            ref_file = os.path.join(git_dir, ref.replace('/', os.sep))
            commit = None
            if os.path.isfile(ref_file):
                with open(ref_file, 'r', encoding='utf-8') as f:
                    commit = f.read().strip()[:7]
            else:
                # Ref may be in packed-refs
                packed = os.path.join(git_dir, 'packed-refs')
                if os.path.isfile(packed):
                    with open(packed, 'r', encoding='utf-8') as f:
                        for line in f:
                            if line.startswith('#'):
                                continue
                            parts = line.strip().split()
                            if len(parts) == 2 and parts[1] == ref:
                                commit = parts[0][:7]
                                break
            if not commit:
                return f' ({branch})'
        else:
            branch = 'detached'
            commit = head[:7]

        return f' ({branch}@{commit})'
    except Exception:
        return ''


def _chrono_version_from_cmakelists(chrono_cmakelists_path):
    """Parse CHRONO_VERSION_{MAJOR,MINOR,PATCH} from Chrono's top CMakeLists.txt."""
    if not os.path.isfile(chrono_cmakelists_path):
        return None
    major = minor = patch = "0"
    try:
        with open(chrono_cmakelists_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line.startswith('set(CHRONO_VERSION_MAJOR'):
                    parts = line.split()
                    if len(parts) >= 2:
                        major = parts[1].rstrip(')')
                elif line.startswith('set(CHRONO_VERSION_MINOR'):
                    parts = line.split()
                    if len(parts) >= 2:
                        minor = parts[1].rstrip(')')
                elif line.startswith('set(CHRONO_VERSION_PATCH'):
                    parts = line.split()
                    if len(parts) >= 2:
                        patch = parts[1].rstrip(')')
        if major != "0" or minor != "0" or patch != "0":
            return f"{major}.{minor}.{patch}"
    except (OSError, IOError, UnicodeDecodeError):
        return None
    return None


def _chrono_layout_roots(chrono_dir):
    """Directories that may hold Chrono's top-level CMakeLists.txt or repo .git.

    Upstream Chrono often uses CMAKE_HOME_DIRECTORY = .../source while the
    repository root (dirname(dirname(Chrono_DIR))) has no CMakeLists.txt.
    Order: CMake source dir from Chrono's build cache, then naive root, then
            naive_root/source.
    """
    if not chrono_dir:
        return []
    chrono_build_dir = os.path.dirname(chrono_dir)
    naive_root = os.path.dirname(chrono_build_dir)
    roots = []
    seen_norm = set()

    def add(path):
        if not path:
            return
        norm = os.path.normcase(os.path.normpath(path))
        if norm in seen_norm or not os.path.isdir(path):
            return
        seen_norm.add(norm)
        roots.append(path)

    cache_file = os.path.join(chrono_build_dir, 'CMakeCache.txt')
    if os.path.isfile(cache_file):
        try:
            with open(cache_file, 'r', encoding='utf-8') as f:
                for ln in f:
                    if ln.startswith('CMAKE_HOME_DIRECTORY:INTERNAL='):
                        home = ln.split('=', 1)[1].strip()
                        if home:
                            add(home)
                        break
        except OSError:
            pass

    add(naive_root)
    add(os.path.join(naive_root, 'source'))
    return roots


def get_chrono_version():
    """Get Chrono version from Chrono CMakeLists.txt, with git branch/hash
    appended when building from a development branch."""
    try:
        cmake_cache_path = get_cmake_cache_path()
        if not cmake_cache_path:
            return os.environ.get('CHRONO_VERSION', 'Unknown')

        chrono_dir = None
        with open(cmake_cache_path, 'r', encoding='utf-8') as f:
            for line in f:
                # CMake uses Chrono_DIR:PATH=, :UNINITIALIZED=, etc.
                if line.startswith('Chrono_DIR:'):
                    chrono_dir = line.split('=', 1)[1].strip()
                    break

        layout_roots = _chrono_layout_roots(chrono_dir) if chrono_dir else []
        version_str = None
        for root in layout_roots:
            candidate = os.path.join(root, 'CMakeLists.txt')
            version_str = _chrono_version_from_cmakelists(candidate)
            if version_str:
                break

        if not version_str:
            version_str = os.environ.get('CHRONO_VERSION', 'Unknown')

        git_suffix = ''
        if chrono_dir:
            for src in layout_roots:
                if os.path.isdir(os.path.join(src, '.git')):
                    git_suffix = _chrono_git_suffix(src)
                    break

        return version_str + git_suffix
    except (OSError, IOError, UnicodeDecodeError):
        return os.environ.get('CHRONO_VERSION', 'Unknown')


def create_text_panel(fig, panel_config, content):
    """Create a text panel with given configuration using absolute positioning
    
    Args:
        fig: matplotlib Figure object
        panel_config: Panel configuration dict (must have 'pos' and 'font_size' keys)
        content: Text content to display in the panel
    
    Returns:
        matplotlib Text object
    """
    if plt is None:
        raise ImportError("matplotlib is required for create_text_panel")
    
    left, bottom, width, height = panel_config['pos']
    ax = fig.add_axes([left, bottom, width, height])
    ax.axis('off')
    style = panel_config['style']
    font_size = LAYOUT['fonts'][panel_config['font_size']]
    
    return ax.text(0.05, 0.95, content,
                  transform=ax.transAxes,
                  verticalalignment='top',
                  bbox=dict(boxstyle='round,pad=0.6',
                           facecolor=style['facecolor'],
                           edgecolor=style['edgecolor'],
                           linewidth=1.5,
                           alpha=0.95),
                  fontsize=font_size,
                  family='monospace',
                  fontweight='normal',
                  color=style['text_color'])


def apply_modern_style(ax):
    """Apply consistent modern styling to plot axes
    
    Args:
        ax: matplotlib Axes object to style
    """
    if plt is None:
        raise ImportError("matplotlib is required for apply_modern_style")
    
    ax.grid(True, alpha=0.2, color='#6c757d', linewidth=0.5)
    ax.tick_params(labelsize=9, colors='#495057')
    for spine in ['top', 'right', 'left', 'bottom']:
        ax.spines[spine].set_visible(True)
        ax.spines[spine].set_color('#dee2e6')
        ax.spines[spine].set_linewidth(1.0)
    ax.set_facecolor('#ffffff')


def extract_units_from_label(y_label):
    """Extract units from y-axis label
    
    Args:
        y_label: Y-axis label string (e.g., "Heave (m)", "Pitch (rad)")
    
    Returns:
        Unit string (e.g., "m", "rad", "deg")
    """
    units = None
    
    # Common patterns for extracting units from labels
    if '(' in y_label and ')' in y_label:
        # Extract text between parentheses
        start = y_label.find('(')
        end = y_label.find(')')
        if start != -1 and end != -1 and end > start:
            units = y_label[start+1:end].strip()
    
    # If no units found in parentheses, check the whole label
    if not units:
        y_label_lower = y_label.lower()
        if 'radian' in y_label_lower or 'rad' in y_label_lower:
            units = 'rad'
        elif 'degree' in y_label_lower or 'deg' in y_label_lower:
            units = 'deg'
        elif 'meter' in y_label_lower or 'm)' in y_label_lower:
            units = 'm'
        elif 'second' in y_label_lower or 's)' in y_label_lower:
            units = 's'
        elif 'newton' in y_label_lower or 'n)' in y_label_lower:
            units = 'N'
        elif 'watt' in y_label_lower or 'w)' in y_label_lower:
            units = 'W'
        else:
            units = 'units'  # Generic fallback
    
    # Normalize common unit variations
    if units:
        units_lower = units.lower()
        if units_lower in ['radians', 'radian']:
            return 'rad'
        elif units_lower in ['degrees', 'degree']:
            return 'deg'
        elif units_lower in ['meters', 'meter']:
            return 'm'
        elif units_lower in ['seconds', 'second']:
            return 's'
        elif units_lower in ['newtons', 'newton']:
            return 'N'
        elif units_lower in ['watts', 'watt']:
            return 'W'
        else:
            return units
    
    return 'units'  # Final fallback


def compute_error_metrics(ref_data, test_data):
    """Compute L2 and L-infinity error norms between reference and test data.
    
    Args:
        ref_data: Reference data array (Nx2, time in col 0, value in col 1)
        test_data: Test data array (Mx2, time in col 0, value in col 1)
    
    Returns:
        Tuple of (l2_norm, linf_norm, max_error, mean_error)
    """
    if np is None:
        raise ImportError("numpy is required for compute_error_metrics")
    
    nval = test_data.shape[0]
    x = np.linspace(test_data[0, 0], test_data[nval-1, 0], nval)
    y1 = np.interp(x, ref_data[:,0], ref_data[:,1])
    y2 = np.interp(x, test_data[:,0], test_data[:,1])
    yd = y1 - y2
    n1 = np.linalg.norm(yd)/nval  # L2 norm
    n2 = np.linalg.norm(yd, np.inf)  # L-infinity norm
    max_error = np.max(np.abs(yd))
    mean_error = np.mean(yd)
    
    return n1, n2, max_error, mean_error


def create_standard_figure(title):
    """Create a standard figure with LAYOUT-driven positioning for main and error plots.
    
    Args:
        title: Figure title (not used directly, but available for future use)
    
    Returns:
        Tuple of (fig, ax_main, ax_error) where:
        - fig: matplotlib Figure object
        - ax_main: Axes for main comparison plot
        - ax_error: Axes for error plot
    """
    if plt is None:
        raise ImportError("matplotlib is required for create_standard_figure")
    
    fig_cfg = LAYOUT['figure']
    fig = plt.figure(figsize=fig_cfg['figsize'], facecolor=fig_cfg['facecolor'])
    
    # Create main plot axes
    plot_cfg = LAYOUT['plots']['main_plot']
    ax_main = fig.add_axes(plot_cfg['pos'])
    
    # Create error plot axes
    plot_cfg = LAYOUT['plots']['error_plot']
    ax_error = fig.add_axes(plot_cfg['pos'])
    
    return fig, ax_main, ax_error


def build_info_panel(fig, test_name, label_a, label_b, extra_info=None, show_method_labels=True):
    """Build the test information panel.
    
    Args:
        fig: matplotlib Figure object
        test_name: Name of the test case
        label_a: Label for method/series A
        label_b: Label for method/series B
        extra_info: Optional dict with additional info (e.g., 'ref_file', 'test_file', 'executable_path')
        show_method_labels: If True, include "Method A" and "Method B" lines (default True, for comparison tests)
    
    Returns:
        matplotlib Text object
    """
    from datetime import datetime
    
    info_parts = [
        f"Test Information",
        "",
        f"Test: {test_name}",
    ]
    if show_method_labels:
        info_parts.append(f"Method A: {label_a}")
        info_parts.append(f"Method B: {label_b}")
    
    if extra_info:
        if 'ref_file' in extra_info:
            info_parts.append(f"Reference File: {format_path(extra_info['ref_file'])}")
        if 'test_file' in extra_info:
            info_parts.append(f"Latest File: {format_path(extra_info['test_file'])}")
        if 'executable_path' in extra_info:
            exe_path = extra_info['executable_path']
            if exe_path:
                exe_name = os.path.basename(exe_path)
                for ext in ['.exe', '.out', '.app']:
                    if exe_name.endswith(ext):
                        exe_name = exe_name[:-len(ext)]
                        break
                info_parts.append(f"Model/Executable: {exe_name}")
    
    info_parts.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    content = "\n".join(info_parts)
    return create_text_panel(fig, LAYOUT['panels']['test_info'], content)


def build_system_panel(fig):
    """Build the system information panel.
    
    Args:
        fig: matplotlib Figure object
    
    Returns:
        matplotlib Text object
    """
    platform_name = {"Windows": "Windows", "Darwin": "macOS"}.get(platform.system(), "Linux")
    python_version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    seastack_version = get_seastack_version()
    chrono_version = get_chrono_version()
    
    content = (
        f"System Information\n\n"
        f"Platform: {platform_name}\n"
        f"Python: {python_version}\n"
        f"SEA-Stack: {seastack_version}\n"
        f"Chrono: {chrono_version}"
    )
    
    return create_text_panel(fig, LAYOUT['panels']['system_info'], content)


def build_error_metrics_panel(fig, metrics_dict):
    """Build the error metrics panel.
    
    Args:
        fig: matplotlib Figure object
        metrics_dict: Dict with keys like 'l2_norm', 'linf_norm', 'max_error', 'mean_error'
                     or 'l2', 'linf' (alternative naming)
    
    Returns:
        matplotlib Text object
    """
    l2 = metrics_dict.get('l2_norm', metrics_dict.get('l2', 0.0))
    linf = metrics_dict.get('linf_norm', metrics_dict.get('linf', 0.0))
    max_err = metrics_dict.get('max_error', metrics_dict.get('max', 0.0))
    mean_err = metrics_dict.get('mean_error', metrics_dict.get('mean', 0.0))
    
    content = (
        f"Error Metrics\n\n"
        f"L₂ Norm: {l2:.2e}\n"
        f"L∞ Norm: {linf:.2e}\n"
        f"Max Error: {max_err:.2e}\n"
        f"Mean Error: {mean_err:.2e}"
    )
    
    return create_text_panel(fig, LAYOUT['panels']['error_metrics'], content)


def build_data_stats_panel(fig, data_a, data_b, label_a, label_b, units):
    """Build the data statistics panel.
    
    Args:
        fig: matplotlib Figure object
        data_a: Nx2 array (time, value) for series A
        data_b: Mx2 array (time, value) for series B
        label_a: Label for series A
        label_b: Label for series B
        units: Unit string (e.g., 'm', 'rad', 'N')
    
    Returns:
        matplotlib Text object
    """
    if np is None:
        raise ImportError("numpy is required for build_data_stats_panel")
    
    # Compute statistics for series A
    mean_a = np.mean(data_a[:, 1])
    std_a = np.std(data_a[:, 1])
    min_a = np.min(data_a[:, 1])
    max_a = np.max(data_a[:, 1])
    
    # Compute statistics for series B
    mean_b = np.mean(data_b[:, 1])
    std_b = np.std(data_b[:, 1])
    min_b = np.min(data_b[:, 1])
    max_b = np.max(data_b[:, 1])
    
    # Compute correlation (interpolate to common grid)
    nval = min(len(data_a), len(data_b))
    if nval > 1:
        t_start = max(data_a[0, 0], data_b[0, 0])
        t_end = min(data_a[-1, 0], data_b[-1, 0])
        if t_end > t_start:
            x = np.linspace(t_start, t_end, nval)
            y_a = np.interp(x, data_a[:, 0], data_a[:, 1])
            y_b = np.interp(x, data_b[:, 0], data_b[:, 1])
            corr = np.corrcoef(y_a, y_b)[0, 1]
        else:
            corr = float('nan')
    else:
        corr = float('nan')
    
    content = (
        f"Data Statistics\n\n"
        f"{label_a}:\n"
        f"  Mean: {mean_a:.4f} {units}\n"
        f"  Std: {std_a:.4f} {units}\n"
        f"  Range: [{min_a:.3f}, {max_a:.3f}] {units}\n\n"
        f"{label_b}:\n"
        f"  Mean: {mean_b:.4f} {units}\n"
        f"  Std: {std_b:.4f} {units}\n"
        f"  Range: [{min_b:.3f}, {max_b:.3f}] {units}\n\n"
        f"Correlation: {corr:.6f}"
    )
    
    return create_text_panel(fig, LAYOUT['panels']['data_stats'], content)


def build_simulation_summary_panel(fig, data_a, data_b):
    """Build the simulation summary panel.
    
    Args:
        fig: matplotlib Figure object
        data_a: Nx2 array (time, value) for series A
        data_b: Mx2 array (time, value) for series B
    
    Returns:
        matplotlib Text object
    """
    if np is None:
        raise ImportError("numpy is required for build_simulation_summary_panel")
    
    # Series A summary
    duration_a = data_a[-1, 0] - data_a[0, 0]
    dt_a = data_a[1, 0] - data_a[0, 0] if len(data_a) > 1 else 0
    points_a = len(data_a)
    
    # Series B summary
    duration_b = data_b[-1, 0] - data_b[0, 0]
    dt_b = data_b[1, 0] - data_b[0, 0] if len(data_b) > 1 else 0
    points_b = len(data_b)
    
    content = (
        f"Simulation Summary\n\n"
        f"Series A:\n"
        f"  Duration: {duration_a:.1f}s\n"
        f"  Timestep: {dt_a:.3f}s\n"
        f"  Points: {points_a}\n\n"
        f"Series B:\n"
        f"  Duration: {duration_b:.1f}s\n"
        f"  Timestep: {dt_b:.3f}s\n"
        f"  Points: {points_b}"
    )
    
    return create_text_panel(fig, LAYOUT['panels']['simulation_summary'], content)
