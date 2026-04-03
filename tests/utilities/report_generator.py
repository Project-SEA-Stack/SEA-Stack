#!/usr/bin/env python3
"""
Shared Report Generation Infrastructure

This module provides common utilities for generating test reports across
regression, verification, and comparison test suites. It handles:
- Status file discovery and reading
- Plot file discovery
- Markdown generation helpers
- PDF conversion
- Status aggregation

The design separates pure utilities (file discovery, path computation, rendering)
from suite-specific policy (ordering, grouping, naming, exclusions).
"""

import json
import os
import subprocess
import tempfile
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any


def _results_tests_roots(build_path: Path, config: Optional[str] = None) -> List[Path]:
    """Roots at build/<...>/results/tests for artifact discovery.

    When ``config`` is set (e.g. Release, Debug), only that CMake build-type
    tree is scanned so Debug reports do not pick up stale Release outputs.

    When ``config`` is None, preserve legacy behavior: prefer
    ``bin/Release/results/tests`` and ``bin/results/tests`` if they exist.
    """
    if config:
        return [build_path / "bin" / config / "results" / "tests"]
    roots: List[Path] = []
    release = build_path / "bin" / "Release" / "results" / "tests"
    if release.exists():
        roots.append(release)
    legacy = build_path / "bin" / "results" / "tests"
    if legacy.exists():
        roots.append(legacy)
    if not roots:
        roots.append(release)
    return roots


def results_tests_search_bases(build_dir, config: Optional[str] = None) -> List[Path]:
    """Return ``Path`` roots ending in ``.../results/tests`` for suite discovery."""
    build_path = Path(build_dir).resolve() if build_dir else Path.cwd()
    return _results_tests_roots(build_path, config)


def find_status_files(
    build_dir,
    suite_filter=None,
    search_patterns=None,
    config: Optional[str] = None,
):
    """Scan for .status.json files across test suites.
    
    Args:
        build_dir: Build directory path
        suite_filter: Optional filter for test suite ('regression', 'verification', 'comparison')
        search_patterns: Optional list of additional search patterns (Path objects or strings)
        config: Optional CMake build type; when set, only ``bin/<config>/results/tests`` is used
    
    Returns:
        List of records, each containing:
        - suite: Test suite name (inferred from path)
        - model: Model name (if applicable)
        - test_name: Test name from status file
        - path: Path to status file
        - status_data: Parsed JSON content
    """
    build_path = Path(build_dir).resolve() if build_dir else Path.cwd()
    
    records = []
    
    # Default search directories based on suite (scoped per build config when given)
    search_dirs = []
    for base in _results_tests_roots(build_path, config):
        if suite_filter == 'regression' or suite_filter is None:
            search_dirs.append(base / "regression")
        if suite_filter == 'verification' or suite_filter is None:
            search_dirs.append(base / "verification")
        if suite_filter == 'comparison' or suite_filter is None:
            search_dirs.append(base / "comparison")
    
    # Add custom search patterns if provided
    if search_patterns:
        for pattern in search_patterns:
            pattern_path = Path(pattern) if isinstance(pattern, str) else pattern
            if pattern_path.is_absolute():
                search_dirs.append(pattern_path)
            else:
                search_dirs.append(build_path / pattern_path)
    
    for search_dir in search_dirs:
        if not search_dir.exists():
            continue
        
        # Walk through directory structure
        for status_file in search_dir.rglob("*.status.json"):
            try:
                with open(status_file, 'r', encoding='utf-8') as f:
                    payload = json.load(f)
                
                # Infer suite from path
                suite = None
                path_parts = status_file.parts
                if 'regression' in path_parts:
                    suite = 'regression'
                elif 'verification' in path_parts:
                    suite = 'verification'
                elif 'comparison' in path_parts:
                    suite = 'comparison'
                
                # Infer model from path (common patterns)
                model = None
                model_candidates = ['sphere', 'f3of', 'oswec', 'rm3', 'rm3_mooring']
                for candidate in model_candidates:
                    if candidate in path_parts:
                        model = candidate
                        break
                
                test_name = payload.get("test_name", status_file.stem.replace(".status", ""))
                
                records.append({
                    'suite': suite,
                    'model': model,
                    'test_name': test_name,
                    'path': status_file,
                    'status_data': payload
                })
            except Exception as e:
                print(f"Warning: could not read status file {status_file}: {e}")
    
    return records


def find_plot_files(
    build_dir,
    suite_filter=None,
    pattern="*.png",
    exclude_patterns=None,
    config: Optional[str] = None,
):
    """Find plot image files across test suites.
    
    Args:
        build_dir: Build directory path
        suite_filter: Optional filter for test suite
        pattern: File pattern to match (default: "*.png")
        exclude_patterns: Optional list of patterns to exclude (e.g., ["*_overlay.png"])
        config: Optional CMake build type; when set, only ``bin/<config>/results/tests`` is used
    
    Returns:
        List of records, each containing:
        - suite: Test suite name (inferred from path)
        - model: Model name (if applicable)
        - test_name: Test name (inferred from directory structure)
        - path: Path to plot file
    """
    build_path = Path(build_dir).resolve() if build_dir else Path.cwd()
    
    records = []
    
    search_dirs = []
    for base in _results_tests_roots(build_path, config):
        if suite_filter == 'regression' or suite_filter is None:
            search_dirs.append(base / "regression")
        if suite_filter == 'verification' or suite_filter is None:
            search_dirs.append(base / "verification")
        if suite_filter == 'comparison' or suite_filter is None:
            search_dirs.append(base / "comparison")
    
    exclude_patterns = exclude_patterns or []
    
    for search_dir in search_dirs:
        if not search_dir.exists():
            continue
        
        for plot_file in search_dir.rglob(pattern):
            # Check exclusion patterns
            should_exclude = False
            for exclude_pattern in exclude_patterns:
                if plot_file.match(exclude_pattern) or exclude_pattern in plot_file.name:
                    should_exclude = True
                    break
            if should_exclude:
                continue
            
            # Infer suite from path
            suite = None
            path_parts = plot_file.parts
            if 'regression' in path_parts:
                suite = 'regression'
            elif 'verification' in path_parts:
                suite = 'verification'
            elif 'comparison' in path_parts:
                suite = 'comparison'
            
            # Infer model from path
            model = None
            model_candidates = ['sphere', 'f3of', 'oswec', 'rm3', 'rm3_mooring']
            for candidate in model_candidates:
                if candidate in path_parts:
                    model = candidate
                    break
            
            # Infer test name from directory structure
            test_name = None
            if 'comparison' in path_parts:
                # For comparison: results/tests/comparison/{model}/{test_name}/plots/...
                try:
                    comparison_idx = path_parts.index('comparison')
                    if len(path_parts) > comparison_idx + 2:
                        test_name = path_parts[comparison_idx + 2]
                except (ValueError, IndexError):
                    pass
            else:
                # For regression/verification: use parent directory or filename
                test_name = plot_file.stem
            
            records.append({
                'suite': suite,
                'model': model,
                'test_name': test_name,
                'path': plot_file
            })
    
    return records


def generate_report_header(suite_name, description, timestamp=None):
    """Generate standard markdown report header.
    
    Args:
        suite_name: Name of the test suite (e.g., "Regression", "Verification")
        description: Brief description of the suite
        timestamp: Optional timestamp (defaults to current time)
    
    Returns:
        List of markdown lines for the header
    """
    if timestamp is None:
        timestamp = datetime.now()
    
    content = []
    content.append(f"# SEA-Stack {suite_name} Test Report")
    content.append("")
    content.append(f"**{description}**")
    content.append("")
    content.append(f"*Generated on {timestamp.strftime('%B %d, %Y at %H:%M:%S')}*")
    content.append("")
    
    return content


def generate_summary_table(status_data, suite_name, columns=None):
    """Generate a summary table from status data.
    
    Args:
        status_data: List of status records (from find_status_files)
        suite_name: Name of the suite (for context)
        columns: Optional list of column names (defaults to suite-specific)
    
    Returns:
        List of markdown lines for the summary table
    """
    if not status_data:
        return ["No test data available.", ""]
    
    # Count by status
    counts = count_by_status(status_data)
    
    content = []
    content.append("## Summary")
    content.append("")
    
    total = len(status_data)
    n_pass = counts.get('PASS', 0)
    n_warn = counts.get('WARN', 0)
    n_fail = counts.get('FAIL', 0)
    n_info = counts.get('INFO', 0)
    n_unknown = counts.get('UNKNOWN', 0) + counts.get('NO DATA', 0)
    
    content.append(f"- **{suite_name} Tests:** {total}")
    if n_info > 0:
        content.append(f"- **INFO:** {n_info}  |  **WARN:** {n_warn}  |  **FAIL:** {n_fail}  |  **PASS:** {n_pass}")
    else:
        content.append(f"- **PASS:** {n_pass}  |  **WARN:** {n_warn}  |  **FAIL:** {n_fail}")
    
    overall = aggregate_status(status_data)
    content.append(f"- **Overall Status:** {overall}")
    content.append("")
    
    return content


def generate_test_section(test_name, status, metrics, plots, output_dir, model=None):
    """Generate a per-test section with status, metrics, and plots.
    
    Args:
        test_name: Name of the test
        status: Test status (PASS/WARN/FAIL/INFO)
        metrics: Dict of metrics (e.g., {"l2_norm": 1e-5, "linf_norm": 1e-4})
        plots: List of plot file paths
        output_dir: Output directory for computing relative paths
        model: Optional model name
    
    Returns:
        List of markdown lines for the test section
    """
    content = []
    
    # Test header
    display_name = test_name.replace('_', ' ').title()
    if model:
        display_name = f"{model.upper()} - {display_name}"
    content.append(f"### {display_name}")
    content.append("")
    content.append(f"**Status:** {status}")
    content.append("")
    
    # Metrics
    if metrics:
        metric_parts = []
        for k, v in metrics.items():
            if isinstance(v, float):
                metric_parts.append(f"{k}={v:.2e}")
            else:
                metric_parts.append(f"{k}={v}")
        if metric_parts:
            content.append(f"**Metrics:** {', '.join(metric_parts)}")
            content.append("")
    
    # Plots
    if plots:
        for plot_file in plots:
            rel_path = embed_plot_image(plot_file, output_dir)
            caption = Path(plot_file).stem.replace('_', ' ').title()
            content.append(f"![{caption}]({rel_path})")
            content.append("")
    else:
        content.append("*No plots available*")
        content.append("")
    
    return content


def format_metrics(metrics):
    """Format metrics dict for display.
    
    Args:
        metrics: Dict of metric name -> value
    
    Returns:
        Formatted string
    """
    if not metrics:
        return "-"
    
    parts = []
    for k, v in metrics.items():
        if isinstance(v, float):
            parts.append(f"{k}={v:.2e}")
        else:
            parts.append(f"{k}={v}")
    
    return ", ".join(parts)


def embed_plot_image(plot_path, output_dir, caption=None):
    """Generate relative image path for embedding in markdown.
    
    Args:
        plot_path: Path to plot file (Path object or string)
        output_dir: Output directory for the report
        caption: Optional caption (not used, kept for API compatibility)
    
    Returns:
        Relative path string suitable for markdown image embedding
    """
    plot_path = Path(plot_path)
    output_dir = Path(output_dir)
    
    try:
        rel = os.path.relpath(plot_path, output_dir).replace('\\', '/')
        # Handle spaces in filenames
        rel = rel.replace(' ', '%20')
        return rel
    except (ValueError, OSError):
        # If relative path fails, return filename only
        return plot_path.name


def generate_pdf(
    markdown_file,
    output_pdf,
    html_styling=False,
    quiet=False,
    report_title: Optional[str] = None,
    report_subtitle: Optional[str] = None,
):
    """Convert markdown to PDF using pandoc.
    
    Args:
        markdown_file: Path to markdown file
        output_pdf: Path for output PDF file
        html_styling: Whether to use HTML styling (not used in pandoc PDF conversion)
        quiet: If True, suppress success-only messages (errors still print)
        report_title: Pandoc ``title`` metadata (default: generic SEA-Stack title)
        report_subtitle: Pandoc ``subtitle`` metadata (optional)
    
    Returns:
        Path to PDF file if successful, None otherwise
    """
    markdown_file = Path(markdown_file)
    output_pdf = Path(output_pdf)
    
    # Check for pandoc
    try:
        result = subprocess.run(['pandoc', '--version'], capture_output=True, text=True)
        if result.returncode != 0:
            raise FileNotFoundError("pandoc")
    except FileNotFoundError:
        print("ERROR: pandoc is required for PDF report generation but was not found.")
        print("Install pandoc:")
        print("  Windows:  winget install JohnMacFarlane.Pandoc")
        print("  macOS:    brew install pandoc")
        print("  Linux:    sudo apt install pandoc")
        return None
    
    # Build pandoc command
    # Use resolved absolute paths to avoid Windows backslash issues
    markdown_abs = markdown_file.resolve()
    output_abs = output_pdf.resolve()
    doc_title = report_title or 'SEA-Stack Test Report'
    base_cmd = [
        'pandoc',
        str(markdown_abs),
        '-o', str(output_abs),
        '--standalone',
        '--metadata', f'title={doc_title}',
        '--metadata', f'date={datetime.now().strftime("%B %d, %Y")}',
    ]
    if report_subtitle:
        base_cmd.extend(['--metadata', f'subtitle={report_subtitle}'])

    # Pandoc wraps Markdown images in LaTeX figure floats by default; figures can
    # then drift away from their section and appear out of order. Pin placement.
    last_stderr = ''
    fd, header_path = tempfile.mkstemp(suffix='.tex', text=True)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as header_file:
            header_file.write(
                '% SEA-Stack: keep figures in Markdown source order in PDF output.\n'
                '\\usepackage{float}\n'
                '\\floatplacement{figure}{H}\n'
            )
        include_header = ['--include-in-header', header_path]

        pdf_engines = ['xelatex', 'pdflatex', 'lualatex']
        for engine in pdf_engines:
            cmd_with_engine = (
                base_cmd + include_header + ['--pdf-engine=' + engine]
            )
            result = subprocess.run(
                cmd_with_engine,
                capture_output=True,
                text=True,
                cwd=markdown_abs.parent,
            )
            last_stderr = result.stderr or ''

            if result.returncode == 0:
                if not quiet:
                    print(f"SUCCESS: PDF generated: {output_pdf}")
                return output_pdf

            if (
                f'{engine} not found' in last_stderr
                or 'not found' in last_stderr.lower()
            ):
                continue
    finally:
        try:
            os.unlink(header_path)
        except OSError:
            pass

    if 'pdflatex not found' in last_stderr or 'xelatex not found' in last_stderr:
        print("ERROR: LaTeX is required for PDF report generation but was not found.")
        print("Install a LaTeX distribution:")
        print("  Windows:  winget install MiKTeX.MiKTeX")
        print("  macOS:    brew install --cask mactex")
        print("  Linux:    sudo apt install texlive-latex-recommended texlive-fonts-recommended")
        return None

    print(f"ERROR: pandoc failed: {last_stderr.strip()}")
    return None


def aggregate_status(status_data):
    """Compute overall status from a list of status records.
    
    Args:
        status_data: List of status records (from find_status_files) or list of status strings
    
    Returns:
        Overall status string: "FAIL", "WARN", "INFO", "PASS", or "NO DATA"
    """
    if not status_data:
        return "NO DATA"
    
    # Extract status strings if records contain status_data
    statuses = []
    for item in status_data:
        if isinstance(item, dict):
            if 'status_data' in item:
                statuses.append(item['status_data'].get('status', 'UNKNOWN'))
            elif 'status' in item:
                statuses.append(item['status'])
        elif isinstance(item, str):
            statuses.append(item)
    
    if not statuses:
        return "NO DATA"
    
    if "FAIL" in statuses:
        return "FAIL"
    elif "WARN" in statuses:
        return "WARN"
    elif "INFO" in statuses:
        return "INFO"
    else:
        return "PASS"


def count_by_status(status_data):
    """Count tests by status.
    
    Args:
        status_data: List of status records (from find_status_files) or list of status strings
    
    Returns:
        Dict mapping status -> count
    """
    counts = defaultdict(int)
    
    for item in status_data:
        if isinstance(item, dict):
            if 'status_data' in item:
                status = item['status_data'].get('status', 'UNKNOWN')
            elif 'status' in item:
                status = item['status']
            else:
                continue
        elif isinstance(item, str):
            status = item
        else:
            continue
        
        counts[status] += 1
    
    return dict(counts)


def get_status_summary(status_data):
    """Generate summary text from status data.
    
    Args:
        status_data: List of status records (from find_status_files)
    
    Returns:
        Summary string
    """
    counts = count_by_status(status_data)
    total = len(status_data)
    
    if total == 0:
        return "No test data available."
    
    parts = []
    for status in ['PASS', 'WARN', 'FAIL', 'INFO', 'UNKNOWN', 'NO DATA']:
        count = counts.get(status, 0)
        if count > 0:
            parts.append(f"{status}: {count}")
    
    return f"Total: {total} ({', '.join(parts)})"
