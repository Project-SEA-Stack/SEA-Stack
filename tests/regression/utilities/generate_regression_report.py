#!/usr/bin/env python3
"""
SEA-Stack Regression Test Report Generator

This script generates a comprehensive markdown report of all regression test results,
including comparison plots and summary statistics. The report can optionally be
converted to PDF using pandoc.

Usage:
    python generate_regression_report.py [--pdf] [--output-dir <dir>]
"""

import os
import sys
import argparse
from pathlib import Path
from datetime import datetime
import subprocess
from collections import defaultdict

# Import shared report generation infrastructure
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "utilities"))
from report_generator import (
    find_status_files,
    find_plot_files as shared_find_plot_files,
    generate_report_header,
    embed_plot_image,
    generate_pdf as shared_generate_pdf,
)

def natural_sort_key(filename):
    """Sort key for natural sorting of filenames with numbers."""
    import re
    # Split filename into text and number parts for natural sorting
    return [int(text) if text.isdigit() else text.lower() 
            for text in re.split('([0-9]+)', str(filename))]

def find_plot_files(build_dir, quiet=False, config=None):
    """Find all comparison plot files in the build directory.
    
    Uses shared infrastructure but organizes results by model for regression-specific logic.
    """
    # Use shared infrastructure to find plot files
    plot_records = shared_find_plot_files(
        build_dir,
        suite_filter='regression',
        pattern="*_comparison.png",
        config=config,
    )
    
    # Organize by model (regression-specific grouping)
    plots = defaultdict(list)
    models = ['sphere', 'f3of', 'oswec', 'rm3']
    
    for record in plot_records:
        model = record.get('model')
        plot_path = record.get('path')
        
        if model and model in models and plot_path:
            plots[model].append(plot_path)
    
    # Sort plots within each model using natural sorting
    for model in plots:
        plots[model] = sorted(plots[model], key=lambda x: natural_sort_key(x.name))
        if not quiet:
            print(f"Found {len(plots[model])} plots for {model}")
    
    # Check for models with no plots
    for model in models:
        if model not in plots:
            if not quiet:
                print(f"No plots directory found for {model}")
    
    return plots

def categorize_plots(plots):
    """Categorize plots by test type (decay, regular_waves, irregular_waves, etc.)."""
    categorized = defaultdict(lambda: defaultdict(list))
    
    for model, plot_files in plots.items():
        for plot_file in plot_files:
            filename = plot_file.name.lower()
            
            # Categorize based on filename patterns
            if 'consistency' in filename:
                category = 'consistency'
            elif 'decay' in filename:
                category = 'decay'
            elif 'irregular' in filename or 'irreg_waves' in filename:
                category = 'irregular_waves'
            elif 'regular' in filename or 'reg_waves' in filename:
                category = 'regular_waves'
            elif 'decay_c1' in filename or 'decay_c2' in filename or 'decay_c3' in filename:
                category = 'decay'  # F3OF tests are decay tests
            else:
                category = 'other'
            
            categorized[model][category].append(plot_file)
        
        # Sort plots within each category using natural sorting
        for category in categorized[model]:
            categorized[model][category] = sorted(categorized[model][category], 
                                                key=lambda x: natural_sort_key(x.name))
    
    return categorized


def _read_status_files(build_dir, config=None):
    """Scan for .status.json files written by comparison scripts.
    
    These files are the most reliable source of pass/fail information because
    they are written by the comparison scripts themselves and persist across
    ctest invocations (unlike LastTest.log which is overwritten each run).
    
    Uses shared infrastructure but converts to regression-specific format.
    
    Returns:
        dict mapping (model, status_name) -> {"status": "PASS"/"FAIL", ...}
    """
    # Use shared infrastructure to find status files
    status_records = find_status_files(
        build_dir, suite_filter='regression', config=config
    )
    
    # Convert to regression-specific format: (model, status_name) -> payload
    results = {}
    for record in status_records:
        model = record.get('model')
        status_data = record.get('status_data', {})
        status_name = status_data.get("test_name", record.get('test_name'))
        
        if model and status_name:
            results[(model, status_name)] = status_data
    
    return results


# Mapping from status-file test names to the report's (model, test_type) pairs.
# Keys are substrings matched against the status-file test_name field.
# Canonical status-file names → report (model, test_type).
# Each key is a prefix matched against the status-file test_name.
# Longest prefix wins, so "sphere_irreg_waves_eta_consistency" is tried
# before "sphere_irreg_waves_eta" which is tried before "sphere_irreg_waves".
_STATUS_NAME_TO_REPORT_TYPE = {
    'sphere_decay_ss':                      ('sphere', 'decay_state_space'),
    'sphere_decay':                         ('sphere', 'decay'),
    'sphere_reg_waves':                     ('sphere', 'regular_waves'),
    'sphere_irreg_waves_eta_consistency':    ('sphere', 'consistency'),
    'sphere_irreg_waves_eta':               ('sphere', 'irregular_waves_eta'),
    'sphere_irreg_waves_ss':                ('sphere', 'irregular_waves'),
    'sphere_irreg_waves':                   ('sphere', 'irregular_waves'),
    'f3of_decay_c1':                             ('f3of', 'decay'),
    'f3of_decay_c2':                             ('f3of', 'decay'),
    'f3of_decay_c3':                             ('f3of', 'decay'),
    'oswec_decay_ss':                       ('oswec', 'decay_state_space'),
    'oswec_decay':                          ('oswec', 'decay'),
    'oswec_reg_waves':                      ('oswec', 'regular_waves'),
    'oswec_irreg_waves_ss':                 ('oswec', 'irregular_waves'),
    'oswec_irreg_waves':                    ('oswec', 'irregular_waves'),
    'rm3_decay':                            ('rm3', 'decay'),
    'rm3_reg_waves':                        ('rm3', 'regular_waves'),
}


def _classify_status_name(model, status_name):
    """Map a status-file test name to the report's (model, test_type) pair."""
    # Try exact match first, then prefix match (longest prefix wins)
    for key in sorted(_STATUS_NAME_TO_REPORT_TYPE, key=len, reverse=True):
        if status_name == key or status_name.startswith(key):
            mapped_model, test_type = _STATUS_NAME_TO_REPORT_TYPE[key]
            if mapped_model == model:
                return (model, test_type)
    return None


def infer_config_from_output_dir(output_dir: Path):
    """Return CMake config name if ``output_dir`` is under ``bin/<Config>/``."""
    parts = output_dir.resolve().parts
    for i, p in enumerate(parts):
        if p == "bin" and i + 1 < len(parts):
            cand = parts[i + 1]
            if cand in (
                "Release",
                "Debug",
                "RelWithDebInfo",
                "MinSizeRel",
            ):
                return cand
    return None


def get_test_results(categorized_plots, build_dir=None, quiet=False, config=None):
    """Get test results from status files (primary) or CTest logs (fallback).
    
    Status files (.status.json) are written by comparison scripts and persist
    across ctest runs.  CTest's LastTest.log is ephemeral — it only contains
    the results from the *most recent* ctest invocation.
    """
    test_results = {
        'sphere': {},
        'f3of': {},
        'oswec': {},
        'rm3': {}
    }
    
    # Default status: plots exist but we don't know pass/fail yet
    for model, model_plots in categorized_plots.items():
        if model not in test_results:
            test_results[model] = {}
        for test_type, plots in model_plots.items():
            test_results[model][test_type] = 'UNKNOWN' if plots else 'NO DATA'
    
    # -----------------------------------------------------------------
    # 1. Primary source: persistent .status.json files
    # -----------------------------------------------------------------
    status_files = _read_status_files(build_dir, config=config)
    status_found = False
    
    for (model, status_name), payload in status_files.items():
        mapping = _classify_status_name(model, status_name)
        if not mapping:
            continue
        m, test_type = mapping
        status = payload.get("status", "UNKNOWN")
        if m in test_results:
            existing = test_results[m].get(test_type)
            # Don't overwrite FAIL with PASS (any sub-test failure → FAIL)
            if existing == 'FAIL':
                continue
            test_results[m][test_type] = status
            status_found = True
    
    if status_found and not quiet:
        print("Successfully read status files from comparison scripts.")
    
    # Count how many slots are still UNKNOWN after reading status files
    unknown_count = sum(
        1 for m in test_results for t, s in test_results[m].items() if s == 'UNKNOWN'
    )
    
    if unknown_count == 0:
        return test_results
    
    # -----------------------------------------------------------------
    # 2. Fallback: parse CTest LastTest.log for any remaining UNKNOWN
    # -----------------------------------------------------------------
    import re
    
    search_roots = []
    if build_dir:
        search_roots.append(Path(build_dir).resolve())
    search_roots.append(Path.cwd())
    
    ctest_log_patterns = []
    for root in search_roots:
        ctest_log_patterns.append(root / "Testing" / "Temporary" / "LastTest.log")
        ctest_log_patterns.append(root / "Testing" / "Temporary" / "LastTest.log.tmp")
    
    test_name_map = {
        'test_regression_sphere_decay_ss_reference': ('sphere', 'decay_state_space'),
        'test_regression_sphere_decay_reference': ('sphere', 'decay'),
        'test_regression_sphere_reg_waves_reference': ('sphere', 'regular_waves'),
        'test_regression_sphere_irreg_waves_ss_reference': ('sphere', 'irregular_waves'),
        'test_regression_sphere_irreg_waves_reference': ('sphere', 'irregular_waves'),
        'test_regression_sphere_irreg_waves_eta_reference': ('sphere', 'irregular_waves_eta'),
        'test_regression_sphere_irreg_waves_eta_consistency_reference': ('sphere', 'consistency'),
        'test_regression_f3of_decay_c1_reference': ('f3of', 'decay'),
        'test_regression_f3of_decay_c2_reference': ('f3of', 'decay'),
        'test_regression_f3of_decay_c3_reference': ('f3of', 'decay'),
        'test_regression_oswec_decay_ss_reference': ('oswec', 'decay_state_space'),
        'test_regression_oswec_decay_reference': ('oswec', 'decay'),
        'test_regression_oswec_reg_waves_reference': ('oswec', 'regular_waves'),
        'test_regression_oswec_irreg_waves_ss_reference': ('oswec', 'irregular_waves'),
        'test_regression_oswec_irreg_waves_reference': ('oswec', 'irregular_waves'),
        'test_regression_rm3_decay_reference': ('rm3', 'decay'),
        'test_regression_rm3_reg_waves_reference': ('rm3', 'regular_waves'),
    }
    for _n in range(1, 11):
        test_name_map[f'test_regression_sphere_reg_waves_c{_n}_reference'] = (
            'sphere', 'regular_waves')
    for _n in range(1, 17):
        test_name_map[f'test_regression_oswec_reg_waves_c{_n}_reference'] = (
            'oswec', 'regular_waves')
    
    ctest_found = False
    for log_file in ctest_log_patterns:
        if log_file.exists():
            try:
                with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                
                test_blocks = re.findall(
                    r'\d+/\d+\s+Testing:\s+(\S+).*?Test\s+(Passed|Failed)\.',
                    content, re.DOTALL)
                
                for test_name, status in test_blocks:
                    mapping = test_name_map.get(test_name)
                    if not mapping:
                        continue
                    model, test_type = mapping
                    
                    # Only fill in slots still showing UNKNOWN
                    if model in test_results:
                        existing = test_results[model].get(test_type)
                        if existing not in ('UNKNOWN',):
                            continue
                        result_status = 'PASS' if status == 'Passed' else 'FAIL'
                        test_results[model][test_type] = result_status
                        ctest_found = True
                                
            except Exception as e:
                print(f"Warning: Could not parse CTest log {log_file}: {e}")
    
    for root in search_roots:
        failed_log = root / "Testing" / "Temporary" / "LastTestsFailed.log"
        if failed_log.exists():
            try:
                with open(failed_log, 'r', encoding='utf-8', errors='ignore') as f:
                    for line in f:
                        parts = line.strip().split(':', 1)
                        if len(parts) == 2:
                            failed_name = parts[1].strip()
                            mapping = test_name_map.get(failed_name)
                            if mapping:
                                model, test_type = mapping
                                if model in test_results:
                                    test_results[model][test_type] = 'FAIL'
                                    ctest_found = True
            except Exception as e:
                print(f"Warning: Could not parse {failed_log}: {e}")
    
    if ctest_found and not quiet:
        print("Supplemented with CTest log results.")
    
    # Final summary
    remaining = sum(
        1 for m in test_results for t, s in test_results[m].items() if s == 'UNKNOWN'
    )
    if remaining and not quiet:
        print(f"Note: {remaining} test(s) still show UNKNOWN — re-run the regression "
              "tests to generate status files.")
    
    return test_results

# _relative_plot_path is now replaced by embed_plot_image from shared infrastructure

def generate_markdown_report(
    categorized_plots,
    output_dir,
    build_dir,
    html_styling=False,
    quiet=False,
    config=None,
):
    """Generate the markdown report content.

    Returns:
        (markdown_text, test_results) so the caller can inspect pass/fail.
    """
    
    # Compute test results up-front so we can derive the overall status for the summary table.
    test_results = get_test_results(
        categorized_plots, build_dir=build_dir, quiet=quiet, config=config
    )
    all_statuses = [s for model in test_results.values() for s in model.values()]
    if "FAIL" in all_statuses:
        overall_status = "FAIL"
    elif "WARN" in all_statuses:
        overall_status = "WARN"
    else:
        overall_status = "PASS"
    
    # Start building the markdown content
    content = []
    
    # Use shared header generation (but preserve HTML styling option)
    header_lines = generate_report_header(
        "Regression",
        "Compares current outputs to stored reference data for core cases."
    )
    
    if html_styling:
        # Wrap header in HTML styling if requested
        content.append('<div class="header">')
        content.append('<h1>SEA-Stack Regression Test Report</h1>')
        content.append('<p class="subtitle">Compares current outputs to stored reference data for core cases.</p>')
        content.append(f'<p class="date">Generated on {datetime.now().strftime("%B %d, %Y at %H:%M:%S")}</p>')
        content.append('</div>')
    else:
        content.extend(header_lines)
    
    n_cases = sum(len(test_results[m]) for m in test_results)

    # Summary (aligned with verification / comparison report PDFs)
    if html_styling:
        content.append('<div class="executive-summary">')
    content.append("## Summary")
    content.append("")
    content.append(f"- **Cases included:** {n_cases}")
    content.append(f"- **Outcome:** **{overall_status}**")
    content.append("")
    if html_styling:
        content.append('</div>')
    content.append("")
    
    # Case status table
    if html_styling:
        content.append('<div class="test-results">')
    content.append("## Results")
    content.append("")
    content.append("Summary of each regression category:")
    content.append("")
    content.append("| Model | Test Type | Status |")
    content.append("|-------|-----------|--------|")
    
    for model in ['sphere', 'oswec', 'rm3', 'f3of']:
        if model in test_results:
            for test_type, status in test_results[model].items():
                if html_styling:
                    if status == "PASS":
                        status_class = "status-pass"
                    elif status == "WARN":
                        status_class = "status-warn"
                    elif status == "FAIL":
                        status_class = "status-fail"
                    else:
                        status_class = "status-unknown"
                    status_badge = f'<span class="{status_class}">{status}</span>'
                else:
                    status_badge = status
                content.append(f"| {model.upper()} | {test_type.replace('_', ' ').title()} | {status_badge} |")
    content.append("")
    if html_styling:
        content.append('</div>')
    content.append("")
    
    # Add page break before detailed test results section
    content.append("\\clearpage")
    content.append("")
    
    # Detailed results by model
    for i, model in enumerate(['sphere', 'oswec', 'rm3', 'f3of']):
        if model in categorized_plots:
            # Add clear page break before each model section (except the first)
            if i > 0:
                content.append("\\clearpage")
                content.append("")
            
            if html_styling:
                content.append('<div class="model-section">')
            content.append(f"## {model.upper()} Tests")
            content.append("")
        
        model_plots = categorized_plots[model]
        
        # Track if this is the first subsection for this model
        first_subsection = True
        
        # Decay tests
        if 'decay' in model_plots and model_plots['decay']:
            if not first_subsection:
                content.append("\\clearpage")
                content.append("")
            first_subsection = False
            
            if html_styling:
                content.append('<div class="test-subsection">')
            content.append("### Decay Tests")
            content.append("")
            for plot_file in model_plots['decay']:
                plot_name = plot_file.stem.replace('_', ' ').replace('comparison', '').strip()
                relative_path = embed_plot_image(plot_file, output_dir)
                
                test_type = "Decay Test"
                # Get the actual test status
                status = test_results.get(model, {}).get('decay', 'UNKNOWN')
                caption = f"**{model.upper()} - {test_type}** - Regression Test {status}"
                
                if html_styling:
                    content.append('<div class="image-container">')
                    content.append(f'<div class="image-title">{caption}</div>')
                    content.append(f"![{caption}]({relative_path})")
                    content.append('</div>')
                else:
                    content.append(f"![{caption}]({relative_path})")
                content.append("")
            if html_styling:
                content.append('</div>')
        
        # Regular waves tests
        if 'regular_waves' in model_plots and model_plots['regular_waves']:
            if not first_subsection:
                content.append("\\clearpage")
                content.append("")
            first_subsection = False
            
            if html_styling:
                content.append('<div class="test-subsection">')
            content.append("### Regular Waves Tests")
            content.append("")
            for plot_file in model_plots['regular_waves']:
                plot_name = plot_file.stem.replace('_', ' ').replace('comparison', '').strip()
                relative_path = embed_plot_image(plot_file, output_dir)
                
                # Extract wave number if present
                wave_num = ""
                if "wave" in plot_name.lower():
                    import re
                    wave_match = re.search(r'wave\s*(\d+)', plot_name.lower())
                    if wave_match:
                        wave_num = f" Wave {wave_match.group(1)}"
                
                test_type = f"Regular Waves{wave_num}"
                # Get the actual test status
                status = test_results.get(model, {}).get('regular_waves', 'UNKNOWN')
                caption = f"**{model.upper()} - {test_type}** - Regression Test {status}"
                
                if html_styling:
                    content.append('<div class="image-container">')
                    content.append(f'<div class="image-title">{caption}</div>')
                    content.append(f"![{caption}]({relative_path})")
                    content.append('</div>')
                else:
                    content.append(f"![{caption}]({relative_path})")
                content.append("")
            if html_styling:
                content.append('</div>')
        
        # Irregular waves tests
        if 'irregular_waves' in model_plots and model_plots['irregular_waves']:
            if not first_subsection:
                content.append("\\clearpage")
                content.append("")
            first_subsection = False
            
            if html_styling:
                content.append('<div class="test-subsection">')
            content.append("### Irregular Waves Tests")
            content.append("")
            for plot_file in model_plots['irregular_waves']:
                plot_name = plot_file.stem.replace('_', ' ').replace('comparison', '').strip()
                relative_path = embed_plot_image(plot_file, output_dir)
                
                test_type = "Irregular Waves"
                # Get the actual test status
                status = test_results.get(model, {}).get('irregular_waves', 'UNKNOWN')
                caption = f"**{model.upper()} - {test_type}** - Regression Test {status}"
                
                if html_styling:
                    content.append('<div class="image-container">')
                    content.append(f'<div class="image-title">{caption}</div>')
                    content.append(f"![{caption}]({relative_path})")
                    content.append('</div>')
                else:
                    content.append(f"![{caption}]({relative_path})")
                content.append("")
            if html_styling:
                content.append('</div>')
        
        # Consistency tests
        if 'consistency' in model_plots and model_plots['consistency']:
            if not first_subsection:
                content.append("\\clearpage")
                content.append("")
            first_subsection = False
            
            if html_styling:
                content.append('<div class="test-subsection">')
            content.append("### Consistency Tests")
            content.append("")
            for plot_file in model_plots['consistency']:
                plot_name = plot_file.stem.replace('_', ' ').replace('comparison', '').strip()
                relative_path = embed_plot_image(plot_file, output_dir)
                
                test_type = "Consistency Test"
                status = test_results.get(model, {}).get('consistency', 'UNKNOWN')
                caption = f"**{model.upper()} - {test_type}** - {status}"
                
                if html_styling:
                    content.append('<div class="image-container">')
                    content.append(f'<div class="image-title">{caption}</div>')
                    content.append(f"![{caption}]({relative_path})")
                    content.append('</div>')
                else:
                    content.append(f"![{caption}]({relative_path})")
                content.append("")
            if html_styling:
                content.append('</div>')
        
        # Other tests
        if 'other' in model_plots and model_plots['other']:
            if not first_subsection:
                content.append("\\clearpage")
                content.append("")
            first_subsection = False
            
            if html_styling:
                content.append('<div class="test-subsection">')
            content.append("### Other Tests")
            content.append("")
            for plot_file in model_plots['other']:
                plot_name = plot_file.stem.replace('_', ' ').replace('comparison', '').strip()
                relative_path = embed_plot_image(plot_file, output_dir)
                
                test_type = "Other Test"
                status = test_results.get(model, {}).get('other', 'UNKNOWN')
                caption = f"**{model.upper()} - {test_type}** - Regression Test {status}"
                
                if html_styling:
                    content.append('<div class="image-container">')
                    content.append(f'<div class="image-title">{caption}</div>')
                    content.append(f"![{caption}]({relative_path})")
                    content.append('</div>')
                else:
                    content.append(f"![{caption}]({relative_path})")
                content.append("")
            if html_styling:
                content.append('</div>')
            
        if model in categorized_plots and html_styling:
            content.append('</div>')  # Close model-section
    
    # Footer
    if html_styling:
        content.append('<div class="footer">')
    content.append("---")
    content.append("")
    content.append("*This report was automatically generated by the SEA-Stack regression test suite.*")
    if html_styling:
        content.append('</div>')
    
    return '\n'.join(content), test_results

def convert_to_pdf(markdown_file, output_dir, quiet=False):
    """Convert markdown to PDF using shared infrastructure."""
    pdf_file = Path(output_dir) / "regression_test_report.pdf"
    if not quiet:
        print("Converting markdown to PDF via pandoc + LaTeX...")
    return shared_generate_pdf(
        markdown_file,
        pdf_file,
        quiet=quiet,
        report_title="SEA-Stack Regression Test Report",
        report_subtitle=(
            "Compares current outputs to stored reference data for core cases."
        ),
    )

def rerun_comparisons(build_dir, config="Release", quiet=False):
    """Re-run CTest comparison tests to regenerate plot images.
    
    This ensures plots reflect the current reference data. Without this step,
    plots may be stale if reference data was updated after the last test run.
    
    Note: The comparison tests use CTest FIXTURES_REQUIRED, which means CTest
    will skip them unless the simulation fixture tests are also selected.
    We use -FA "." to exclude all fixture requirements so the comparison
    scripts run directly (the simulation output files already exist on disk).
    """
    if not quiet:
        print("Re-running comparison tests to regenerate plots...")
    build_path = Path(build_dir).resolve()
    
    cmd = [
        "ctest",
        "--test-dir", str(build_path),
        "-C", config,
        "-L", "regression",
        "-LE", "report",
        "-R", "_reference$",
        "-FA", ".",
        "--output-on-failure"
    ]
    if quiet:
        cmd.append("-Q")
    
    result = subprocess.run(cmd, capture_output=False)
    if result.returncode != 0:
        print("WARNING: Some comparison tests failed (plots were still regenerated).")
    elif not quiet:
        print("All comparison tests passed.")
    if not quiet:
        print()


def main():
    parser = argparse.ArgumentParser(description='Generate SEA-Stack regression test report')
    parser.add_argument('--output-dir', help='Output directory for reports (default: build/bin/tests/regression/report)')
    parser.add_argument('--build-dir', default='build', help='Build directory path')
    parser.add_argument('--pdf', action='store_true', help='Generate PDF report (requires pandoc + LaTeX)')
    parser.add_argument('--html-styling', action='store_true', help='Include HTML styling in markdown')
    parser.add_argument('--recompare', action='store_true',
                        help='Re-run CTest comparison tests before generating the report. '
                             'Use this after updating reference data to ensure plots are current.')
    parser.add_argument('--config', default='Release', help='Build configuration (default: Release)')
    parser.add_argument('--exit-on-failure', action='store_true',
                        help='Exit with code 1 if any regression test failed. '
                             'Useful for CI: makes the report CTest entry fail '
                             'when a regression test has failed.')
    parser.add_argument('--quiet', action='store_true',
                        help='Less console output (warnings and errors still print)')
    
    args = parser.parse_args()
    
    # Re-run comparison tests if requested
    if args.recompare:
        rerun_comparisons(args.build_dir, args.config, quiet=args.quiet)
    
    if args.output_dir is None:
        output_dir = (
            Path(args.build_dir)
            / "bin"
            / args.config
            / "results"
            / "tests"
            / "regression"
            / "report"
        )
    else:
        output_dir = Path(args.output_dir)

    discovery_config = infer_config_from_output_dir(output_dir) or args.config

    # Ensure output directory exists
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Find all plot files
    if not args.quiet:
        print("Scanning for comparison plots...")
    plots = find_plot_files(
        args.build_dir, quiet=args.quiet, config=discovery_config
    )
    
    if not plots:
        print("WARNING: No comparison plots found — generating partial report.")
        print(
            "   Expected location: "
            f"{args.build_dir}/bin/{discovery_config}/results/tests/regression/*/plots/"
        )
    
    # Categorize plots
    if not args.quiet:
        print("Categorizing plots by test type...")
    categorized_plots = categorize_plots(plots)
    
    # Generate markdown report
    if not args.quiet:
        print("Generating markdown report...")
    markdown_content, test_results = generate_markdown_report(
        categorized_plots,
        output_dir,
        args.build_dir,
        html_styling=args.html_styling,
        quiet=args.quiet,
        config=discovery_config,
    )
    
    # Write markdown file
    markdown_file = output_dir / "regression_test_report.md"
    with open(markdown_file, 'w', encoding='utf-8') as f:
        f.write(markdown_content)
    
    if not args.quiet:
        print(f"SUCCESS: Clean markdown report generated: {markdown_file}")
    
    # Generate PDF if requested
    if args.pdf:
        pdf_file = convert_to_pdf(markdown_file, output_dir, quiet=args.quiet)
        if pdf_file:
            if not args.quiet:
                print(f"Report available in both formats:")
                print(f"   - Markdown: {markdown_file}")
                print(f"   - PDF: {pdf_file}")
        else:
            print(f"PDF generation failed. Report available as markdown: {markdown_file}")
    else:
        if not args.quiet:
            print(f"Report available as clean markdown: {markdown_file}")
            print("Use --pdf flag to generate PDF (requires pandoc + LaTeX)")
    
    # Propagate failures when requested (e.g. from CTest report entry)
    if args.exit_on_failure:
        all_statuses = [s for model in test_results.values()
                        for s in model.values()]
        if "FAIL" in all_statuses:
            failed = [(m, t) for m, types in test_results.items()
                      for t, s in types.items() if s == "FAIL"]
            print(f"\nFAILED tests detected ({len(failed)}):")
            for m, t in failed:
                print(f"  - {m}/{t}")
            sys.exit(1)

if __name__ == "__main__":
    main() 