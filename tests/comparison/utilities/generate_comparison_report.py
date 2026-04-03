#!/usr/bin/env python3
"""
SEA-Stack Comparison Test Report Generator

Generates a comparison test report (markdown + optional PDF) from .status.json
files and plots written by comparison test scripts. This scans the per-test
directory structure to aggregate all comparison test results.

Usage:
    python generate_comparison_report.py --build-dir <dir> --output-dir <dir> [--pdf] [--exit-on-failure]
"""

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Optional

# Import shared report generation infrastructure
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "utilities"))
from report_generator import (
    embed_plot_image,
    generate_pdf as shared_generate_pdf,
    generate_report_header,
    results_tests_search_bases,
)


def infer_config_from_output_dir(output_dir: Path) -> Optional[str]:
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


def find_comparison_tests(build_dir, config=None):
    """Scan for comparison test directories and collect status/plot information."""
    build_path = Path(build_dir).resolve() if build_dir else Path.cwd()

    search_dirs = []
    for base in results_tests_search_bases(build_path, config):
        search_dirs.append(base / "comparison")

    tests = defaultdict(dict)  # {model: {test_name: {status, metrics, plots}}}

    for search_dir in search_dirs:
        if not search_dir.exists():
            continue

        # Walk through model directories
        for model_dir in search_dir.iterdir():
            if not model_dir.is_dir():
                continue

            model = model_dir.name

            # Walk through test directories
            for test_dir in model_dir.iterdir():
                if not test_dir.is_dir():
                    continue

                test_name = test_dir.name

                # Read status.json if it exists
                status_file = test_dir / "status.json"
                status_data = None
                if status_file.exists():
                    try:
                        with open(status_file, 'r', encoding='utf-8') as f:
                            status_data = json.load(f)
                    except Exception as e:
                        print(f"Warning: could not read {status_file}: {e}")

                # Find plots
                plots_dir = test_dir / "plots"
                plots = []
                if plots_dir.exists():
                    plots = sorted(plots_dir.glob("*.png"))

                tests[model][test_name] = {
                    "status": status_data.get("status", "UNKNOWN") if status_data else "NO DATA",
                    "metrics": status_data.get("metrics", {}) if status_data else {},
                    "note": status_data.get("note") if status_data else None,
                    "plots": plots,
                    "test_dir": test_dir,
                }

    return tests


def generate_report(build_dir, output_dir, html_styling=False, config=None):
    """Generate the comparison test report markdown.

    Returns:
        (markdown_text, test_data)
    """
    tests = find_comparison_tests(build_dir, config=config)

    if not tests:
        return "# SEA-Stack Comparison Test Report\n\nNo comparison tests found.\n", {}

    all_statuses = []
    for model_tests in tests.values():
        for test_data in model_tests.values():
            all_statuses.append(test_data["status"])

    total_tests = sum(len(model_tests) for model_tests in tests.values())
    if not all_statuses:
        outcome_line = "No comparison data was found."
    elif "FAIL" in all_statuses:
        outcome_line = (
            "**Failed** — one or more method comparisons exceeded failure thresholds."
        )
    else:
        outcome_line = (
            "**Passed** — all comparisons completed successfully "
            "(diagnostic summaries are not shown in this report)."
        )

    content = []

    header_lines = generate_report_header(
        "Comparison",
        "Internal method comparisons within SEA-Stack",
    )
    content.extend(header_lines)

    content.append("## Summary")
    content.append("")
    content.append(f"- **Cases included:** {total_tests}")
    content.append(f"- **Outcome:** {outcome_line}")
    content.append("")

    if tests:
        content.append("## Results")
        content.append("")

        for model in sorted(tests.keys()):
            content.append("\\clearpage")
            content.append("")
            content.append(f"### {model.capitalize()}")
            content.append("")
            content.append("\\nopagebreak")
            content.append("")

            test_names = sorted(tests[model].keys())
            for ti, test_name in enumerate(test_names):
                if ti > 0:
                    content.append("\\clearpage")
                    content.append("")

                test_data = tests[model][test_name]
                status = test_data["status"]
                plots = test_data["plots"]
                note = test_data.get("note")
                if not isinstance(note, str):
                    note = None

                if status == "FAIL":
                    content.append(
                        "**Failed** — see logs and `status.json` under this case for metrics."
                    )
                    content.append("")
                    if note:
                        content.append(note)
                        content.append("")
                elif status in ("NO DATA", "UNKNOWN"):
                    content.append(
                        f"*Status: {status} — no status file or incomplete run "
                        f"(`{model}/{test_name}`).*"
                    )
                    content.append("")

                if plots:
                    for plot_file in plots:
                        rel_path = embed_plot_image(plot_file, output_dir)
                        stem_caption = plot_file.stem.replace("_", " ").title()
                        caption = f"{model.upper()} — {stem_caption}"
                        content.append(f"![{caption}]({rel_path})")
                        content.append("")
                elif status != "FAIL":
                    content.append("*No plots available.*")
                    content.append("")

    content.append("---")
    content.append("")
    content.append("*This report was automatically generated by the SEA-Stack comparison test suite.*")

    return '\n'.join(content), tests


def main():
    parser = argparse.ArgumentParser(description='Generate SEA-Stack comparison test report')
    parser.add_argument('--build-dir', default='build', help='Build directory path')
    parser.add_argument('--output-dir', default=None, help='Output directory for report')
    parser.add_argument(
        '--config',
        default=None,
        help='CMake build type. When omitted, inferred from --output-dir if possible; '
        'otherwise legacy Release + bin/results discovery is used.',
    )
    parser.add_argument('--pdf', action='store_true', help='Generate PDF report (requires pandoc + LaTeX)')
    parser.add_argument('--exit-on-failure', action='store_true',
                        help='Exit with code 1 if any comparison test failed')
    parser.add_argument('--quiet', action='store_true',
                        help='Less console output (warnings and errors still print)')

    args = parser.parse_args()

    default_cfg = args.config or "Release"
    if args.output_dir is None:
        output_dir = (
            Path(args.build_dir)
            / "bin"
            / default_cfg
            / "results"
            / "tests"
            / "comparison"
            / "report"
        )
    else:
        output_dir = Path(args.output_dir)

    output_dir.mkdir(parents=True, exist_ok=True)

    discovery_config = args.config or infer_config_from_output_dir(output_dir)

    if not args.quiet:
        print("Generating comparison test report...")
    markdown, test_data = generate_report(
        args.build_dir, output_dir, config=discovery_config
    )

    md_file = output_dir / "comparison_test_report.md"
    with open(md_file, 'w', encoding='utf-8') as f:
        f.write(markdown)
    if not args.quiet:
        print(f"Comparison report: {md_file}")

    if args.pdf:
        pdf_file = output_dir / "comparison_test_report.pdf"
        shared_generate_pdf(
            md_file,
            pdf_file,
            quiet=args.quiet,
            report_title="SEA-Stack Comparison Test Report",
            report_subtitle="Internal method comparisons within SEA-Stack",
        )

    if args.exit_on_failure:
        all_statuses = []
        for model_tests in test_data.values():
            for test_info in model_tests.values():
                all_statuses.append(test_info["status"])

        if "FAIL" in all_statuses:
            failed = []
            for model, model_tests in test_data.items():
                for test_name, test_info in model_tests.items():
                    if test_info["status"] == "FAIL":
                        failed.append(f"{model}/{test_name}")

            print(f"\nFAILED comparison tests ({len(failed)}):")
            for name in failed:
                print(f"  - {name}")
            sys.exit(1)


if __name__ == '__main__':
    main()
