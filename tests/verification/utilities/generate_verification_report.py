#!/usr/bin/env python3
"""
SEA-Stack Verification Test Report Generator

Generates a verification report (markdown + optional PDF) from .status.json
files written by verification comparison scripts.  This is the verification
counterpart to tests/regression/utilities/generate_regression_report.py.

Usage:
    python generate_verification_report.py --build-dir <dir> --output-dir <dir> [--pdf] [--exit-on-failure]
"""

import argparse
import sys
from collections import defaultdict
from pathlib import Path
from typing import Optional

# Import shared report generation infrastructure
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "utilities"))
from report_generator import (
    find_status_files,
    find_plot_files as shared_find_plot_files,
    generate_report_header,
    embed_plot_image,
    generate_pdf as shared_generate_pdf,
)

# CMake TEST_GROUP names / status prefixes (single source for ordering and scoping)
VERIFICATION_TEST_GROUPS = (
    "sphere_decay_multicode",
    "sphere_rao_sweep",
    "oswec_decay_wecsim",
    "oswec_rao_sweep",
    "f3of_decay_multicode",
    "rm3_mooring",
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


def _find_status_files(build_dir, config=None):
    """Scan for verification .status.json files under results/tests/verification only."""
    status_records = find_status_files(
        build_dir, suite_filter="verification", config=config
    )
    results = {}
    for record in status_records:
        path = record.get("path")
        if path is not None and "verification" not in path.parts:
            continue
        status_data = record.get("status_data", {})
        test_name = status_data.get("test_name", record.get("test_name"))
        if test_name:
            results[test_name] = status_data
    return results


def _find_plot_files(build_dir, config=None):
    """Find verification comparison plots (excluding basic overlay debug artifacts)."""
    plot_records = shared_find_plot_files(
        build_dir,
        suite_filter="verification",
        pattern="*.png",
        exclude_patterns=["*_overlay.png"],
        config=config,
    )
    plots = defaultdict(list)
    allowed_groups = frozenset(VERIFICATION_TEST_GROUPS)
    for record in plot_records:
        plot_path = record.get("path")
        if not plot_path:
            continue
        if "verification" not in plot_path.parts:
            continue
        group = plot_path.parent.parent.name
        if group not in allowed_groups:
            continue
        plots[group].append(plot_path)
    for group in plots:
        plots[group].sort(key=lambda p: p.name)
    return plots


def generate_report(build_dir, output_dir, config=None, html_styling=False):
    """Generate the verification report markdown.

    Returns:
        (markdown_text, status_data)
    """
    status_data = _find_status_files(build_dir, config=config)
    plots = _find_plot_files(build_dir, config=config)

    verification_prefixes = list(VERIFICATION_TEST_GROUPS)

    ver_status = {}
    for name, payload in status_data.items():
        for prefix in verification_prefixes:
            if name.startswith(prefix):
                ver_status[name] = payload
                break

    all_statuses = [p.get("status", "UNKNOWN") for p in ver_status.values()]
    if not all_statuses:
        outcome_line = "No verification data was found; run the verification suite first."
    elif "FAIL" in all_statuses:
        outcome_line = (
            "**Failed** — one or more cases exceeded cross-code failure tolerances."
        )
    else:
        outcome_line = (
            "**Passed** — all cases are within published verification tolerances "
            "(including advisory margin checks)."
        )

    content = []

    header_lines = generate_report_header(
        "Verification",
        "Cross-code verification of hydrodynamic simulation capabilities",
    )
    content.extend(header_lines)

    content.append("## Summary")
    content.append("")
    if ver_status:
        content.append(f"- **Cases included:** {len(ver_status)}")
    content.append(f"- **Outcome:** {outcome_line}")
    content.append("")

    allowed_plot_groups = frozenset(VERIFICATION_TEST_GROUPS)
    ver_plot_groups = {g: ps for g, ps in plots.items() if g in allowed_plot_groups}

    if ver_plot_groups:
        content.append("## Results")
        content.append("")

        def _plot_sort_key(item):
            group = item[0]
            for i, prefix in enumerate(verification_prefixes):
                if group == prefix or group.startswith(prefix):
                    return (i, group)
            return (len(verification_prefixes), group)

        for group, plot_files in sorted(ver_plot_groups.items(), key=_plot_sort_key):
            # Keep the section title with its first figure in PDF output.
            content.append("\\clearpage")
            content.append("")
            display_name = group.replace("_", " ").title().replace("Wecsim", "WEC-Sim")
            content.append(f"### {display_name}")
            content.append("")
            content.append("\\nopagebreak")
            content.append("")
            for pf in plot_files:
                rel = embed_plot_image(pf, output_dir)
                caption = (
                    pf.stem.replace("_", " ")
                    .replace("comparison", "")
                    .replace("verification", "")
                    .replace("overlay", "")
                    .strip()
                )
                content.append(f"![{caption}]({rel})")
                content.append("")

    content.append("---")
    content.append("")
    content.append(
        "*This report was automatically generated by the SEA-Stack verification test suite.*"
    )

    return "\n".join(content), ver_status


def main():
    parser = argparse.ArgumentParser(description="Generate SEA-Stack verification report")
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument(
        "--config",
        default=None,
        help="CMake build type (Release, Debug, ...). When omitted, inferred from "
        "--output-dir if it contains bin/<Config>/; otherwise legacy Release + "
        "bin/results discovery is used.",
    )
    parser.add_argument("--pdf", action="store_true")
    parser.add_argument("--exit-on-failure", action="store_true")
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Less console output (warnings and errors still print)",
    )

    args = parser.parse_args()

    default_cfg = args.config or "Release"
    if args.output_dir is None:
        output_dir = (
            Path(args.build_dir)
            / "bin"
            / default_cfg
            / "results"
            / "tests"
            / "verification"
            / "report"
        )
    else:
        output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    config = args.config or infer_config_from_output_dir(output_dir)

    if not args.quiet:
        print("Generating verification report...")
    markdown, ver_status = generate_report(args.build_dir, output_dir, config=config)

    md_file = output_dir / "verification_report.md"
    with open(md_file, "w", encoding="utf-8") as f:
        f.write(markdown)
    if not args.quiet:
        print(f"Verification report: {md_file}")

    if args.pdf:
        pdf_file = output_dir / "verification_report.pdf"
        shared_generate_pdf(
            md_file,
            pdf_file,
            quiet=args.quiet,
            report_title="SEA-Stack Verification Test Report",
            report_subtitle=(
                "Cross-code verification of hydrodynamic simulation capabilities"
            ),
        )

    if args.exit_on_failure:
        all_statuses = [p.get("status", "UNKNOWN") for p in ver_status.values()]
        if "FAIL" in all_statuses:
            failed = [n for n, p in ver_status.items() if p.get("status") == "FAIL"]
            print(f"\nFAILED verification tests ({len(failed)}):")
            for name in failed:
                print(f"  - {name}")
            sys.exit(1)


if __name__ == "__main__":
    main()
