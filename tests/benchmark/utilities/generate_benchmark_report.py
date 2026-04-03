#!/usr/bin/env python3
"""
SEA-Stack Benchmark Report Generator

Scans for benchmark JSON files produced by bench_utils.h, detects
execution overlap, auto-compares against the latest promoted baseline,
and generates a markdown summary report.

Usage:
    python generate_benchmark_report.py \
        --build-dir <dir> \
        --output-dir <dir> \
        --history-dir <dir> \
        [--baseline TAG]
"""

import argparse
import json
import statistics
import sys
from datetime import datetime
from pathlib import Path


def find_benchmark_files(build_dir):
    """Scan for results_bench_*.json files in the build output."""
    build_path = Path(build_dir).resolve()
    results = {}

    for search_dir in [
        build_path / "bin" / "Release" / "results",
        build_path / "bin" / "Release",
        build_path / "bin" / "results",
        build_path / "bin",
    ]:
        if not search_dir.exists():
            continue
        for jf in search_dir.rglob("results_bench_*.json"):
            try:
                with open(jf, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                case_id = data.get("case_id", jf.stem)
                if case_id not in results:
                    results[case_id] = data
            except Exception as e:
                print(f"Warning: could not read {jf}: {e}")

    results = _synthesize_batch_from_conditions(results)
    return results


def _synthesize_batch_from_conditions(cases):
    """If per-condition OSWEC results exist but no full-batch result, synthesize
    an aggregate 'oswec_reg_batch' entry so baseline comparisons still work."""
    if "oswec_reg_batch" in cases:
        return cases

    condition_cases = {k: v for k, v in cases.items()
                       if k.startswith("oswec_reg_condition_")}
    if not condition_cases:
        return cases

    all_trials = []
    for cid in sorted(condition_cases.keys(),
                      key=lambda x: int(x.rsplit("_", 1)[-1])):
        data = condition_cases[cid]
        summary = data.get("summary", {})
        mean_sim = summary.get("sim_wall_s", {}).get("mean")
        if mean_sim is not None:
            trial = {
                "trial": len(all_trials) + 1,
                "setup_wall_s": summary.get("total_wall_s", {}).get("mean", 0)
                                - (mean_sim or 0),
                "sim_wall_s": mean_sim,
                "total_wall_s": summary.get("total_wall_s", {}).get("mean", 0),
                "component_breakdown": {
                    k: v.get("mean", 0) if isinstance(v, dict) else v
                    for k, v in summary.get("component_breakdown", {}).items()
                },
            }
            all_trials.append(trial)

    if not all_trials:
        return cases

    sim_vals = [t["sim_wall_s"] for t in all_trials]
    total_vals = [t["total_wall_s"] for t in all_trials]

    def _stats(vals):
        n = len(vals)
        m = statistics.mean(vals)
        return {
            "mean": m,
            "min": min(vals),
            "max": max(vals),
            "stddev": statistics.stdev(vals) if n > 1 else 0.0,
        }

    first = next(iter(condition_cases.values()))
    settings = first.get("settings", {})
    settings["wave_type"] = "regular_batch_16"
    settings["num_trials"] = len(all_trials)

    batch = {
        "schema_version": 2,
        "case_id": "oswec_reg_batch",
        "started_at": min(c.get("started_at", "") for c in condition_cases.values()),
        "finished_at": max(c.get("finished_at", "") for c in condition_cases.values()),
        "metadata": first.get("metadata", {}),
        "settings": settings,
        "trials": all_trials,
        "summary": {
            "sim_wall_s": _stats(sim_vals),
            "total_wall_s": _stats(total_vals),
        },
        "_synthesized_from": "per_condition",
    }

    cases["oswec_reg_batch"] = batch
    print(f"  Synthesized oswec_reg_batch from {len(all_trials)} per-condition results")
    return cases


def find_latest_history(history_dir):
    """Find the most recent promoted file by promoted_at timestamp."""
    history_path = Path(history_dir)
    if not history_path.exists():
        return None

    latest_data = None
    latest_time = None
    for jf in history_path.glob("*.json"):
        try:
            with open(jf, 'r', encoding='utf-8') as f:
                data = json.load(f)
            ts = data.get("promoted_at", "")
            t = datetime.fromisoformat(ts.replace("Z", "+00:00"))
            if latest_time is None or t > latest_time:
                latest_time = t
                latest_data = data
        except Exception:
            continue
    return latest_data


def load_baseline(history_dir, tag):
    """Load a specific baseline or the latest."""
    if tag:
        path = Path(history_dir) / f"{tag}.json"
        if path.exists():
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        return None
    return find_latest_history(history_dir)


def check_overlap(cases):
    """Check for overlapping execution windows."""
    intervals = []
    for case_id, data in cases.items():
        start = data.get("started_at", "")
        end = data.get("finished_at", "")
        if start and end:
            try:
                t0 = datetime.fromisoformat(start.replace("Z", "+00:00"))
                t1 = datetime.fromisoformat(end.replace("Z", "+00:00"))
                intervals.append((case_id, t0, t1))
            except ValueError:
                pass

    warnings = []
    for i, (id_a, start_a, end_a) in enumerate(intervals):
        for j in range(i + 1, len(intervals)):
            id_b, start_b, end_b = intervals[j]
            if start_a < end_b and start_b < end_a:
                warnings.append(
                    f"{id_a} ({start_a:%H:%M:%S}--{end_a:%H:%M:%S}) "
                    f"overlaps with {id_b} ({start_b:%H:%M:%S}--{end_b:%H:%M:%S})"
                )
    return warnings


def fmt_stat(summary, key, fmt=".2f"):
    """Extract mean from a summary stat block."""
    block = summary.get(key, {})
    if isinstance(block, dict):
        val = block.get("mean")
        if val is not None:
            return f"{val:{fmt}}"
    return "--"


def fmt_stddev(summary, key, fmt=".2f"):
    """Extract stddev from a summary stat block."""
    block = summary.get(key, {})
    if isinstance(block, dict):
        val = block.get("stddev")
        if val is not None:
            return f"{val:{fmt}}"
    return "--"


def generate_report(cases, baseline_data, overlap_warnings):
    """Generate a markdown benchmark report."""
    content = []
    content.append("# SEA-Stack Benchmark Report")
    content.append("")
    content.append(f"*Generated on {datetime.now().strftime('%B %d, %Y at %H:%M:%S')}*")
    content.append("")

    # Overlap warning
    if overlap_warnings:
        content.append("> **WARNING: Overlapping benchmark execution detected.**")
        content.append("> Results may not be comparable to sequential baselines.")
        content.append(">")
        for w in overlap_warnings:
            content.append(f"> - {w}")
        content.append("")

    if not cases:
        content.append("No benchmark results found.")
        content.append("")
        content.append("Run benchmarks with: `ctest -L benchmark -j 1`")
        return '\n'.join(content)

    # Metadata from first case
    first_case = next(iter(cases.values()))
    meta = first_case.get("metadata", {})
    content.append("## Environment")
    content.append("")
    content.append(f"- **Version**: {meta.get('seastack_version', '?')}")
    content.append(f"- **Git commit**: `{meta.get('git_commit', '?')}`"
                   f"{' (dirty)' if meta.get('git_dirty') else ''}")
    content.append(f"- **Host**: {meta.get('hostname', '?')}")
    content.append(f"- **CPU**: {meta.get('cpu', '?')}")
    content.append(f"- **OMP threads**: {meta.get('omp_max_threads', '?')}"
                   f" (of {meta.get('omp_num_procs', '?')} procs)")
    content.append(f"- **Build**: {meta.get('build_type', '?')}"
                   f" ({meta.get('compiler', '?')})")
    content.append(f"- **OS**: {meta.get('os', '?')}")
    content.append("")

    # Results table
    has_baseline = baseline_data is not None
    baseline_tag = baseline_data.get("run_tag", "?") if has_baseline else None
    baseline_results = {}
    if has_baseline:
        baseline_results = {r["case_id"]: r for r in baseline_data.get("results", [])}

    content.append("## Results")
    content.append("")

    if has_baseline:
        content.append(f"Baseline: **{baseline_tag}**")
        content.append("")
        content.append("| Case | Bodies | Trials | Baseline (s) | Current mean (s) | "
                       "Stddev | Delta % | Realtime |")
        content.append("|------|-------:|-------:|-------------:|-----------------:|"
                       "-------:|--------:|---------:|")
    else:
        content.append("*No promoted baseline found for comparison.*")
        content.append("")
        content.append("| Case | Bodies | Trials | Mean sim (s) | Stddev | "
                       "s/step | Realtime |")
        content.append("|------|-------:|-------:|-------------:|-------:|"
                       "-------:|---------:|")

    for case_id in sorted(cases.keys()):
        data = cases[case_id]
        settings = data.get("settings", {})
        summary = data.get("summary", {})
        n_bodies = settings.get("num_bodies", "?")
        n_trials = settings.get("num_trials", "?")
        label = case_id

        cur_mean = fmt_stat(summary, "sim_wall_s")
        cur_sd = fmt_stddev(summary, "sim_wall_s")
        sps = fmt_stat(summary, "s_per_step", ".5f")
        rtf = fmt_stat(summary, "realtime_factor")

        if has_baseline:
            bl = baseline_results.get(case_id, {})
            bl_summary = bl.get("summary", {})
            bl_mean_val = bl_summary.get("sim_wall_s", {}).get("mean")
            bl_mean = f"{bl_mean_val:.2f}" if bl_mean_val is not None else "--"

            cur_mean_val = summary.get("sim_wall_s", {}).get("mean")
            if bl_mean_val and cur_mean_val and bl_mean_val > 0:
                delta_pct = 100.0 * (cur_mean_val - bl_mean_val) / bl_mean_val
                sign = "+" if delta_pct >= 0 else ""
                delta_str = f"{sign}{delta_pct:.1f}%"
            else:
                delta_str = "--"

            content.append(f"| {label} | {n_bodies} | {n_trials} | "
                           f"{bl_mean} | {cur_mean} | {cur_sd} | {delta_str} | {rtf}x |")
        else:
            content.append(f"| {label} | {n_bodies} | {n_trials} | "
                           f"{cur_mean} | {cur_sd} | {sps} | {rtf}x |")

    content.append("")

    # Component breakdown
    content.append("## Component Breakdown (mean seconds)")
    content.append("")
    content.append("| Case | Hydrostatics | Radiation | Excitation | Mooring |")
    content.append("|------|------------:|-----------:|-----------:|--------:|")

    for case_id in sorted(cases.keys()):
        summary = cases[case_id].get("summary", {})
        label = case_id
        cb = summary.get("component_breakdown", {})
        hs = cb.get("hydrostatics_s", {}).get("mean", 0)
        rad = cb.get("radiation_s", {}).get("mean", 0)
        exc = cb.get("excitation_s", {}).get("mean", 0)
        moor = cb.get("mooring_s", {}).get("mean", 0)
        content.append(f"| {label} | {hs:.2f} | {rad:.2f} | {exc:.2f} | {moor:.2f} |")

    content.append("")
    content.append("---")
    content.append("")
    content.append("*Generated by the SEA-Stack benchmark suite.*")

    return '\n'.join(content)


def print_console_table(cases, baseline_data, overlap_warnings):
    """Print a concise comparison table to stdout."""
    if not cases:
        print("\n  No benchmark results found.")
        return

    print("Summary")

    has_baseline = baseline_data is not None
    baseline_tag = baseline_data.get("run_tag", "?") if has_baseline else None
    baseline_results = {}
    if has_baseline:
        baseline_results = {r["case_id"]: r for r in baseline_data.get("results", [])}

    if overlap_warnings:
        print()
        print("  WARNING: Overlapping benchmark execution detected.")
        for w in overlap_warnings:
            print(f"    {w}")
        print("  Results may not be comparable to sequential baselines.")
        print()

    if has_baseline:
        print()
        print(f"  Baseline: {baseline_tag}")
        hdr = (f"  {'Case':<28s}  {'Baseline (s)':>12s}  {'Current (s)':>11s}  "
               f"{'Delta':>8s}  {'Delta %':>8s}  {'Stddev':>8s}")
        print(hdr)
        print("  " + "-" * (len(hdr) - 2))
    else:
        print()
        print("  No promoted baseline found -- showing current run only.")
        hdr = f"  {'Case':<28s}  {'Mean sim (s)':>12s}  {'Stddev':>8s}  {'Realtime':>8s}"
        print(hdr)
        print("  " + "-" * (len(hdr) - 2))

    for case_id in sorted(cases.keys()):
        summary = cases[case_id].get("summary", {})
        label = case_id
        cur_mean_val = summary.get("sim_wall_s", {}).get("mean")
        cur_sd_val = summary.get("sim_wall_s", {}).get("stddev")
        rtf_val = summary.get("realtime_factor", {}).get("mean")

        cur_str = f"{cur_mean_val:11.2f}" if cur_mean_val is not None else f"{'--':>11s}"
        sd_str = f"{cur_sd_val:8.2f}" if cur_sd_val is not None else f"{'--':>8s}"

        if has_baseline:
            bl = baseline_results.get(case_id, {})
            bl_mean_val = bl.get("summary", {}).get("sim_wall_s", {}).get("mean")
            bl_str = f"{bl_mean_val:12.2f}" if bl_mean_val is not None else f"{'--':>12s}"

            if bl_mean_val and cur_mean_val and bl_mean_val > 0:
                delta = cur_mean_val - bl_mean_val
                pct = 100.0 * delta / bl_mean_val
                sign = "+" if delta >= 0 else ""
                delta_str = f"{delta:8.2f}"
                pct_str = f"{sign}{pct:7.1f}%"
            else:
                delta_str = f"{'--':>8s}"
                pct_str = f"{'--':>8s}"

            print(f"  {label:<28s}  {bl_str}  {cur_str}  {delta_str}  {pct_str}  {sd_str}")
        else:
            rtf_str = f"{rtf_val:7.1f}x" if rtf_val is not None else f"{'--':>8s}"
            print(f"  {label:<28s}  {cur_str:>12s}  {sd_str}  {rtf_str}")

    print()


def main():
    parser = argparse.ArgumentParser(description='Generate SEA-Stack benchmark report')
    parser.add_argument('--build-dir', default='build')
    parser.add_argument('--output-dir', default=None)
    parser.add_argument('--history-dir', default=None,
                        help='Path to data/benchmarks/history/ for baseline comparison')
    parser.add_argument('--baseline', default=None,
                        help='Specific baseline tag (default: latest promoted)')

    args = parser.parse_args()

    if args.output_dir is None:
        output_dir = Path(args.build_dir) / "bin" / "Release" / "results" / "tests" / "benchmark" / "report"
    else:
        output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Generating benchmark report...")

    cases = find_benchmark_files(args.build_dir)
    print(f"  Found {len(cases)} benchmark result(s)")

    # Overlap detection
    overlap_warnings = check_overlap(cases) if cases else []
    if overlap_warnings:
        print("  WARNING: overlapping execution detected")

    # Baseline loading (baseline / no-baseline messaging is in Summary table output)
    baseline_data = None
    if args.history_dir:
        baseline_data = load_baseline(args.history_dir, args.baseline)

    # Console comparison table
    print_console_table(cases, baseline_data, overlap_warnings)

    # Markdown report
    markdown = generate_report(cases, baseline_data, overlap_warnings)
    md_file = output_dir / "benchmark_report.md"
    with open(md_file, 'w', encoding='utf-8') as f:
        f.write(markdown)

    # Resolve the results directory (parent of any found JSON)
    results_dir = None
    build_path = Path(args.build_dir).resolve()
    for search_dir in [
        build_path / "bin" / "Release" / "results" / "tests" / "benchmark",
        build_path / "bin" / "results" / "tests" / "benchmark",
    ]:
        if search_dir.exists():
            results_dir = search_dir
            break

    print("Paths")
    print(f"  Report:  {md_file}")
    if results_dir:
        print(f"  Results: {results_dir}")
    if args.history_dir:
        history_dir = Path(args.history_dir).resolve()
        print(f"  History: {history_dir}")
    print()
    if results_dir:
        print("Next step")
        print("  To promote this run to permanent history:")
        print(f"    python tests/benchmark/utilities/promote_benchmark.py "
              f"--run-dir {results_dir} --tag <TAG> --history-dir data/benchmarks/history")
    print()


if __name__ == '__main__':
    main()
