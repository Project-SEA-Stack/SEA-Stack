#!/usr/bin/env python3
"""
Compare current benchmark run against a promoted baseline.

Usage:
    python compare_benchmarks.py \
        --history-dir data/benchmarks/history \
        --current build/bin/Release/results/tests/benchmark \
        --baseline v0.1.0-baseline
"""

import argparse
import json
import statistics
import sys
from datetime import datetime
from pathlib import Path


def find_benchmark_jsons(run_dir):
    """Discover all results_bench_*.json files."""
    run_path = Path(run_dir).resolve()
    results = {}
    for jf in run_path.rglob("results_bench_*.json"):
        try:
            with open(jf, 'r', encoding='utf-8') as f:
                data = json.load(f)
            case_id = data.get("case_id", jf.stem)
            results[case_id] = data
        except Exception:
            continue

    # Synthesize batch from per-condition results if needed
    if "oswec_reg_batch" not in results:
        condition_cases = {k: v for k, v in results.items()
                           if k.startswith("oswec_reg_condition_")}
        if condition_cases:
            sim_vals = []
            for cid in sorted(condition_cases.keys(),
                               key=lambda x: int(x.rsplit("_", 1)[-1])):
                mean = condition_cases[cid].get("summary", {}).get(
                    "sim_wall_s", {}).get("mean")
                if mean is not None:
                    sim_vals.append(mean)
            if sim_vals:
                first = next(iter(condition_cases.values()))
                results["oswec_reg_batch"] = {
                    "summary": {
                        "sim_wall_s": {
                            "mean": statistics.mean(sim_vals),
                            "stddev": statistics.stdev(sim_vals) if len(sim_vals) > 1 else 0.0,
                        }
                    },
                    "metadata": first.get("metadata", {}),
                }
    return results


def load_history_file(history_dir, tag):
    """Load a specific history file by tag."""
    path = Path(history_dir) / f"{tag}.json"
    if not path.exists():
        return None
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


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


def print_comparison(current_cases, baseline_data):
    """Print a formatted comparison table."""
    baseline_tag = baseline_data.get("run_tag", "?")
    baseline_results = {r["case_id"]: r for r in baseline_data.get("results", [])}

    all_cases = sorted(set(list(current_cases.keys()) + list(baseline_results.keys())))

    header = (f"{'Case':<30s} | {'Baseline mean (s)':>18s} | {'Current mean (s)':>17s} | "
              f"{'Delta':>8s} | {'Delta %':>8s} | {'Cur stddev':>10s}")
    print(f"\nBaseline: {baseline_tag}")
    print(header)
    print("-" * len(header))

    for case_id in all_cases:
        bl = baseline_results.get(case_id, {})
        bl_mean = bl.get("summary", {}).get("sim_wall_s", {}).get("mean")

        cur = current_cases.get(case_id, {})
        cur_summary = cur.get("summary", {}).get("sim_wall_s", {})
        cur_mean = cur_summary.get("mean")
        cur_sd = cur_summary.get("stddev")

        bl_str = f"{bl_mean:18.2f}" if bl_mean is not None else f"{'--':>18s}"
        cur_str = f"{cur_mean:17.2f}" if cur_mean is not None else f"{'--':>17s}"
        sd_str = f"{cur_sd:10.2f}" if cur_sd is not None else f"{'--':>10s}"

        if bl_mean is not None and cur_mean is not None and bl_mean > 0:
            delta = cur_mean - bl_mean
            pct = 100.0 * delta / bl_mean
            sign = "+" if delta >= 0 else ""
            delta_str = f"{delta:8.2f}"
            pct_str = f"{sign}{pct:7.1f}%"
        else:
            delta_str = f"{'--':>8s}"
            pct_str = f"{'--':>8s}"

        print(f"{case_id:<30s} | {bl_str} | {cur_str} | {delta_str} | {pct_str} | {sd_str}")


def main():
    parser = argparse.ArgumentParser(description='Compare benchmarks against a promoted baseline')
    parser.add_argument('--history-dir', required=True, help='Path to data/benchmarks/history/')
    parser.add_argument('--current', required=True, help='Build-tree benchmark results directory')
    parser.add_argument('--baseline', default=None, help='Baseline tag (default: latest promoted)')

    args = parser.parse_args()

    current_cases = find_benchmark_jsons(args.current)
    if not current_cases:
        print("ERROR: no current benchmark results found.")
        return 1

    print(f"Current run: {len(current_cases)} case(s)")

    # Load baseline
    if args.baseline:
        baseline_data = load_history_file(args.history_dir, args.baseline)
        if baseline_data is None:
            print(f"ERROR: baseline '{args.baseline}' not found in {args.history_dir}")
            return 1
    else:
        baseline_data = find_latest_history(args.history_dir)
        if baseline_data is None:
            print("No promoted baselines found. Showing current run only:\n")
            for case_id, data in sorted(current_cases.items()):
                mean = data.get("summary", {}).get("sim_wall_s", {}).get("mean")
                sd = data.get("summary", {}).get("sim_wall_s", {}).get("stddev")
                if mean is not None:
                    print(f"  {case_id:<30s}  mean={mean:.2f}s  stddev={sd:.2f}s")
            return 0

    print_comparison(current_cases, baseline_data)
    return 0


if __name__ == '__main__':
    sys.exit(main())
