#!/usr/bin/env python3
"""
SEA-Stack Benchmark Batch Aggregation

Scans for per-condition benchmark JSON files (e.g., oswec_reg_condition_*.json)
and merges them into a single batch summary JSON file compatible with existing
benchmark report/comparison tooling.

Usage:
    python aggregate_batch_results.py \
        --build-dir <dir> \
        [--case-prefix <prefix>]
"""

import argparse
import json
import statistics
import sys
from datetime import datetime
from pathlib import Path


def find_condition_files(build_dir, case_prefix="oswec_reg_condition"):
    """Scan for per-condition JSON files."""
    build_path = Path(build_dir).resolve()
    condition_files = []

    for search_dir in [
        build_path / "bin" / "Release" / "results" / "tests" / "benchmark" / "oswec",
        build_path / "bin" / "Release" / "results" / "benchmark" / "oswec",
        build_path / "bin" / "Release" / "results",
        build_path / "bin" / "Release",
        build_path / "bin" / "results",
        build_path / "bin",
    ]:
        if not search_dir.exists():
            continue
        pattern = f"results_bench_{case_prefix}_*.json"
        for jf in search_dir.glob(pattern):
            condition_files.append(jf)

    # Sort by condition number
    condition_files.sort(key=lambda p: int(p.stem.rsplit("_", 1)[-1]))
    return condition_files


def aggregate_conditions(condition_files, batch_case_id="oswec_reg_batch"):
    """Aggregate per-condition results into a batch summary."""
    if not condition_files:
        print(f"Warning: No condition files found for {batch_case_id}")
        return None

    all_trials = []
    condition_data = []

    for jf in condition_files:
        try:
            with open(jf, 'r', encoding='utf-8') as f:
                data = json.load(f)
            condition_data.append(data)

            # Extract trial data from condition summary
            summary = data.get("summary", {})
            mean_sim = summary.get("sim_wall_s", {}).get("mean")
            if mean_sim is not None:
                trial = {
                    "trial": len(all_trials) + 1,
                    "setup_wall_s": summary.get("setup_wall_s", {}).get("mean", 0),
                    "sim_wall_s": mean_sim,
                    "total_wall_s": summary.get("total_wall_s", {}).get("mean", 0),
                    "component_breakdown": {
                        k: v.get("mean", 0) if isinstance(v, dict) else v
                        for k, v in summary.get("component_breakdown", {}).items()
                    },
                }
                all_trials.append(trial)
        except Exception as e:
            print(f"Warning: could not read {jf}: {e}")

    if not all_trials:
        print(f"Warning: No valid trial data found in condition files")
        return None

    # Compute aggregate statistics
    sim_vals = [t["sim_wall_s"] for t in all_trials]
    total_vals = [t["total_wall_s"] for t in all_trials]
    setup_vals = [t["setup_wall_s"] for t in all_trials]

    def _stats(vals):
        n = len(vals)
        if n == 0:
            return {"mean": 0.0, "min": 0.0, "max": 0.0, "stddev": 0.0}
        m = statistics.mean(vals)
        return {
            "mean": m,
            "min": min(vals),
            "max": max(vals),
            "stddev": statistics.stdev(vals) if n > 1 else 0.0,
        }

    # Use first condition's metadata and settings as template
    first = condition_data[0]
    settings = first.get("settings", {}).copy()
    settings["wave_type"] = "regular_batch_16"
    settings["num_trials"] = len(all_trials)
    settings["warmup"] = False

    # Aggregate component breakdown
    component_breakdown = {}
    for trial in all_trials:
        for k, v in trial.get("component_breakdown", {}).items():
            if k not in component_breakdown:
                component_breakdown[k] = []
            component_breakdown[k].append(v)

    component_summary = {
        k: _stats(vals) for k, vals in component_breakdown.items()
    }

    batch = {
        "schema_version": 2,
        "case_id": batch_case_id,
        "started_at": min(c.get("started_at", "") for c in condition_data),
        "finished_at": max(c.get("finished_at", "") for c in condition_data),
        "metadata": first.get("metadata", {}),
        "settings": settings,
        "trials": all_trials,
        "summary": {
            "setup_wall_s": _stats(setup_vals),
            "sim_wall_s": _stats(sim_vals),
            "total_wall_s": _stats(total_vals),
            "component_breakdown": component_summary,
        },
    }

    return batch


def main():
    parser = argparse.ArgumentParser(
        description="Aggregate per-condition benchmark results into batch summary"
    )
    parser.add_argument(
        "--build-dir",
        required=True,
        help="Build directory containing benchmark results",
    )
    parser.add_argument(
        "--case-prefix",
        default="oswec_reg_condition",
        help="Case ID prefix for condition files (default: oswec_reg_condition)",
    )
    parser.add_argument(
        "--batch-case-id",
        default="oswec_reg_batch",
        help="Case ID for aggregated batch result (default: oswec_reg_batch)",
    )
    parser.add_argument(
        "--output-dir",
        help="Output directory for batch JSON (default: same as condition files)",
    )

    args = parser.parse_args()

    condition_files = find_condition_files(args.build_dir, args.case_prefix)
    if not condition_files:
        print(f"No condition files found for prefix '{args.case_prefix}'")
        return 1

    print(f"Found {len(condition_files)} condition files")
    batch = aggregate_conditions(condition_files, args.batch_case_id)
    if not batch:
        print("Failed to aggregate batch results")
        return 1

    # Determine output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        # Use same directory as first condition file
        output_dir = condition_files[0].parent

    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / f"results_bench_{args.batch_case_id}.json"

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(batch, f, indent=2)

    print(f"Wrote aggregated batch result to {output_file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
