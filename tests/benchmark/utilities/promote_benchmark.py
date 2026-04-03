#!/usr/bin/env python3
"""
Promote a benchmark run into the persistent history.

Reads all per-case JSON results from a build-tree benchmark run,
validates completeness, checks for overlapping execution, and writes
a single promoted history file.

Usage:
    python promote_benchmark.py \
        --run-dir build/bin/Release/results/tests/benchmark \
        --tag v0.1.0-baseline \
        --history-dir data/benchmarks/history
"""

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path


EXPECTED_CASES = [
    "sphere_decay_conv",
    "sphere_decay_ss",
    "sphere_irreg_conv",
    "sphere_irreg_ss",
    "f3of_decay_c3",
    "rm3_mooring_irreg",
    "oswec_reg_batch",
    "5sa_spreading",
]

OPTIONAL_CASES = {"rm3_mooring_irreg", "5sa_spreading"}


def find_benchmark_jsons(run_dir):
    """Discover all results_bench_*.json files in the run directory tree."""
    run_path = Path(run_dir).resolve()
    results = {}
    for jf in run_path.rglob("results_bench_*.json"):
        try:
            with open(jf, 'r', encoding='utf-8') as f:
                data = json.load(f)
            case_id = data.get("case_id", jf.stem)
            results[case_id] = data
        except Exception as e:
            print(f"  WARNING: could not read {jf}: {e}")
    return results


def check_overlap(cases):
    """Check for overlapping execution windows. Returns list of warnings."""
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
        for j, (id_b, start_b, end_b) in enumerate(intervals):
            if j <= i:
                continue
            if start_a < end_b and start_b < end_a:
                warnings.append(
                    f"  {id_a} ({start_a:%H:%M:%S}--{end_a:%H:%M:%S}) "
                    f"overlaps with {id_b} ({start_b:%H:%M:%S}--{end_b:%H:%M:%S})"
                )
    return warnings


def find_latest_history(history_dir):
    """Find the most recent promoted file by promoted_at timestamp."""
    history_path = Path(history_dir)
    if not history_path.exists():
        return None, None

    latest_file = None
    latest_time = None
    for jf in history_path.glob("*.json"):
        try:
            with open(jf, 'r', encoding='utf-8') as f:
                data = json.load(f)
            ts = data.get("promoted_at", "")
            t = datetime.fromisoformat(ts.replace("Z", "+00:00"))
            if latest_time is None or t > latest_time:
                latest_time = t
                latest_file = (jf, data)
        except Exception:
            continue
    return latest_file


def print_delta(current_cases, baseline_data):
    """Print a comparison summary."""
    if not baseline_data:
        return
    baseline_tag = baseline_data.get("run_tag", "?")
    baseline_results = {r["case_id"]: r for r in baseline_data.get("results", [])}

    print(f"\n  vs. {baseline_tag}:")
    for case_id, data in sorted(current_cases.items()):
        current_mean = data.get("summary", {}).get("sim_wall_s", {}).get("mean")
        if current_mean is None:
            continue
        bl = baseline_results.get(case_id, {})
        bl_mean = bl.get("summary", {}).get("sim_wall_s", {}).get("mean")
        if bl_mean is None or bl_mean == 0:
            print(f"    {case_id:30s}  {current_mean:8.2f}s  (no baseline)")
            continue
        delta = current_mean - bl_mean
        pct = 100.0 * delta / bl_mean
        sign = "+" if delta >= 0 else ""
        print(f"    {case_id:30s}  {current_mean:8.2f}s  ({sign}{pct:.1f}%)")


def confirm(prompt, default_no=True):
    """Ask user for y/N confirmation. Returns True if confirmed."""
    suffix = " [y/N] " if default_no else " [Y/n] "
    try:
        answer = input(prompt + suffix).strip().lower()
    except EOFError:
        return not default_no
    if default_no:
        return answer in ("y", "yes")
    return answer not in ("n", "no")


def main():
    parser = argparse.ArgumentParser(description='Promote a benchmark run to history')
    parser.add_argument('--run-dir', required=True, help='Build-tree benchmark results directory')
    parser.add_argument('--tag', required=True, help='Tag for this promoted run (e.g. v0.1.0-baseline)')
    parser.add_argument('--history-dir', required=True, help='Path to data/benchmarks/history/')
    parser.add_argument('--yes', '-y', action='store_true', help='Skip confirmation prompt')
    parser.add_argument('--force', '-f', action='store_true', help='Overwrite existing history file without asking')
    args = parser.parse_args()

    run_path = Path(args.run_dir).resolve()
    history_path = Path(args.history_dir).resolve()
    out_file = history_path / f"{args.tag}.json"

    print()
    print("  Promote Benchmark Run")
    print("  " + "=" * 40)
    print(f"  Tag:         {args.tag}")
    print(f"  Source:      {run_path}")
    print(f"  Destination: {out_file}")
    print()

    cases = find_benchmark_jsons(args.run_dir)
    if not cases:
        print("  ERROR: no benchmark results found in the source directory.")
        print(f"         Searched: {run_path}")
        return 1

    print(f"  Discovered {len(cases)} benchmark case(s):")
    for case_id in sorted(cases.keys()):
        mean = cases[case_id].get("summary", {}).get("sim_wall_s", {}).get("mean")
        mean_str = f"{mean:.2f}s" if mean is not None else "?"
        print(f"    {case_id:<28s}  mean sim = {mean_str}")

    # Validate completeness
    missing = []
    for expected in EXPECTED_CASES:
        if expected not in cases:
            if expected in OPTIONAL_CASES:
                print(f"\n  NOTE: optional case '{expected}' not found (MoorDyn not enabled?)")
            else:
                missing.append(expected)
    if missing:
        print(f"\n  WARNING: missing required cases: {', '.join(missing)}")

    # Check overlap
    overlap_warnings = check_overlap(cases)
    if overlap_warnings:
        print("\n  WARNING: Overlapping benchmark execution detected:")
        for w in overlap_warnings:
            print(w)
        print("  Results may not be comparable to sequential baselines.")

    # Check for existing file
    if out_file.exists():
        print(f"\n  WARNING: {out_file} already exists!")
        if not args.force:
            if not confirm("  Overwrite?"):
                print("  Aborted.")
                return 1

    # Confirmation
    if not args.yes:
        print()
        if not confirm("  Continue with promotion?"):
            print("  Aborted.")
            return 1

    # Build metadata from first case
    first_case = next(iter(cases.values()))
    metadata = first_case.get("metadata", {})

    # Assemble promoted file
    promoted = {
        "schema_version": 2,
        "run_tag": args.tag,
        "promoted_at": datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
        "metadata": metadata,
        "results": []
    }

    for case_id in sorted(cases.keys()):
        data = cases[case_id]
        entry = {
            "case_id": case_id,
            "settings": data.get("settings", {}),
            "summary": data.get("summary", {}),
            "trials": data.get("trials", [])
        }
        promoted["results"].append(entry)

    # Write
    history_path.mkdir(parents=True, exist_ok=True)
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(promoted, f, indent=2)
        f.write('\n')

    print(f"\n  Promoted to: {out_file}")
    print("  Remember to review and commit this file.")

    # Delta vs latest existing
    latest = find_latest_history(str(history_path))
    if latest:
        latest_file, latest_data = latest
        if latest_data.get("run_tag") != args.tag:
            print_delta(cases, latest_data)

    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
