#!/usr/bin/env python3
"""
Promote regression test outputs to reference data.

Copies all results_*.txt files from a completed regression run into
data/reference_data/ and writes a provenance manifest.  Three hard gates
must pass before any files are touched:

  Gate 1 -- Completeness: every expected result file must exist.
  Gate 2 -- Staleness:    every result file must be newer than the build.
  Gate 3 -- Test pass:    no CTest failures (override with --force).

Usage:
    python promote_regression_references.py \\
        --run-dir  build/bin/Release \\
        --build-dir build \\
        --tag v1.0-beta \\
        [--ref-dir data/reference_data] \\
        [--yes] [--force]
"""

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

# ── Expected reference files (41 total) ─────────────────────────────────────
# Each entry: (group, result_stem)  ->  results_{stem}.txt  ->  ss_ref_{stem}.txt

EXPECTED_FILES = {
    "sphere": [
        "sphere_decay",
        "sphere_decay_ss",
        "sphere_irreg_waves",
        "sphere_irreg_waves_ss",
        "sphere_irreg_waves_eta",
        "sphere_irreg_waves_eta_consistency",
    ]
    + [f"sphere_reg_waves_{i}" for i in range(1, 11)],
    "oswec": [
        "oswec_decay",
        "oswec_decay_ss",
        "oswec_irreg_waves",
        "oswec_irreg_waves_ss",
    ]
    + [f"oswec_reg_waves_{i}" for i in range(1, 17)],
    "f3of": [
        "f3of_decay_c1",
        "f3of_decay_c2",
        "f3of_decay_c3",
    ],
    "rm3": [
        "rm3_decay",
        "rm3_reg_waves",
    ],
}

MULTI_CONDITION_STEMS = {
    f"sphere_reg_waves_{i}" for i in range(1, 11)
} | {f"oswec_reg_waves_{i}" for i in range(1, 17)}


def _md5(path: Path) -> str:
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            h.update(chunk)
    return h.hexdigest()


def _count_data_rows(path: Path) -> int:
    """Count lines that start with a number (skipping headers)."""
    count = 0
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue
            try:
                float(stripped.split()[0])
                count += 1
            except (ValueError, IndexError):
                pass
    return count


def _parse_header_columns(path: Path) -> list[str]:
    """Return column names from the first non-empty, non-numeric line."""
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue
            try:
                float(stripped.split()[0])
                return []
            except (ValueError, IndexError):
                if stripped.startswith("-"):
                    continue
                if ":" in stripped and "\t" in stripped:
                    continue
                return [c.strip() for c in stripped.split("  ") if c.strip()]
    return []


def _git_info(repo_root: Path) -> dict:
    """Collect git provenance from the repo."""
    info = {}
    try:
        info["git_commit"] = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo_root, text=True
        ).strip()
    except Exception:
        info["git_commit"] = "unknown"
    try:
        info["git_commit_short"] = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=repo_root, text=True
        ).strip()
    except Exception:
        info["git_commit_short"] = "unknown"
    try:
        dirty = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=repo_root, text=True
        ).strip()
        info["git_dirty"] = len(dirty) > 0
    except Exception:
        info["git_dirty"] = None
    try:
        info["git_describe"] = subprocess.check_output(
            ["git", "describe", "--tags", "--always", "--dirty"],
            cwd=repo_root,
            text=True,
        ).strip()
    except Exception:
        info["git_describe"] = "unknown"
    return info


def _compiler_info(build_dir: Path) -> str:
    """Try to extract compiler ID from CMakeCache.txt."""
    cache = build_dir / "CMakeCache.txt"
    if not cache.exists():
        return "unknown"
    try:
        for line in cache.read_text(encoding="utf-8").splitlines():
            if line.startswith("CMAKE_CXX_COMPILER_ID:"):
                return line.split("=", 1)[1]
    except Exception:
        pass
    return "unknown"


def _build_type_info(build_dir: Path) -> str:
    cache = build_dir / "CMakeCache.txt"
    if not cache.exists():
        return "unknown"
    try:
        for line in cache.read_text(encoding="utf-8").splitlines():
            if line.startswith("CMAKE_BUILD_TYPE:"):
                val = line.split("=", 1)[1]
                if val:
                    return val
    except Exception:
        pass
    return "Release"


def _parse_ctest_failures(build_dir: Path) -> list[str]:
    """Read CTest failure log. Returns list of failing test names."""
    fail_log = build_dir / "Testing" / "Temporary" / "LastTestsFailed.log"
    if not fail_log.exists():
        return []
    failures = []
    for line in fail_log.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        # Format: "23:test_name"
        parts = line.split(":", 1)
        if len(parts) == 2:
            failures.append(parts[1])
        else:
            failures.append(line)
    return failures


def confirm(prompt: str, default_no: bool = True) -> bool:
    suffix = " [y/N] " if default_no else " [Y/n] "
    try:
        answer = input(prompt + suffix).strip().lower()
    except EOFError:
        return not default_no
    if default_no:
        return answer in ("y", "yes")
    return answer not in ("n", "no")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Promote regression results to reference data"
    )
    parser.add_argument(
        "--run-dir",
        required=True,
        help="Build-tree run directory (e.g. build/bin/Release)",
    )
    parser.add_argument(
        "--build-dir",
        required=True,
        help="CMake build directory (e.g. build) for staleness + CTest logs",
    )
    parser.add_argument(
        "--tag",
        required=True,
        help="Version tag for this baseline (e.g. v1.0-beta)",
    )
    parser.add_argument(
        "--ref-dir",
        default=None,
        help="Reference data directory (default: data/reference_data relative to repo root)",
    )
    parser.add_argument(
        "--yes", "-y", action="store_true", help="Skip interactive confirmation"
    )
    parser.add_argument(
        "--force",
        "-f",
        action="store_true",
        help="Allow promotion despite test failures (records them in manifest)",
    )
    args = parser.parse_args()

    # Resolve paths
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent.parent.parent  # tests/regression/utilities -> repo
    run_dir = Path(args.run_dir).resolve()
    build_dir = Path(args.build_dir).resolve()
    ref_dir = Path(args.ref_dir).resolve() if args.ref_dir else repo_root / "data" / "reference_data"

    results_base = run_dir / "results" / "tests" / "regression"

    print()
    print("  Promote Regression References")
    print("  " + "=" * 50)
    print(f"  Tag:             {args.tag}")
    print(f"  Source:          {results_base}")
    print(f"  Destination:     {ref_dir}")
    print(f"  Build dir:       {build_dir}")
    print()

    # ── Gate 1: Completeness ────────────────────────────────────────────────
    print("  Gate 1 -- Completeness")

    all_expected = []
    for group, stems in EXPECTED_FILES.items():
        for stem in stems:
            result_file = results_base / group / f"results_{stem}.txt"
            ref_file = ref_dir / group / f"ss_ref_{stem}.txt"
            all_expected.append((group, stem, result_file, ref_file))

    missing = []
    found = []
    for group, stem, result_file, ref_file in all_expected:
        if result_file.exists():
            found.append((group, stem, result_file, ref_file))
        else:
            missing.append((group, stem, result_file))

    if missing:
        print(f"  FAILED: {len(missing)} of {len(all_expected)} result files missing:\n")
        for group, stem, path in missing:
            print(f"    {group}/{stem}: {path}")

        missing_multi = [s for _, s, _ in missing if s in MULTI_CONDITION_STEMS]
        if missing_multi:
            print()
            print(
                "  HINT: Multi-condition files missing. Did you configure with"
            )
            print("        -DSEASTACK_CORE_REGRESSION_SUBSET=OFF and run the full suite?")
        print()
        print("  Promotion aborted.")
        return 1

    print(f"    {len(found)}/{len(all_expected)} result files found  [PASS]")
    print()

    # ── Gate 2: Staleness ───────────────────────────────────────────────────
    print("  Gate 2 -- Staleness")

    sentinel = build_dir / "CMakeCache.txt"
    if not sentinel.exists():
        print(f"    WARNING: {sentinel} not found; skipping staleness check")
        build_mtime = 0.0
    else:
        build_mtime = sentinel.stat().st_mtime

    stale = []
    for group, stem, result_file, ref_file in found:
        if result_file.stat().st_mtime < build_mtime:
            stale.append((group, stem, result_file))

    if stale:
        print(
            f"  FAILED: {len(stale)} result file(s) older than {sentinel.name}:\n"
        )
        for group, stem, path in stale:
            print(f"    {group}/{stem}")
        print()
        print("  Results appear stale. Re-run the regression suite, then try again.")
        print("  Promotion aborted.")
        return 1

    print(f"    All {len(found)} result files newer than build  [PASS]")
    print()

    # ── Gate 3: Test-pass check ─────────────────────────────────────────────
    print("  Gate 3 -- Test pass")

    failures = _parse_ctest_failures(build_dir)
    regression_failures = [f for f in failures if "regr" in f or "regression" in f.lower()]

    if regression_failures:
        print(f"    {len(regression_failures)} regression test failure(s) detected:\n")
        for name in regression_failures:
            print(f"      - {name}")
        print()

        if not args.force:
            print(
                "  Promotion blocked: re-run after fixing, or use --force to acknowledge"
            )
            print("  known failures (they will be recorded in the manifest).")
            print("  Promotion aborted.")
            return 1

        print("    --force specified: proceeding despite failures  [FORCED]")
    else:
        print("    No regression failures in CTest log  [PASS]")
    print()

    # ── Summary table ───────────────────────────────────────────────────────
    print("  Files to promote:")
    print(f"    {'Group':<10s}  {'Count':>5s}")
    print(f"    {'-'*10}  {'-'*5}")
    for group in sorted(EXPECTED_FILES.keys()):
        count = len(EXPECTED_FILES[group])
        print(f"    {group:<10s}  {count:>5d}")
    print(f"    {'-'*10}  {'-'*5}")
    print(f"    {'TOTAL':<10s}  {len(all_expected):>5d}")
    print()

    if regression_failures and args.force:
        print("  WARNING: The following failures will be recorded in the manifest:")
        for name in regression_failures:
            print(f"    - {name}")
        print()

    # ── Confirmation ────────────────────────────────────────────────────────
    if not args.yes:
        if not confirm("  Overwrite reference data with these results?"):
            print("  Aborted.")
            return 1
        print()

    # ── Copy files ──────────────────────────────────────────────────────────
    print("  Copying result files to reference data...")

    file_manifest = {}
    for group, stem, result_file, ref_file in found:
        ref_file.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(result_file, ref_file)

        rel_path = f"{group}/ss_ref_{stem}.txt"
        file_manifest[rel_path] = {
            "source_result": f"results_{stem}.txt",
            "rows": _count_data_rows(ref_file),
            "columns": _parse_header_columns(ref_file),
            "md5": _md5(ref_file),
        }

    print(f"    Copied {len(found)} files")

    # ── Write manifest ──────────────────────────────────────────────────────
    print("  Writing manifest.json...")

    git = _git_info(repo_root)

    manifest = {
        "schema_version": "1.0",
        "tag": args.tag,
        "git_commit": git["git_commit"],
        "git_commit_short": git["git_commit_short"],
        "git_describe": git["git_describe"],
        "git_dirty": git["git_dirty"],
        "generated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "platform": f"{platform.system()} {platform.release()} ({platform.machine()})",
        "compiler": _compiler_info(build_dir),
        "build_type": _build_type_info(build_dir),
        "generator": "promote_regression_references.py",
        "total_files": len(found),
        "files": file_manifest,
    }

    if regression_failures and args.force:
        manifest["known_failures"] = regression_failures

    manifest_path = ref_dir / "manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")

    print(f"    Written: {manifest_path}")
    print()
    print(f"  Promoted {len(found)} reference files as '{args.tag}'")
    print("  Remember to review and commit the updated data/reference_data/ directory.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
