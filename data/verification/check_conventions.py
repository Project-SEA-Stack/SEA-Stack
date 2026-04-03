#!/usr/bin/env python3
"""
Validate all normalized verification data files against CONVENTIONS.md.

Walks every `normalized/` directory under data/verification/, parses the
# metadata header from each .txt file, and reports any violations:
  - Missing required metadata fields
  - Non-canonical units
  - Missing coordinate_frame / sign_convention in manifest.json

Usage:
    python data/verification/check_conventions.py

Exit code: 0 if all checks pass, 1 if any issues found.
"""

import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

REQUIRED_FIELDS = {'format', 'source', 'model', 'quantity', 'units', 'columns'}

CANONICAL_UNITS = {
    's', 'm', 'rad', 'N', 'm/m', 'rad/m', 'rad/s', '-', 'kg',
}

MANIFEST_REQUIRED_DATASET_KEYS = {'coordinate_frame', 'sign_convention'}


def parse_metadata(path):
    meta = {}
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            if not line.startswith('#'):
                break
            if ':' in line:
                key_val = line.lstrip('#').strip()
                key, _, val = key_val.partition(':')
                meta[key.strip()] = val.strip()
    return meta


def check_txt_file(path):
    issues = []
    meta = parse_metadata(path)

    if not meta:
        issues.append(f"  No # metadata header found")
        return issues

    missing = REQUIRED_FIELDS - set(meta.keys())
    if missing:
        issues.append(f"  Missing metadata fields: {', '.join(sorted(missing))}")

    units_str = meta.get('units', '')
    if units_str:
        units = [u.strip() for u in units_str.split(',')]
        bad_units = [u for u in units if u not in CANONICAL_UNITS]
        if bad_units:
            issues.append(f"  Non-canonical units: {', '.join(bad_units)}")

    return issues


def check_manifest(manifest_path):
    issues = []
    try:
        with open(manifest_path, 'r', encoding='utf-8') as f:
            manifest = json.load(f)
    except Exception as e:
        issues.append(f"  Cannot parse manifest: {e}")
        return issues

    for i, ds in enumerate(manifest.get('normalized_datasets', [])):
        ds_file = ds.get('file', f'dataset[{i}]')
        missing = MANIFEST_REQUIRED_DATASET_KEYS - set(ds.keys())
        if missing:
            issues.append(
                f"  Dataset '{ds_file}': missing {', '.join(sorted(missing))}"
            )

    return issues


def main():
    total_issues = 0
    total_files = 0

    print("=== SEA-Stack Verification Data Convention Check ===\n")

    for case_dir in sorted(HERE.iterdir()):
        if not case_dir.is_dir():
            continue
        norm_dir = case_dir / 'normalized'
        if not norm_dir.is_dir():
            continue

        # Check normalized .txt files
        for txt_file in sorted(norm_dir.rglob('*.txt')):
            total_files += 1
            rel = txt_file.relative_to(HERE)
            issues = check_txt_file(txt_file)
            if issues:
                print(f"FAIL  {rel}")
                for iss in issues:
                    print(iss)
                total_issues += len(issues)
            else:
                print(f"OK    {rel}")

        # Check manifest.json
        manifest = case_dir / 'manifest.json'
        if manifest.exists():
            rel = manifest.relative_to(HERE)
            issues = check_manifest(manifest)
            if issues:
                print(f"FAIL  {rel}")
                for iss in issues:
                    print(iss)
                total_issues += len(issues)
            else:
                print(f"OK    {rel}")
        else:
            print(f"WARN  {case_dir.name}/manifest.json: not found")
            total_issues += 1

    print(f"\n{'='*50}")
    print(f"Files checked: {total_files}")
    if total_issues == 0:
        print("Result: ALL CHECKS PASSED")
    else:
        print(f"Result: {total_issues} issue(s) found")
    sys.exit(0 if total_issues == 0 else 1)


if __name__ == '__main__':
    main()
