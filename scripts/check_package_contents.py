#!/usr/bin/env python3
"""Check a SEA-Stack runtime ZIP against the expected v1.0 package contents.

Usage:
  python scripts/check_package_contents.py <SEAStack-*.zip> --platform {win64,linux-x64,darwin}

Checks:
  - demos/ contains exactly the expected case directories (allowlist below);
  - bin/ contains run_seastack (run_seastack.exe on Windows), and SDK-only
    executables are absent (on Windows, run_seastack.exe is the only .exe);
  - nothing forbidden is packaged: Wigley demos, run outputs, logs, meshes,
    caches, release notes, the downstream-consumer test.

Complements scripts/verify_release_install_prefix.py, which checks that
required files exist in the install prefix before archiving.
Exit code 0 = pass, 1 = fail, 2 = usage error. Standard library only.
"""
from __future__ import annotations

import argparse
import re
import sys
import zipfile
from pathlib import Path

EXPECTED_DEMO_TOP_FILES = {"README.md", "h5outputsReader.py"}
EXPECTED_DEMO_CASES = {
    "5sa": {"assets", "bimodal", "irregular_waves", "mooring", "power_matrix",
            "regular_waves", "spreading"},
    "f3of": {"assets", "decay_dt3", "irregular_waves"},
    "iea_sphere": {"assets", "decay", "decay_lin_5m", "decay_nl_1m", "decay_nl_5m",
                   "decay_ss", "irregular_waves_ss"},
    "oswec": {"assets", "decay", "divergence_limits", "external_pto",
              "external_pto_adaptive", "external_pto_hydraulic", "irregular_waves",
              "regular_waves", "translucent_hull"},
    "rm3": {"assets", "bimodal_sea", "decay", "decay_nl", "external_pto",
            "external_pto_adaptive", "external_pto_hydraulic", "irregular_waves", "mooring"},
    "trimaran": {"assets", "rigid"},
}

# Built by `cmake --install` into the SDK component only; never in the runtime ZIP.
SDK_ONLY_EXECUTABLES = {"standalone_controller", "standalone_hydro", "demo_sphere_decay"}

FORBIDDEN = [
    re.compile(r"(?i)wigley"),
    re.compile(r"/outputs[^/]*/"),
    re.compile(r"\.ssph$"),
    re.compile(r"\.log$"),
    re.compile(r"\.nc$"),
    re.compile(r"/meshes/"),
    re.compile(r"/__pycache__/"),
    re.compile(r"RELEASE_NOTES"),
    re.compile(r"(^|/)tests/consumer/"),
]


def strip_root(names: list[str]) -> list[str]:
    """Drop a single common top-level directory, if the ZIP has one."""
    tops = {n.split("/", 1)[0] for n in names}
    if len(tops) == 1 and all("/" in n for n in names):
        return [n.split("/", 1)[1] for n in names if n.split("/", 1)[1]]
    return names


def check_demos(names: list[str]) -> bool:
    ok = True
    top_files: set[str] = set()
    cases: dict[str, set[str]] = {}
    for n in names:
        if not n.startswith("demos/"):
            continue
        parts = [p for p in n[len("demos/"):].split("/") if p]
        if len(parts) == 1 and not n.endswith("/"):
            top_files.add(parts[0])
        elif len(parts) >= 2:
            if len(parts) == 2 and not n.endswith("/"):
                continue  # file directly under demos/<model>/ (e.g. README), not a case dir
            cases.setdefault(parts[0], set()).add(parts[1])

    print(f"demos/ top-level files: {sorted(top_files)}")
    for model in sorted(cases):
        print(f"  {model}: {sorted(cases[model])}")
    if top_files != EXPECTED_DEMO_TOP_FILES:
        ok = False
        print(f"FAIL demos/ top-level files: missing={sorted(EXPECTED_DEMO_TOP_FILES - top_files)} "
              f"extra={sorted(top_files - EXPECTED_DEMO_TOP_FILES)}")
    for model in sorted(set(cases) | set(EXPECTED_DEMO_CASES)):
        got, want = cases.get(model, set()), EXPECTED_DEMO_CASES.get(model, set())
        if got != want:
            ok = False
            print(f"FAIL demos/{model}: missing={sorted(want - got)} extra={sorted(got - want)}")
    return ok


def check_executables(names: list[str], platform: str) -> bool:
    ok = True
    exe_name = "run_seastack.exe" if platform == "win64" else "run_seastack"
    if f"bin/{exe_name}" not in names:
        ok = False
        print(f"FAIL bin/{exe_name} missing")
    sdk_hits = [n for n in names if Path(n).stem in SDK_ONLY_EXECUTABLES and not n.endswith("/")]
    if sdk_hits:
        ok = False
        print(f"FAIL SDK-only executables packaged: {sdk_hits}")
    if platform == "win64":
        exes = sorted(n for n in names if n.lower().endswith(".exe"))
        print(f"executables: {exes}")
        if exes != ["bin/run_seastack.exe"]:
            ok = False
            print("FAIL expected bin/run_seastack.exe to be the only .exe")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("zip", type=Path)
    parser.add_argument("--platform", required=True, choices=["win64", "linux-x64", "darwin"])
    args = parser.parse_args()
    if not args.zip.is_file():
        print(f"Not a file: {args.zip}", file=sys.stderr)
        return 2

    with zipfile.ZipFile(args.zip) as zf:
        names = strip_root([n.replace("\\", "/") for n in zf.namelist()])
    print(f"ZIP: {args.zip.name}  size: {args.zip.stat().st_size / 1e6:.1f} MB  "
          f"entries: {len(names)}  platform: {args.platform}")

    ok = check_demos(names)
    ok = check_executables(names, args.platform) and ok

    bad = [n for n in names if any(p.search("/" + n) for p in FORBIDDEN)]
    if bad:
        ok = False
        print(f"FAIL forbidden entries ({len(bad)}), first 10:")
        for n in bad[:10]:
            print(f"  {n}")

    print("PASS" if ok else "RESULT: FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
