#!/usr/bin/env python3
"""Fail if staged install prefix is missing files required for release ZIP QA.

Usage:
  python scripts/verify_release_install_prefix.py <install_prefix>

Called from scripts/unix/build.sh and scripts/windows/build.ps1 after cmake --install
and before or after cpack (prefix is the same tree that is archived).

If macOS packaged runs show NaNs while Windows passes, compare otool -L on run_seastack,
bundled lib/*.dylib, OpenMP, and BLAS linkage against a local non-packaged Release build.
"""
from __future__ import annotations

import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: verify_release_install_prefix.py <install_prefix>", file=sys.stderr)
        return 2
    root = Path(sys.argv[1]).resolve()
    if not root.is_dir():
        print(f"Not a directory: {root}", file=sys.stderr)
        return 2

    required = [
        root / "demos" / "rm3" / "mooring" / "eta_rm3_mooring.txt",
        root / "tests" / "utilities" / "plot_helpers.py",
        root / "tests" / "compare_template.py",
        root / "demos" / "f3of" / "decay_dt3" / "f3of_decay_dt3.setup.yaml",
        # External PTO demos (RM3 TSDA + OSWEC RSDA) + IPC helper + plot workflow.
        root / "python" / "seastack_external.py",
        root / "demos" / "rm3" / "external_pto" / "linear_damper_pto.py",
        root / "demos" / "rm3" / "external_pto" / "rm3_external_pto.external_pto.yaml",
        root / "demos" / "rm3" / "external_pto_adaptive" / "adaptive_damping_pto.py",
        root / "demos" / "rm3" / "external_pto_hydraulic" / "hydraulic_accumulator_pto.py",
        root / "demos" / "oswec" / "external_pto" / "linear_damper_pto.py",
        root / "demos" / "oswec" / "external_pto" / "oswec_external_pto.external_pto.yaml",
        root / "demos" / "oswec" / "external_pto_adaptive" / "adaptive_damping_pto.py",
        root / "demos" / "oswec" / "external_pto_hydraulic" / "hydraulic_accumulator_pto.py",
        root / "examples" / "external_pto" / "plot_verification.py",
        root / "examples" / "external_pto" / "run_visual_verification.py",
        root / "examples" / "external_pto" / "README.md",
    ]
    missing = [p for p in required if not p.is_file()]
    if missing:
        print("[FAIL] Release install prefix missing required files:", file=sys.stderr)
        for p in missing:
            print(f"  {p}", file=sys.stderr)
        print(
            "\nHint: package builds must enable -DSEASTACK_ENABLE_EXTERNAL=ON "
            "(scripts/windows/build.ps1 -Package and scripts/unix/build.sh --package "
            "do this automatically).",
            file=sys.stderr,
        )
        return 1
    print(f"[OK] Release install prefix checks passed ({root})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
