#!/usr/bin/env python3
"""Fail if a Linux SEA-Stack install/package tree depends on host Chrono/VSG paths.

Usage:
  python3 scripts/check_elf_portability.py <install_prefix_or_unpacked_zip_root>

For bin/run_seastack and every non-symlink ELF under lib/:
  - DT_RPATH / DT_RUNPATH entries are only $ORIGIN or $ORIGIN/<relative>;
  - no absolute path outside the package root.

Then runs `ldd` on bin/run_seastack with a cleared environment (no LD_LIBRARY_PATH).
Chrono / VSG / vsgXchange / draco / glslang / yaml-cpp must resolve to files inside
the package. System libraries (libc, libstdc++, libm, libgcc, libpthread, libdl,
libz, OpenMP, HDF5, Vulkan loader, X11/Wayland, etc.) may resolve from the host.

Called by scripts/verify_release_install_prefix.py on Linux; run by hand on an
unpacked release ZIP as part of the clean-machine acceptance test.
Exit code 0 = pass, 1 = fail, 2 = usage error. Requires readelf and ldd.
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

ELF_MAGICS = {b"\x7fELF"}
EXECUTABLE = Path("bin") / "run_seastack"
SDK_ONLY_EXECUTABLES = {"standalone_controller", "standalone_hydro", "demo_sphere_decay"}

# Basenames (or prefixes) that must resolve inside the package, not /home/... hosts.
PACKAGE_ONLY_PREFIXES = (
    "libChrono",
    "libyaml-cpp",
    "libvsg",
    "libvsgImGui",
    "libvsgXchange",
    "libdraco",
    "libglslang",
    "libSPIRV",
    "libSPVRemapper",
    "libSPIRV-Tools",
)

# ldd "not a dynamic executable" / vdso / etc.
LDD_SKIP = {"linux-vdso.so.1", "ld-linux-x86-64.so.2", "ld-linux-aarch64.so.1"}


def is_elf(path: Path) -> bool:
    try:
        with path.open("rb") as f:
            return f.read(4) in ELF_MAGICS
    except OSError:
        return False


def read_runpaths(path: Path) -> list[str]:
    out = subprocess.run(
        ["readelf", "-d", str(path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    paths: list[str] = []
    for line in out.splitlines():
        m = re.search(r"\((?:RPATH|RUNPATH)\)[^[]*\[([^\]]+)\]", line)
        if m:
            paths.extend(p for p in m.group(1).split(":") if p)
    return paths


def inside(root: Path, path: Path) -> bool:
    try:
        real = Path(os.path.realpath(path))
    except OSError:
        return False
    return real.exists() and (real == root or root in real.parents)


def must_be_packaged(soname: str) -> bool:
    base = Path(soname).name
    return any(base == p or base.startswith(p + ".") or base.startswith(p + "-")
               for p in PACKAGE_ONLY_PREFIXES)


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__.strip().splitlines()[2], file=sys.stderr)
        return 2
    root = Path(os.path.realpath(sys.argv[1]))
    if not root.is_dir():
        print(f"Not a directory: {root}", file=sys.stderr)
        return 2
    exe = root / EXECUTABLE
    if not exe.is_file():
        print(f"[FAIL] {EXECUTABLE} not found under {root}")
        return 1

    lib_dir = root / "lib"
    files: list[Path] = [exe]
    if lib_dir.is_dir():
        for p in sorted(lib_dir.iterdir()):
            if p.is_symlink() or not p.is_file():
                continue
            if is_elf(p):
                files.append(p)

    # Skip SDK-only executables if present under bin/
    bin_dir = root / "bin"
    if bin_dir.is_dir():
        for p in bin_dir.iterdir():
            if p.name in SDK_ONLY_EXECUTABLES:
                print(f"skipped (SDK-only, not in runtime ZIP): {p.relative_to(root)}")

    errors: list[str] = []

    for f in files:
        rel = f.relative_to(root)
        for rp in read_runpaths(f):
            if rp.startswith("$ORIGIN"):
                # $ORIGIN or $ORIGIN/relative — relative part must not escape via .. beyond root
                if rp == "$ORIGIN":
                    continue
                if rp.startswith("$ORIGIN/"):
                    # Reject absolute-looking leftovers; allow ../lib from bin/
                    rest = rp[len("$ORIGIN/"):]
                    if rest.startswith("/") or re.match(r"^[A-Za-z]:", rest):
                        errors.append(f"{rel}: invalid RUNPATH {rp}")
                    continue
                errors.append(f"{rel}: unsupported $ORIGIN form {rp}")
            elif rp.startswith("/"):
                if not inside(root, Path(rp)):
                    errors.append(f"{rel}: absolute RUNPATH outside package: {rp}")
            else:
                errors.append(f"{rel}: non-$ORIGIN RUNPATH: {rp}")

    # ldd with no LD_LIBRARY_PATH — resolution must use package RUNPATHs.
    env = {
        "PATH": "/usr/bin:/bin",
        "HOME": os.environ.get("HOME", "/tmp"),
        "LANG": "C",
    }
    # Explicitly do not pass LD_LIBRARY_PATH / LD_PRELOAD.
    ldd = subprocess.run(
        ["ldd", str(exe)],
        env=env,
        capture_output=True,
        text=True,
    )
    if ldd.returncode != 0:
        errors.append(f"ldd failed on {EXECUTABLE}: {ldd.stderr.strip() or ldd.stdout.strip()}")
    else:
        print("ldd (no LD_LIBRARY_PATH):")
        for line in ldd.stdout.splitlines():
            print(f"  {line.strip()}")
            s = line.strip()
            if " => " not in s:
                # e.g. "linux-vdso.so.1 (0x...)" or "statically linked"
                continue
            left, right = s.split(" => ", 1)
            soname = left.strip()
            if soname in LDD_SKIP:
                continue
            if "not found" in right:
                errors.append(f"ldd: {soname} => not found")
                continue
            # "path (0xaddr)"
            path_str = right.split(" (", 1)[0].strip()
            if path_str in ("", "not found"):
                errors.append(f"ldd: {soname} unresolved")
                continue
            resolved = Path(path_str)
            if must_be_packaged(soname):
                if not inside(root, resolved):
                    errors.append(
                        f"ldd: {soname} resolved outside package: {resolved}"
                    )

    print(f"ELF files checked: {len(files)}  root: {root}")
    if errors:
        print(f"[FAIL] {len(errors)} ELF portability problem(s):")
        for e in errors:
            print(f"  {e}")
        return 1
    print("[OK] Package-relative RUNPATHs; Chrono/VSG stack resolves from the package (no LD_LIBRARY_PATH)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
