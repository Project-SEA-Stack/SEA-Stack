#!/usr/bin/env python3
"""Fail if a macOS SEA-Stack install/package tree depends on anything outside itself.

Usage:
  python3 scripts/check_macho_portability.py <install_prefix_or_unpacked_zip_root>

For every Mach-O file under the root (bin/run_seastack, lib/*.dylib, ...), except the
SDK-only executables that the runtime ZIP never contains:
  - each dylib dependency is a system library (/usr/lib, /System/Library) or an
    @rpath / @loader_path / @executable_path reference that resolves to a file
    inside the root (@rpath via the image's own LC_RPATHs and the executable's);
  - no LC_RPATH is absolute or resolves outside the root;
  - every dylib install name starts with @rpath/;
  - `codesign --verify --strict` passes.
If the Vulkan loader is bundled, the MoltenVK ICD manifest must be present and its
library_path must resolve to a Mach-O file inside the root.

Also prints the minimum macOS version (LC_BUILD_VERSION minos) of the files.
Called by scripts/verify_release_install_prefix.py on macOS; run it by hand on an
unpacked release ZIP as part of the clean-machine acceptance test.
Exit code 0 = pass, 1 = fail, 2 = usage error. Requires otool and codesign.
"""
from __future__ import annotations

import collections
import json
import os
import re
import subprocess
import sys
from pathlib import Path

MACHO_MAGICS = {
    b"\xfe\xed\xfa\xce", b"\xce\xfa\xed\xfe",  # 32-bit
    b"\xfe\xed\xfa\xcf", b"\xcf\xfa\xed\xfe",  # 64-bit
    b"\xca\xfe\xba\xbe", b"\xbe\xba\xfe\xca",  # universal
}
SYSTEM_PREFIXES = ("/usr/lib/", "/System/Library/")
DYLIB_LOAD_CMDS = {"LC_LOAD_DYLIB", "LC_LOAD_WEAK_DYLIB", "LC_REEXPORT_DYLIB",
                   "LC_LAZY_LOAD_DYLIB", "LC_LOAD_UPWARD_DYLIB"}
EXECUTABLE = Path("bin") / "run_seastack"
# Installed by `cmake --install` into the SDK component only; never in the runtime ZIP
# (same list as scripts/check_package_contents.py).
SDK_ONLY_EXECUTABLES = {"standalone_controller", "standalone_hydro", "demo_sphere_decay"}


def is_macho(path: Path) -> bool:
    try:
        with path.open("rb") as f:
            return f.read(4) in MACHO_MAGICS
    except OSError:
        return False


def load_commands(path: Path) -> tuple[str, list[str], list[str], str]:
    """Return (install name, dylib deps, rpaths, minos) parsed from `otool -l`."""
    out = subprocess.run(["otool", "-l", str(path)], check=True,
                         capture_output=True, text=True).stdout
    cmd, ident, deps, rpaths, minos = "", "", [], [], ""
    for line in out.splitlines():
        m = re.match(r"^\s+cmd (LC_\w+)$", line)
        if m:
            cmd = m.group(1)
            continue
        m = re.match(r"^\s+(name|path) (.+) \(offset \d+\)$", line)
        if m:
            if cmd == "LC_ID_DYLIB":
                ident = m.group(2)
            elif cmd in DYLIB_LOAD_CMDS and m.group(2) not in deps:
                deps.append(m.group(2))
            elif cmd == "LC_RPATH":
                rpaths.append(m.group(2))
            continue
        m = re.match(r"^\s+minos (\S+)$", line)
        if m and cmd == "LC_BUILD_VERSION" and not minos:
            minos = m.group(1)
    return ident, deps, rpaths, minos


def inside(root: Path, path: Path) -> bool:
    try:
        real = Path(os.path.realpath(path))
    except OSError:
        return False
    return real.exists() and (real == root or root in real.parents)


def expand(token_path: str, owner: Path, exe: Path) -> Path | None:
    if token_path.startswith("@loader_path/"):
        return owner.parent / token_path[len("@loader_path/"):]
    if token_path.startswith("@executable_path/"):
        return exe.parent / token_path[len("@executable_path/"):]
    if token_path.startswith("/"):
        return Path(token_path)
    return None


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

    files = sorted(p for p in root.rglob("*")
                   if p.is_file() and not p.is_symlink() and is_macho(p))
    sdk_only = [p for p in files if p.parent == root / "bin" and p.name in SDK_ONLY_EXECUTABLES]
    for p in sdk_only:
        print(f"skipped (SDK-only, not in runtime ZIP): {p.relative_to(root)}")
    files = [p for p in files if p not in sdk_only]
    exe_rpaths = load_commands(exe)[2]
    errors: list[str] = []
    minos_count: collections.Counter[str] = collections.Counter()

    for f in files:
        rel = f.relative_to(root)
        ident, deps, rpaths, minos = load_commands(f)
        minos_count[minos or "unknown"] += 1

        if f != exe and f.suffix == ".dylib" and not ident.startswith("@rpath/"):
            errors.append(f"{rel}: install name is not @rpath: {ident}")

        for rp in rpaths:
            if rp.startswith("/"):
                errors.append(f"{rel}: absolute LC_RPATH {rp}")
            else:
                target = expand(rp, f, exe)
                if target is None or not inside(root, target):
                    errors.append(f"{rel}: LC_RPATH {rp} does not resolve inside the package")

        search = [(rp, f) for rp in rpaths] + [(rp, exe) for rp in exe_rpaths]
        for dep in deps:
            if dep.startswith(SYSTEM_PREFIXES):
                continue
            if dep.startswith("@rpath/"):
                tail = dep[len("@rpath/"):]
                resolved = False
                for rp, owner in search:
                    base = expand(rp, owner, exe)
                    if base is not None and inside(root, base / tail):
                        resolved = True
                        break
                if not resolved:
                    errors.append(f"{rel}: {dep} does not resolve inside the package")
            elif dep.startswith(("@loader_path/", "@executable_path/")):
                target = expand(dep, f, exe)
                if target is None or not inside(root, target):
                    errors.append(f"{rel}: {dep} does not resolve inside the package")
            else:
                errors.append(f"{rel}: non-system dependency outside the package: {dep}")

        sig = subprocess.run(["codesign", "--verify", "--strict", str(f)],
                             capture_output=True, text=True)
        if sig.returncode != 0:
            errors.append(f"{rel}: codesign --verify --strict failed: {sig.stderr.strip()}")

    if list((root / "lib").glob("libvulkan*.dylib")):
        icd = root / "share" / "vulkan" / "icd.d" / "MoltenVK_icd.json"
        if not icd.is_file():
            errors.append("Vulkan loader bundled but share/vulkan/icd.d/MoltenVK_icd.json missing")
        else:
            lib_path = json.loads(icd.read_text()).get("ICD", {}).get("library_path", "")
            target = icd.parent / lib_path
            if not lib_path or not inside(root, target) or not is_macho(target):
                errors.append(f"MoltenVK_icd.json library_path does not resolve inside the package: {lib_path}")

    print(f"Mach-O files: {len(files)}  root: {root}")
    print("minimum macOS (minos): " + ", ".join(f"{k} x{v}" for k, v in sorted(minos_count.items())))
    if errors:
        print(f"[FAIL] {len(errors)} Mach-O portability problem(s):")
        for e in errors:
            print(f"  {e}")
        return 1
    print("[OK] All Mach-O dependencies and rpaths resolve inside the package; signatures valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
