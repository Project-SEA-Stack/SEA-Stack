#!/usr/bin/env python3
"""Emit shell export statements for scripts/unix/build.sh (Chrono_DIR, Eigen, HDF5, flags).

Parses build-config.json and optionally chrono-config.cmake / ChronoConfig.cmake
for the same heuristics as scripts/windows/build.ps1.
"""
from __future__ import annotations

import json
import re
import shlex
import sys
from pathlib import Path


def _emit(key: str, value: str | None) -> None:
    if value is None:
        print(f"export {key}=")
    else:
        print(f"export {key}={shlex.quote(value)}")


def _emit_int(key: str, value: int) -> None:
    print(f"export {key}={value}")


def _chrono_pkg_path(chrono_dir: Path) -> Path | None:
    lower = chrono_dir / "chrono-config.cmake"
    pascal = chrono_dir / "ChronoConfig.cmake"
    if lower.is_file():
        return lower
    if pascal.is_file():
        return pascal
    return None


def main() -> int:
    if len(sys.argv) < 4:
        print(
            "usage: seastack_build_prepare.py <repo_root> <config_path> <use_chrono:0|1> [doctor]",
            file=sys.stderr,
        )
        return 2

    repo = Path(sys.argv[1]).resolve()
    config_path = Path(sys.argv[2]).resolve()
    use_chrono = sys.argv[3] == "1"
    doctor = len(sys.argv) > 4 and sys.argv[4] == "doctor"

    _emit("SEASTACK_REPO_ROOT", str(repo))

    if not config_path.is_file():
        if use_chrono and doctor:
            print(f"[doctor] Config file not found: {config_path}", file=sys.stderr)
        if use_chrono and not doctor:
            print(f"Config file not found: {config_path}", file=sys.stderr)
            print("Copy build-config.example.json to build-config.json and set ChronoDir.", file=sys.stderr)
            return 1
        _emit("SEASTACK_CONFIG_OK", "0")
        _emit("SEASTACK_CHRONO_DIR", None)
        _emit("SEASTACK_PYTHON_ROOT", None)
        _emit("SEASTACK_HDF5_DIR", None)
        _emit("SEASTACK_CONFIG_GENERATOR", None)
        _emit_int("SEASTACK_HAS_VSG_IN_CHRONO", 0)
        _emit_int("SEASTACK_HAS_HDF5_IN_CHRONO", 0)
        _emit("SEASTACK_CHRONO_PKG_READ", "0")
        return 0

    _emit("SEASTACK_CONFIG_OK", "1")
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        print(f"Invalid JSON in {config_path}: {e}", file=sys.stderr)
        return 1

    gen = raw.get("Generator")
    _emit("SEASTACK_CONFIG_GENERATOR", gen if isinstance(gen, str) and gen.strip() else None)

    py_root = raw.get("PythonRoot")
    _emit(
        "SEASTACK_PYTHON_ROOT",
        py_root if isinstance(py_root, str) and py_root.strip() else None,
    )

    hdf5_from_config = raw.get("HDF5Dir")
    hdf5_from_config = hdf5_from_config if isinstance(hdf5_from_config, str) and hdf5_from_config.strip() else None

    chrono_content: str | None = None
    eigen3_include: str | None = None
    hdf5_dir: str | None = hdf5_from_config
    has_vsg = 0
    has_hdf5 = 0

    if use_chrono:
        chrono_dir_s = raw.get("ChronoDir")
        if not isinstance(chrono_dir_s, str) or not chrono_dir_s.strip():
            print("ChronoDir missing or empty in build-config.json", file=sys.stderr)
            if doctor:
                _emit("SEASTACK_CHRONO_DIR", None)
                _emit("SEASTACK_EIGEN3_INCLUDE_DIR", None)
                _emit("SEASTACK_HDF5_DIR", hdf5_dir)
                _emit_int("SEASTACK_HAS_VSG_IN_CHRONO", 0)
                _emit_int("SEASTACK_HAS_HDF5_IN_CHRONO", 0)
                _emit("SEASTACK_CHRONO_PKG_READ", "0")
                return 0
            return 1
        chrono_dir = Path(chrono_dir_s).expanduser()
        if not chrono_dir.is_dir():
            print(f"ChronoDir does not exist: {chrono_dir}", file=sys.stderr)
            if doctor:
                _emit("SEASTACK_CHRONO_DIR", str(chrono_dir))
                _emit("SEASTACK_EIGEN3_INCLUDE_DIR", None)
                _emit("SEASTACK_HDF5_DIR", hdf5_dir)
                _emit_int("SEASTACK_HAS_VSG_IN_CHRONO", 0)
                _emit_int("SEASTACK_HAS_HDF5_IN_CHRONO", 0)
                _emit("SEASTACK_CHRONO_PKG_READ", "0")
                return 0
            return 1
        chrono_resolved = str(chrono_dir.resolve())
        _emit("SEASTACK_CHRONO_DIR", chrono_resolved)

        pkg = _chrono_pkg_path(chrono_dir)
        if pkg and pkg.is_file():
            chrono_content = pkg.read_text(encoding="utf-8", errors="replace")
            if re.search(r"Chrono_VSG_AVAILABLE\s+ON", chrono_content):
                has_vsg = 1
            if re.search(r"CHRONO_HDF5_AVAILABLE\s+(ON|TRUE|1)", chrono_content):
                has_hdf5 = 1

            m = re.search(r'Eigen3_DIR\s+"(\S+)"', chrono_content)
            if m and "NOTFOUND" not in m.group(1):
                eigen3_root = Path(m.group(1)).parent.parent
                candidate = eigen3_root / "include" / "eigen3"
                if (candidate / "Eigen").is_dir():
                    eigen3_include = str(candidate.resolve())

            if not eigen3_include:
                m = re.search(r'EIGEN3_INCLUDE_DIR\s+"(\S+)"', chrono_content)
                if m and "NOTFOUND" not in m.group(1):
                    candidate = Path(m.group(1))
                    if (candidate / "Eigen").is_dir():
                        eigen3_include = str(candidate.resolve())

            if not hdf5_dir:
                m = re.search(r'HDF5_DIR\s+"(\S+)"', chrono_content)
                if m and "NOTFOUND" not in m.group(1):
                    candidate = Path(m.group(1))
                    if (candidate / "hdf5-config.cmake").is_file():
                        hdf5_dir = str(candidate.resolve())
    else:
        _emit("SEASTACK_CHRONO_DIR", None)

    _emit("SEASTACK_EIGEN3_INCLUDE_DIR", eigen3_include)

    if hdf5_from_config and Path(hdf5_from_config).is_dir():
        hdf5_dir = str(Path(hdf5_from_config).resolve())

    _emit("SEASTACK_HDF5_DIR", hdf5_dir)
    _emit_int("SEASTACK_HAS_VSG_IN_CHRONO", has_vsg)
    _emit_int("SEASTACK_HAS_HDF5_IN_CHRONO", has_hdf5)
    if chrono_content is not None:
        # Pass a marker only; build.sh does not need full content (no DLL copy on Unix).
        _emit("SEASTACK_CHRONO_PKG_READ", "1")
    else:
        _emit("SEASTACK_CHRONO_PKG_READ", "0")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
