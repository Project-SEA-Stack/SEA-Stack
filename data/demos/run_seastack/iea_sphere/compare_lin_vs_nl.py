#!/usr/bin/env python3
"""
Cross-case comparison: linear vs nonlinear hydrostatics on IEA sphere decay.

For each amplitude (1 m, 5 m), loads the output H5 from the corresponding
linear and nonlinear case directories and plots heave displacement from
equilibrium (z - z_eq) as an overlay with a difference subplot.

Heave convention: displacement from BEM equilibrium, z_eq = -2.0 m for the
IEA sphere. This matches the C++ comparison test convention.

Usage:
    python compare_lin_vs_nl.py <demos_dir>

    where <demos_dir> is the path to the iea_sphere demo directory,
    e.g. data/demos/run_seastack/iea_sphere
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    import h5py
except ImportError:
    h5py = None

Z_EQ = -2.0


AMPLITUDE_CASES = [
    {
        "tag": "1m",
        "linear_dir": "decay",
        "nonlinear_dir": "decay_nl_1m",
    },
    {
        "tag": "5m",
        "linear_dir": "decay_lin_5m",
        "nonlinear_dir": "decay_nl_5m",
    },
]


def _find_output_h5(case_dir: Path) -> Path:
    """Locate the simulation output H5 in a case's outputs/ directory."""
    outputs = case_dir / "outputs"
    for p in outputs.glob("results.*.h5"):
        return p
    raise FileNotFoundError(f"No results.*.h5 found in {outputs}")


def _load_heave(h5_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load time and heave displacement from equilibrium from an output H5."""
    if h5py is None:
        raise RuntimeError("h5py is required for compare_lin_vs_nl.py")
    with h5py.File(h5_path, "r") as f:
        for tkey in ["/results/time/time", "/results/time", "/time"]:
            if tkey in f:
                t = np.asarray(f[tkey][:], dtype=float).reshape(-1)
                break
        else:
            raise KeyError(f"time vector not found in {h5_path}")
        for pkey in [
            "/results/model/bodies/body1/position",
            "/results/bodies/body1/position",
        ]:
            if pkey in f:
                arr = np.asarray(f[pkey][:])
                if arr.ndim == 2 and arr.shape[1] >= 3:
                    z_raw = arr[:, 2]
                elif arr.ndim == 1:
                    z_raw = arr
                else:
                    raise ValueError(f"Unexpected shape {arr.shape} for {pkey}")
                heave = z_raw - Z_EQ
                return t, heave
        raise KeyError(f"body1 position not found in {h5_path}")


def _compute_norms(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    diff = a - b
    l2 = float(np.sqrt(np.mean(diff**2)))
    linf = float(np.max(np.abs(diff)))
    return l2, linf


def _plot_comparison(
    time_lin: np.ndarray,
    heave_lin: np.ndarray,
    time_nl: np.ndarray,
    heave_nl: np.ndarray,
    amp_tag: str,
    output_path: Path,
) -> None:
    if not HAS_MATPLOTLIB:
        return

    min_len = min(len(time_lin), len(time_nl))
    t = time_lin[:min_len]
    h_lin = heave_lin[:min_len]
    h_nl = heave_nl[:min_len]

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(12, 8), sharex=True,
        gridspec_kw={"height_ratios": [3, 1]},
    )

    ax1.plot(t, h_lin, label="Linear", linewidth=1.0)
    ax1.plot(t, h_nl, label="Nonlinear", linewidth=1.0, linestyle="--")
    ax1.set_ylabel("Heave displacement from equilibrium (m)")
    ax1.set_title(f"Linear vs Nonlinear Hydrostatics: {amp_tag} drop")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    diff = h_nl - h_lin
    ax2.plot(t, diff, color="red", linewidth=0.8)
    ax2.set_ylabel("Difference (m)")
    ax2.set_xlabel("Time (s)")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(output_path), dpi=150)
    plt.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare linear vs nonlinear hydrostatics YAML sphere decay cases",
    )
    parser.add_argument(
        "demos_dir",
        help="Path to iea_sphere demo directory (e.g. data/demos/run_seastack/iea_sphere)",
    )
    args = parser.parse_args()

    base = Path(args.demos_dir).resolve()
    plots_dir = base / "outputs" / "plots"

    any_error = False
    for case in AMPLITUDE_CASES:
        tag = case["tag"]
        lin_dir = base / case["linear_dir"]
        nl_dir = base / case["nonlinear_dir"]

        try:
            lin_h5 = _find_output_h5(lin_dir)
            nl_h5 = _find_output_h5(nl_dir)
        except FileNotFoundError as e:
            print(f"  SKIP {tag}: {e}")
            continue

        try:
            t_lin, h_lin = _load_heave(lin_h5)
            t_nl, h_nl = _load_heave(nl_h5)
        except Exception as e:
            print(f"  ERROR {tag}: {e}")
            any_error = True
            continue

        min_len = min(len(t_lin), len(t_nl))
        l2, linf = _compute_norms(h_lin[:min_len], h_nl[:min_len])
        print(f"  {tag}: L2={l2:.4e}, Linf={linf:.4e}")

        plot_path = plots_dir / f"lin_vs_nl_{tag}_heave.png"
        _plot_comparison(t_lin, h_lin, t_nl, h_nl, tag, plot_path)
        print(f"    Plot: {plot_path}")

    return 1 if any_error else 0


if __name__ == "__main__":
    sys.exit(main())
