#!/usr/bin/env python3
"""
Validate the proposed power-matrix summary HDF5 and campaign YAML schemas.

Creates synthetic data matching the schema defined in the WEC power-matrix
workflow plan (Section E), writes it to HDF5/CSV, and prints the structure
for human review (or inspection with HDFView / h5py).

Also writes an example campaign YAML to verify the proposed YAML schema.

Usage:
    python validate_campaign_schema.py [--output-dir DIR]

Requires: h5py, numpy, pyyaml (all standard scientific-Python packages).
"""

import argparse
import os
import sys

import numpy as np

try:
    import h5py
except ImportError:
    sys.exit("h5py is required. Install with: pip install h5py")

try:
    import yaml
except ImportError:
    yaml = None  # YAML validation is optional


def build_synthetic_cells(hs_values, tp_values, seeds, steepness_max=0.07):
    """Enumerate a Cartesian grid of (Hs, Tp, seed) with steepness filtering."""
    g = 9.81
    cells = []
    for hs in hs_values:
        for tp in tp_values:
            steepness = hs / (g * tp**2 / (2 * np.pi))
            for seed in seeds:
                cell = {
                    "hs": hs,
                    "tp": tp,
                    "heading_deg": 0.0,
                    "seed": seed,
                }
                if steepness > steepness_max:
                    cell["status"] = 1  # skipped_invalid
                    cell["skip_reason"] = f"steepness {steepness:.4f} > {steepness_max}"
                    cell["mean_absorbed_power_W"] = float("nan")
                    cell["total_absorbed_energy_J"] = float("nan")
                    cell["wall_time_s"] = 0.0
                    cell["exit_code"] = -1
                else:
                    cell["status"] = 0  # ok
                    cell["skip_reason"] = ""
                    power = 500.0 * hs**2 * tp * (0.8 + 0.2 * np.random.rand())
                    sim_duration = 120.0
                    cell["mean_absorbed_power_W"] = power
                    cell["total_absorbed_energy_J"] = power * sim_duration
                    cell["wall_time_s"] = 5.0 + 10.0 * np.random.rand()
                    cell["exit_code"] = 0
                cells.append(cell)
    return cells


def write_summary_hdf5(path, cells, hs_values, tp_values, seeds,
                       scatter_weights=None):
    """Write power_matrix_summary.h5 in the proposed schema."""
    n = len(cells)

    with h5py.File(path, "w") as f:
        # /meta
        meta = f.create_group("meta")
        meta.attrs["version"] = "1"
        meta.attrs["seastack_version"] = "0.9.0-dev"
        meta.attrs["created_utc"] = "2026-03-26T12:00:00Z"
        meta.attrs["campaign_file"] = "example_campaign.yaml"
        meta.attrs["campaign_hash"] = "abc123synthetic"
        meta.attrs["model_file"] = "5sa.model.yaml"
        meta.attrs["simulation_file"] = "5sa.simulation.yaml"
        meta.attrs["hydro_template"] = "5sa.hydro.yaml"

        # /cells
        grp = f.create_group("cells")
        grp.create_dataset("hs", data=np.array([c["hs"] for c in cells]))
        grp.create_dataset("tp", data=np.array([c["tp"] for c in cells]))
        grp.create_dataset("heading_deg", data=np.array([c["heading_deg"] for c in cells]))
        grp.create_dataset("seed", data=np.array([c["seed"] for c in cells], dtype=np.int32))
        grp.create_dataset("status", data=np.array([c["status"] for c in cells], dtype=np.uint8))

        dt = h5py.special_dtype(vlen=str)
        reasons = np.array([c["skip_reason"] for c in cells], dtype=object)
        grp.create_dataset("skip_reason", data=reasons, dtype=dt)

        grp.create_dataset("mean_absorbed_power_W",
                           data=np.array([c["mean_absorbed_power_W"] for c in cells]))
        grp.create_dataset("total_absorbed_energy_J",
                           data=np.array([c["total_absorbed_energy_J"] for c in cells]))
        grp.create_dataset("wall_time_s",
                           data=np.array([c["wall_time_s"] for c in cells]))
        grp.create_dataset("exit_code",
                           data=np.array([c["exit_code"] for c in cells], dtype=np.int32))

        # /axes
        axes = f.create_group("axes")
        axes.create_dataset("hs", data=np.array(hs_values))
        axes.create_dataset("tp", data=np.array(tp_values))
        axes.create_dataset("seed", data=np.array(seeds, dtype=np.int32))

        # /aep (if scatter weights provided)
        if scatter_weights is not None:
            hours_per_year = 8766.0
            aep_wh = 0.0
            for c in cells:
                if c["status"] == 0:
                    key = (c["hs"], c["tp"])
                    w = scatter_weights.get(key, 0.0)
                    aep_wh += c["mean_absorbed_power_W"] * w * hours_per_year
            aep_grp = f.create_group("aep")
            aep_grp.attrs["annual_energy_Wh"] = aep_wh
            aep_grp.attrs["hours_per_year"] = hours_per_year
            aep_grp.attrs["method"] = "weighted_sum"

    return path


def write_summary_csv(path, cells):
    """Write flat CSV summary alongside HDF5."""
    header = "hs,tp,heading_deg,seed,status,skip_reason,mean_absorbed_power_W,total_absorbed_energy_J,wall_time_s\n"
    with open(path, "w") as f:
        f.write(header)
        for c in cells:
            skip = c["skip_reason"].replace(",", ";")
            f.write(f'{c["hs"]},{c["tp"]},{c["heading_deg"]},{c["seed"]},'
                    f'{c["status"]},{skip},{c["mean_absorbed_power_W"]},'
                    f'{c["total_absorbed_energy_J"]},{c["wall_time_s"]}\n')
    return path


def write_example_campaign_yaml(path):
    """Write an example campaign YAML matching the proposed schema."""
    content = """\
# Example SEA-Stack power-matrix campaign file.
# This is the minimal form — only kind, case, and axes are required.
kind: performance_matrix

case: ./5sa/spreading/

axes:
  hs: [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
  tp: [4, 6, 8, 10, 12]
"""
    with open(path, "w") as f:
        f.write(content)
    return path


def write_full_campaign_yaml(path):
    """Write a full campaign YAML with all optional knobs."""
    content = """\
kind: performance_matrix
version: 1

case:
  directory: ./5sa/spreading/
  setup: 5sa_spreading.setup.yaml

axes:
  hs: { linspace: { start: 0.5, stop: 5.0, num: 10 } }
  tp: { linspace: { start: 4.0, stop: 14.0, num: 11 } }
  heading: [0, 45, 90]
  seed: [1, 2, 3]

filters:
  steepness_max: 0.07

scatter:
  file: ./billia_croo_scatter.csv
  hs_column: Hs
  tp_column: Tp
  weight_column: probability

output:
  directory: ./power_matrix_output/
  per_cell_h5: false
  csv: true
"""
    with open(path, "w") as f:
        f.write(content)
    return path


def print_hdf5_structure(path):
    """Print HDF5 file structure for visual inspection."""
    print(f"\n{'='*60}")
    print(f"  HDF5 schema validation: {os.path.basename(path)}")
    print(f"{'='*60}")

    def visitor(name, obj):
        indent = "  " * (name.count("/") + 1)
        if isinstance(obj, h5py.Group):
            print(f"{indent}/{name}/")
            for k, v in obj.attrs.items():
                print(f"{indent}  @{k} = {v}")
        elif isinstance(obj, h5py.Dataset):
            shape_str = str(obj.shape)
            dtype_str = str(obj.dtype)
            print(f"{indent}{name}  [{dtype_str}, {shape_str}]")
            if obj.shape[0] > 0 and obj.shape[0] <= 5:
                print(f"{indent}  -> {obj[:]}")
            elif obj.shape[0] > 5:
                print(f"{indent}  -> [{obj[0]}, {obj[1]}, ..., {obj[-1]}]")

    with h5py.File(path, "r") as f:
        for k, v in f.attrs.items():
            print(f"  @{k} = {v}")
        f.visititems(visitor)

    print()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=".",
                        help="Directory for output files (default: cwd)")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    np.random.seed(42)

    hs_values = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    tp_values = [4, 6, 8, 10, 12]
    seeds = [1]

    cells = build_synthetic_cells(hs_values, tp_values, seeds)

    scatter_weights = {}
    for hs in hs_values:
        for tp in tp_values:
            scatter_weights[(hs, tp)] = 1.0 / (len(hs_values) * len(tp_values))

    h5_path = os.path.join(args.output_dir, "power_matrix_summary.h5")
    csv_path = os.path.join(args.output_dir, "power_matrix_summary.csv")
    yaml_min_path = os.path.join(args.output_dir, "example_campaign_minimal.yaml")
    yaml_full_path = os.path.join(args.output_dir, "example_campaign_full.yaml")

    write_summary_hdf5(h5_path, cells, hs_values, tp_values, seeds,
                       scatter_weights=scatter_weights)
    write_summary_csv(csv_path, cells)
    write_example_campaign_yaml(yaml_min_path)
    write_full_campaign_yaml(yaml_full_path)

    print_hdf5_structure(h5_path)

    n_ok = sum(1 for c in cells if c["status"] == 0)
    n_skipped = sum(1 for c in cells if c["status"] == 1)
    print(f"Summary: {len(cells)} cells total, {n_ok} ok, {n_skipped} skipped")
    print(f"  HDF5:  {h5_path}")
    print(f"  CSV:   {csv_path}")
    print(f"  YAML (minimal): {yaml_min_path}")
    print(f"  YAML (full):    {yaml_full_path}")

    if yaml is not None:
        for yp in [yaml_min_path, yaml_full_path]:
            with open(yp) as f:
                doc = yaml.safe_load(f)
            assert doc["kind"] == "performance_matrix", f"Bad kind in {yp}"
            print(f"  YAML parse OK: {os.path.basename(yp)}")
    else:
        print("  (pyyaml not installed — skipping YAML round-trip check)")

    print("\nSchema validation passed.")


if __name__ == "__main__":
    main()
