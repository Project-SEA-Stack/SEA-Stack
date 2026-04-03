#!/usr/bin/env python3
"""
Normalize F3OF multi-code decay verification data.

Raw F3OF .dat format (INW example):
  Line 1: tool name (INW)
  Line 2: model (F3OF)
  Line 3: test case (DT1)
  Line 4: DOF (SURGE)
  Line 5: column headers
  Line 6+: numeric data
"""

import sys, os, glob
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from normalize_utils import write_timeseries

HERE = os.path.dirname(os.path.abspath(__file__))

DOF_MAP = {
    'SURGE': ('surge', 'm', False),
    'PITCH': ('pitch', 'rad', True),
    'HEAVE': ('heave', 'm', False),
    'FLAP1': ('flap_fore', 'rad', True),
    'FLAP2': ('flap_aft', 'rad', True),
}


def normalize_dat_file(raw_path, tool_lower):
    """Parse an F3OF .dat file and write normalized output.

    External codes (inw, wsm, wdn, pds) store rotational DOFs in degrees.
    HydroChrono stores them in radians.  This function converts degrees to
    radians for rotational DOFs so all normalized data is in SI units.
    """
    with open(raw_path, 'r') as f:
        lines = f.readlines()

    tool_name = lines[0].strip()
    test_case = lines[2].strip().lower()
    dof_raw = lines[3].strip()

    dof_key, dof_unit, is_rotational = DOF_MAP.get(dof_raw, (dof_raw.lower(), 'unknown', False))
    data = np.loadtxt(raw_path, skiprows=5)

    if is_rotational:
        data[:, 1] *= np.pi / 180.0

    out_name = f"{test_case}_{dof_key}.txt"
    out_path = os.path.join(HERE, 'normalized', tool_lower, out_name)
    col_header = f"Time(s) {dof_raw}({dof_unit})"

    notes = []
    if is_rotational:
        notes.append("Converted from degrees to radians")

    write_timeseries(
        out_path, data,
        source=tool_lower, model='f3of_decay_multicode',
        quantity=f'{test_case}_{dof_key}',
        units=f's, {dof_unit}',
        columns=col_header,
        extra_comments=notes if notes else None,
    )
    return out_name


def normalize_chrono_txt(raw_path):
    """Parse a HydroChrono F3OF postprocessing file."""
    fname = os.path.basename(raw_path)
    data = np.loadtxt(raw_path, skiprows=1)

    if 'DT1' in fname:
        test_case = 'dt1'
    elif 'DT2' in fname:
        test_case = 'dt2'
    elif 'DT3' in fname:
        test_case = 'dt3'
    else:
        return

    col_dofs = [
        ('surge', 'm'),
        ('pitch', 'rad'),
        ('flap_fore', 'rad'),
        ('flap_aft', 'rad'),
    ]

    for col_idx, (dof_key, dof_unit) in enumerate(col_dofs):
        col_data = data[:, [0, col_idx + 1]]
        out_name = f"{test_case}_{dof_key}.txt"
        out_path = os.path.join(HERE, 'normalized', 'hydrochrono', out_name)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        write_timeseries(
            out_path, col_data,
            source='hydrochrono', model='f3of_decay_multicode',
            quantity=f'{test_case}_{dof_key}',
            units=f's, {dof_unit}',
            columns=f'Time(s) {dof_key.upper()}({dof_unit})',
        )


def main():
    count = 0
    for tool in ['inw', 'wsm', 'wdn', 'pds']:
        raw_dir = os.path.join(HERE, 'raw', tool)
        for dat_file in sorted(glob.glob(os.path.join(raw_dir, '*.dat'))):
            out = normalize_dat_file(dat_file, tool)
            count += 1

    for txt_file in sorted(glob.glob(os.path.join(HERE, 'raw', 'hydrochrono', 'CHRONO_F3OF_*.txt'))):
        normalize_chrono_txt(txt_file)
        count += 1

    print(f"Normalized {count} F3OF files")


if __name__ == '__main__':
    main()
