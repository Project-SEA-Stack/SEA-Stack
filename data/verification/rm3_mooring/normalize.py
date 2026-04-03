#!/usr/bin/env python3
"""
Re-wrap RM3 mooring reference files with the standard # metadata header.

The original reference files (from extract_wecsim_ref.py) use a bare
column-name header line (no # prefix). This script reads them and rewrites
in-place using normalize_utils.write_timeseries so they conform to the
standard format expected by load_standard() / load_and_normalize().
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from normalize_utils import write_timeseries

HERE = os.path.dirname(os.path.abspath(__file__))
NORM_DIR = os.path.join(HERE, 'normalized')


def _load_plain(path):
    """Load a file whose first line is a bare column header (no # prefix)."""
    return np.loadtxt(path, skiprows=1)


def main():
    # Body motions: Time, FloatHeaveZ, PlateHeaveZ (absolute CG positions)
    body_file = os.path.join(NORM_DIR, 'wecsim_moordyn_body_motions.txt')
    body_data = _load_plain(body_file)
    write_timeseries(
        os.path.join(NORM_DIR, 'wecsim_moordyn_body_motions.txt'),
        body_data,
        source='wecsim_moordyn',
        model='rm3_mooring',
        quantity='body_motions',
        units='s, m, m',
        columns='Time(s) FloatHeaveZ(m) PlateHeaveZ(m)',
        extra_comments=[
            "Absolute CG Z positions from WEC-Sim/MoorDyn",
            "Float equilibrium ~ -0.72 m, Plate equilibrium ~ -21.50 m",
        ],
    )
    print(f"Wrote normalized body motions ({body_data.shape[0]} rows)")

    # Fairlead tensions: Time, FairTen4, FairTen5, FairTen6
    tension_file = os.path.join(NORM_DIR, 'wecsim_moordyn_fairlead_tensions.txt')
    tension_data = _load_plain(tension_file)
    write_timeseries(
        os.path.join(NORM_DIR, 'wecsim_moordyn_fairlead_tensions.txt'),
        tension_data,
        source='wecsim_moordyn',
        model='rm3_mooring',
        quantity='fairlead_tensions',
        units='s, N, N, N',
        columns='Time(s) FairTen4(N) FairTen5(N) FairTen6(N)',
        extra_comments=["Fairlead tensions from MoorDyn (inside WEC-Sim)"],
    )
    print(f"Wrote normalized fairlead tensions ({tension_data.shape[0]} rows)")

    # Wave elevation: Time, Elevation
    wave_file = os.path.join(NORM_DIR, 'wecsim_moordyn_wave_elevation.txt')
    wave_data = _load_plain(wave_file)
    write_timeseries(
        os.path.join(NORM_DIR, 'wecsim_moordyn_wave_elevation.txt'),
        wave_data,
        source='wecsim_moordyn',
        model='rm3_mooring',
        quantity='wave_elevation',
        units='s, m',
        columns='Time(s) Elevation(m)',
        extra_comments=["Wave elevation from WEC-Sim co-simulation"],
    )
    print(f"Wrote normalized wave elevation ({wave_data.shape[0]} rows)")

    print("\nDone. All files now use standard # metadata headers.")


if __name__ == '__main__':
    main()
