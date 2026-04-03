#!/usr/bin/env python3
"""
Normalize sphere decay multi-code verification data.

Per CONVENTIONS.md, all displacements are relative to equilibrium in metres.

Raw input:
  raw/multicode/sphere_decay_ref_data.txt
    Tab-delimited, 7 columns: Time(s), ProteusDS, InWave, Marin, NREL, WavEC, InWave+H
    Heave is normalized (dimensionless), initial displacement = 1 m.
    Since z0 = 1 m, displacement_m = NormalizedHeave * 1.0.

  raw/hydrochrono/sphere_decay_hc_data.txt
    Whitespace-delimited, 2 columns: Time(s), Heave(m)
    Absolute position; equilibrium estimated from tail of signal.
    displacement_m = Heave_absolute - equilibrium_z.

Output (normalized/):
  One file per tool: heave displacement from equilibrium in metres.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from normalize_utils import write_timeseries

HERE = os.path.dirname(os.path.abspath(__file__))

_INITIAL_DISPLACEMENT_M = 1.0


def main():
    # --- Multi-code decay (normalized heave -> metres from equilibrium) ---
    raw_multicode = os.path.join(HERE, 'raw', 'multicode', 'sphere_decay_ref_data.txt')
    data = np.loadtxt(raw_multicode, skiprows=1, delimiter='\t')

    # Raw columns: Time(s), ProteusDS, InWave, Marin, NREL, WavEC, InWave+H
    tool_names = ['proteusds', 'inwave', 'marin', 'nrel_cfd', 'wavec']
    col_indices = [1, 2, 3, 4, 5]  # skip InWave+H (col 6)
    for tool, col_idx in zip(tool_names, col_indices):
        t = data[:, 0]
        heave_normalised = data[:, col_idx]
        heave_m = heave_normalised * _INITIAL_DISPLACEMENT_M
        col = np.column_stack([t, heave_m])

        outpath = os.path.join(HERE, 'normalized', tool, 'heave_decay.txt')
        os.makedirs(os.path.dirname(outpath), exist_ok=True)
        notes = []
        if tool == 'marin':
            notes.append("Nonlinear simulation")
        else:
            notes.append("Linear simulation")
        notes.append(f"Converted from normalised heave * {_INITIAL_DISPLACEMENT_M} m")
        write_timeseries(
            outpath, col,
            source=tool, model='sphere_decay_multicode',
            quantity='heave_decay',
            units='s, m',
            columns='Time(s) Heave(m)',
            extra_comments=notes,
        )
    print(f"Wrote {len(tool_names)} normalized multi-code files (metres from equilibrium)")

    # --- HydroChrono decay (absolute heave -> metres from equilibrium) ---
    raw_hc = os.path.join(HERE, 'raw', 'hydrochrono', 'sphere_decay_hc_data.txt')
    with open(raw_hc, 'r') as f:
        skip = 0
        for line in f:
            try:
                float(line.split()[0])
                break
            except (ValueError, IndexError):
                skip += 1
    hc_data = np.loadtxt(raw_hc, skiprows=skip)

    # Estimate equilibrium from last 10% of signal (settled portion)
    n_tail = max(1, len(hc_data) // 10)
    eq_z = np.mean(hc_data[-n_tail:, 1])

    hc_data[:, 1] -= eq_z
    outpath = os.path.join(HERE, 'normalized', 'hydrochrono', 'heave_decay.txt')
    os.makedirs(os.path.dirname(outpath), exist_ok=True)
    write_timeseries(
        outpath, hc_data,
        source='hydrochrono', model='sphere_decay_multicode',
        quantity='heave_decay',
        units='s, m',
        columns='Time(s) Heave(m)',
        extra_comments=[f"Converted from absolute position, equilibrium = {eq_z:.4f} m"],
    )
    print(f"Wrote HydroChrono normalized file (equilibrium = {eq_z:.4f} m)")


if __name__ == '__main__':
    main()
