#!/usr/bin/env python3
"""Normalize OSWEC WEC-Sim decay reference data."""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from normalize_utils import write_timeseries

HERE = os.path.dirname(os.path.abspath(__file__))

def main():
    raw = os.path.join(HERE, 'raw', 'wecsim', 'wecsim_oswec_decay.txt')
    data = np.loadtxt(raw, skiprows=1, delimiter='\t')
    write_timeseries(
        os.path.join(HERE, 'normalized', 'wecsim', 'pitch_decay.txt'),
        data,
        source='wecsim', model='oswec_decay_wecsim',
        quantity='pitch_decay', units='s, rad',
        columns='Time(s) Pitch(rad)',
    )
    print("Wrote normalized OSWEC WEC-Sim decay")

if __name__ == '__main__':
    main()
