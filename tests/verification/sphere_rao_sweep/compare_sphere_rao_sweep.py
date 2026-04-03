#!/usr/bin/env python3
"""
Sphere RAO Sweep Verification -- extract heave RAO from SEA-Stack regular-wave
results and compare against multicode reference data.

Usage (called automatically by CTest):
    python compare_sphere_rao_sweep.py <normalized_dir> <regression_input_dir> <verification_output_dir>

    normalized_dir:         data/verification/sphere_rao_sweep/normalized
    regression_input_dir:  directory containing results_sphere_reg_waves_N.txt (where to read from)
    verification_output_dir: directory where verification outputs should be written
"""

import sys
import os
import re
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'regression', 'utilities'))
from compare_template import write_status_file

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utilities'))
from rao_extraction import extract_rao
from verification_plot import create_verification_rao_plot

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'data', 'verification'))
from normalize_utils import load_standard

N_CONDITIONS = 10
PRIMARY_REF = 'inwave'
DISPLAY_NAMES = {
    'inwave_hotint': 'InWave-HOTINT',
    'inwave': 'InWave',
    'marin': 'Marin',
    'nrel': 'NREL',
    'proteusds': 'ProteusDS',
}

RAO_TOL_PASS = 0.15
RAO_TOL_WARN = 0.30


def parse_result_header(filepath):
    """Parse wave amplitude and omega from a SEA-Stack result file header."""
    wave_amp = None
    omega = None
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('Wave amplitude'):
                wave_amp = float(line.split('\t')[-1].strip())
            elif line.startswith('Wave omega'):
                omega = float(line.split('\t')[-1].strip())
            if wave_amp is not None and omega is not None:
                break
    return wave_amp, omega


def load_result_timeseries(filepath):
    """Load time and response columns, skipping the header."""
    lines = []
    with open(filepath, 'r') as f:
        for line in f:
            stripped = line.strip()
            if stripped and stripped[0].isdigit():
                lines.append(stripped)
            elif stripped.startswith('-') and len(stripped) > 2:
                continue
    data = np.loadtxt(lines)
    return data[:, 0], data[:, 1]


def main():
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <normalized_dir> <regression_input_dir> <verification_output_dir>")
        sys.exit(1)

    normalized_dir = Path(sys.argv[1])
    regression_input_dir = Path(sys.argv[2])  # Where to read regression test outputs from
    verification_output_dir = Path(sys.argv[3])  # Where to write verification outputs
    output_dir = verification_output_dir / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    sim_omegas, sim_amps, sim_phases = [], [], []
    found_conditions = 0

    for n in range(1, N_CONDITIONS + 1):
        result_file = regression_input_dir / f"results_sphere_reg_waves_{n}.txt"
        if not result_file.exists():
            print(f"  Condition {n}: result file not found, skipping")
            continue

        wave_amp, omega = parse_result_header(result_file)
        if wave_amp is None or omega is None:
            print(f"  Condition {n}: could not parse header, skipping")
            continue

        time, heave = load_result_timeseries(result_file)
        result = extract_rao(time, heave, omega, wave_amp)
        sim_omegas.append(omega)
        sim_amps.append(result['rao_amplitude'])
        sim_phases.append(result['rao_phase'])
        found_conditions += 1

        stat = "OK" if result['stationarity_ok'] else "WARN(non-stationary)"
        print(f"  Condition {n}: T={2*np.pi/omega:.1f}s, "
              f"RAO={result['rao_amplitude']:.4f}, phase={np.degrees(result['rao_phase']):.1f}deg [{stat}]")

    if found_conditions == 0:
        print("ERROR: No result files found")
        write_status_file(str(verification_output_dir), "sphere_rao_sweep", "FAIL",
                          note="No result files found")
        sys.exit(1)

    sim_omega = np.array(sim_omegas)
    sim_amp = np.array(sim_amps)
    sim_phase = np.array(sim_phases)

    sort_idx = np.argsort(sim_omega)
    sim_omega = sim_omega[sort_idx]
    sim_amp = sim_amp[sort_idx]
    sim_phase = sim_phase[sort_idx]

    ref_sources = {}
    for dirname in sorted(normalized_dir.iterdir()):
        if not dirname.is_dir():
            continue
        rao_file = dirname / 'heave_rao.txt'
        if not rao_file.exists():
            continue
        meta, data = load_standard(rao_file)
        omega_ref = data[:, 0]
        amp_ref = data[:, 1]
        sort_r = np.argsort(omega_ref)
        ref_sources[dirname.name] = (omega_ref[sort_r], amp_ref[sort_r], None)

    if PRIMARY_REF not in ref_sources:
        print(f"WARNING: primary reference '{PRIMARY_REF}' not found in {normalized_dir}")
        primary_key = next(iter(ref_sources)) if ref_sources else None
    else:
        primary_key = PRIMARY_REF

    if primary_key is None:
        print("ERROR: No reference data found")
        write_status_file(str(verification_output_dir), "sphere_rao_sweep", "FAIL",
                          note="No reference data")
        sys.exit(1)

    ref_omega, ref_amp, _ = ref_sources[primary_key]

    extra_sources = {}
    for name, (w, a, p) in ref_sources.items():
        if name == primary_key:
            continue
        display = DISPLAY_NAMES.get(name, name)
        extra_sources[display] = (w, a, p)

    sim_amp_interp = np.interp(ref_omega, sim_omega, sim_amp)
    amp_errors = np.abs(sim_amp_interp - ref_amp) / np.maximum(ref_amp, 1e-10)

    metrics = create_verification_rao_plot(
        ref_omega, ref_amp, None,
        sim_omega, sim_amp, sim_phase,
        title='Sphere Heave RAO — Multicode Comparison',
        amp_label='Heave RAO (m/m)',
        output_path=str(output_dir / 'sphere_rao_sweep_heave_rao.png'),
        ref_label=DISPLAY_NAMES.get(primary_key, primary_key),
        sim_label='SEA-Stack',
        amp_errors=amp_errors,
        ref_file_path=str(normalized_dir / primary_key / 'heave_rao.txt'),
        test_file_path=str(regression_input_dir),
        extra_sources=extra_sources,
    )

    max_err = float(np.max(amp_errors)) if len(amp_errors) > 0 else 0.0
    mean_err = float(np.mean(amp_errors)) if len(amp_errors) > 0 else 0.0

    if max_err <= RAO_TOL_PASS:
        status = "PASS"
    elif max_err <= RAO_TOL_WARN:
        status = "WARN"
    else:
        status = "FAIL"

    all_metrics = {
        'max_amp_rel_err': max_err,
        'mean_amp_rel_err': mean_err,
        'n_conditions': found_conditions,
    }

    for name, (w, a, _) in ref_sources.items():
        src_interp = np.interp(w, sim_omega, sim_amp)
        src_err = np.abs(src_interp - a) / np.maximum(a, 1e-10)
        all_metrics[f'{name}_max_rel_err'] = float(np.max(src_err))
        all_metrics[f'{name}_mean_rel_err'] = float(np.mean(src_err))

    write_status_file(str(verification_output_dir), "sphere_rao_sweep", status, all_metrics)

    print(f"\nSphere RAO sweep: max_rel_err={max_err:.1%}, mean_rel_err={mean_err:.1%} -> {status}")
    sys.exit(1 if status == "FAIL" else 0)


if __name__ == '__main__':
    main()
