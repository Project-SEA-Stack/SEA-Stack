#!/usr/bin/env python3
"""
Shared RAO extraction utility for verification comparison scripts.

Two extraction methods:
  - extract_rao: Single-frequency DFT (robust, gives phase).
  - extract_rao_peak: Peak-from-equilibrium or peak-to-peak amplitude.
"""

import numpy as np


def extract_rao_peak(response, wave_amplitude, equilibrium=None):
    """Extract RAO amplitude using peak extraction.

    If equilibrium is given: response_amplitude = |max(response) - equilibrium|

    If equilibrium is None: response_amplitude = (max - min) / 2
    (peak-to-peak, robust when mean position is unknown).

    Uses full signal (no steady-state window). Phase is not computed.

    Returns:
        dict with rao_amplitude, rao_phase (0), raw_amplitude,
        stationarity_ok (True), rms_residual (0), sub_window_rms_values ([]).
    """
    if equilibrium is not None:
        response_amplitude = float(np.abs(np.max(response) - equilibrium))
    else:
        response_amplitude = float((np.max(response) - np.min(response)) / 2.0)
    rao_amplitude = response_amplitude / max(wave_amplitude, 1e-15)
    return {
        'rao_amplitude': rao_amplitude,
        'rao_phase': 0.0,  # not computed; use 0 for compatibility
        'rms_residual': 0.0,
        'stationarity_ok': True,
        'sub_window_rms_values': [],
        'raw_amplitude': response_amplitude,
    }


def extract_rao(time, response, omega, wave_amplitude, steady_state_fraction=0.5):
    """Extract RAO amplitude and phase from a regular-wave time series.

    Algorithm:
      1. Select steady-state window (last `steady_state_fraction` of signal).
      2. Validate stationarity: RMS over 4 sub-windows must agree within 5%.
      3. Single-frequency DFT: C = (2/N) * sum(y[n] * exp(-j*omega*t[n])).
      4. RAO amplitude = |C| / wave_amplitude.
      5. RAO phase = arg(C), relative to cos(omega*t), positive = lead.
      6. Quality metric: RMS residual of fitted sinusoid.

    Returns:
        dict with keys: rao_amplitude, rao_phase, rms_residual,
                        stationarity_ok, sub_window_rms_values
    """
    n_total = len(time)
    i_start = int(n_total * (1.0 - steady_state_fraction))
    t_ss = time[i_start:]
    y_ss = response[i_start:]
    n_ss = len(t_ss)

    # Stationarity check: split into 4 sub-windows
    sub_len = n_ss // 4
    sub_rms = []
    for k in range(4):
        seg = y_ss[k * sub_len : (k + 1) * sub_len]
        sub_rms.append(np.std(seg))
    mean_rms = np.mean(sub_rms)
    stationarity_ok = True
    if mean_rms > 1e-15:
        deviations = [abs(r - mean_rms) / mean_rms for r in sub_rms]
        if max(deviations) > 0.05:
            stationarity_ok = False

    # Remove DC offset before DFT to prevent spectral leakage from non-zero
    # equilibrium positions (e.g. sphere at z=-2 m).
    y_centered = y_ss - np.mean(y_ss)
    complex_coeff = (2.0 / n_ss) * np.sum(y_centered * np.exp(-1j * omega * t_ss))

    amplitude = np.abs(complex_coeff)
    phase = np.angle(complex_coeff)

    rao_amplitude = amplitude / wave_amplitude

    # Quality: RMS residual of fit
    fitted = amplitude * np.cos(omega * t_ss + phase)
    rms_residual = np.std(y_ss - fitted)

    return {
        'rao_amplitude': float(rao_amplitude),
        'rao_phase': float(phase),
        'rms_residual': float(rms_residual),
        'stationarity_ok': stationarity_ok,
        'sub_window_rms_values': [float(r) for r in sub_rms],
        'raw_amplitude': float(amplitude),
    }


def self_test():
    """Validate RAO extraction on synthetic sinusoids with known amplitude and phase."""
    passed = 0
    failed = 0

    for omega in [0.5, 1.0, 2.0]:
        for amp_true in [0.5, 1.0, 2.5]:
            for phase_true in [0.0, 0.3, -0.7]:
                wave_amp = 0.1
                response_amp = amp_true * wave_amp
                dt = 0.01
                t = np.arange(0, 200, dt)
                y = response_amp * np.cos(omega * t + phase_true)

                result = extract_rao(t, y, omega, wave_amp)
                rao_err = abs(result['rao_amplitude'] - amp_true) / max(amp_true, 1e-10)
                phase_err = abs(result['rao_phase'] - phase_true)
                phase_err = abs((phase_err + np.pi) % (2 * np.pi) - np.pi)

                ok = rao_err < 0.01 and phase_err < 0.01
                if ok:
                    passed += 1
                else:
                    failed += 1
                    print(f"  FAIL: omega={omega}, amp={amp_true}, phase={phase_true} "
                          f"-> rao_err={rao_err:.2e}, phase_err={phase_err:.2e}")

    print(f"RAO extraction self-test: {passed} passed, {failed} failed")
    return failed == 0


if __name__ == '__main__':
    import sys
    ok = self_test()
    sys.exit(0 if ok else 1)
