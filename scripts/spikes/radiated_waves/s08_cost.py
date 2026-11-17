"""S8: cost, memory and accuracy of the radial-profile synthesis.

Two ways to turn the velocity history into radial profiles p_{b,m}(r, t)
(one per body b and angular harmonic m; the water surface then evaluates
eta = sum_b sum_m p_{b,m}(r_b) * trig(m theta_b)):

 (1) time-domain kernels: p(r_i, t) = sum_j sum_n g_{j,m}(r_i, tau_n) u_j(t - tau_n) dtau
     (one kernel table N_r x N_tau per DOF and harmonic);
 (2) spectral: windowed Fourier transform of the last T_w seconds of velocity,
     U_j(w_n) = sum_{t' in window} u_j(t') e^{-i w_n t'} dt   (sliding, O(N_w) per sample),
     S_{b,m}(w_n) = sum_j a_{b,j,m}(w_n) U_j(w_n)            (source coefficients),
     p_{b,m}(r_i, t) = Re sum_n S_{b,m}(w_n) P_|m|(k_n r_i) e^{i w_n t} dw / pi
     (one propagation table P_m(k_n r_i) shared by all bodies and DOFs).

Both are pure functions of (velocity history, t), so pause, speed change and
seeking only need the history; (2) needs one O(T_w N_w) recompute after a seek.

Accuracy of (2) is checked against the exact convolution on the S7 decay.
"""
import json
import time

import numpy as np
from scipy.special import hankel2

import rwlib as R

PATCH_RADIUS = 400.0      # m, radiated field drawn out to this radius
CASES = {"sphere (1 body)": (1, 3, 2), "crane (1 body)": (1, 6, 2), "rm3 (2 bodies)": (2, 6, 2),
         "5SA (5 bodies)": (5, 6, 2), "5SA Haskind M=6": (5, 6, 6)}


def sizes(w_max, depth=250.0, g=9.81, r_max=PATCH_RADIUS):
    k_max = R.wavenumber(np.array([w_max]), g, depth)[0]
    lam_min = 2 * np.pi / k_max
    dr = lam_min / 8
    n_r = int(np.ceil(r_max / dr))
    cg_min = R.group_velocity(np.array([w_max]), g, depth)[0]
    tau_max = r_max / cg_min + 20.0             # + source duration
    dtau = min(0.1, np.pi / (4 * w_max))
    n_tau = int(np.ceil(tau_max / dtau))
    dw = np.pi / tau_max                         # alias period 2 tau_max
    n_w = int(np.ceil(w_max / dw))
    return dict(lam_min=lam_min, dr=dr, n_r=n_r, tau_max=tau_max, dtau=dtau, n_tau=n_tau, dw=dw, n_w=n_w)


def accuracy_check():
    """Spectral synthesis (2) vs exact convolution for the S7 sphere decay."""
    import s07_transient as S7
    h = R.load("sphere")
    t, x, v, *_ = S7.simulate(h)
    from s05_causality import setup, DT, N
    s = setup("sphere", 0, 2, 0.0, "axisym")
    wb, kb = s["w"], s["k"]
    Hs = s["mag"] * R.model_phase(0, -1.0)
    out = {}
    for r in (60.0, 150.0, 300.0):
        P = np.sqrt(np.pi / 2) * np.exp(-1j * np.pi / 4) * hankel2(0, np.maximum(kb * r, 1e-9))
        vpad = np.zeros(N); vpad[: len(v)] = v
        exact = np.fft.irfft(Hs * P * np.fft.rfft(vpad), n=N)[: len(v) + 1600]
        sz = sizes(3.0, h.depth, r_max=r)
        Tw, dw = sz["tau_max"], sz["dw"]
        wn = np.arange(1, sz["n_w"] + 1) * dw
        Hn = np.interp(wn, wb, Hs.real) + 1j * np.interp(wn, wb, Hs.imag)
        kn = R.wavenumber(wn, h.g, h.depth)
        Pn = np.sqrt(np.pi / 2) * np.exp(-1j * np.pi / 4) * hankel2(0, kn * r)
        tt = np.arange(len(exact)) * DT
        vv = np.zeros(len(exact)); vv[: len(v)] = v
        approx = np.zeros(len(exact))
        for i in range(0, len(exact), 10):
            lo = max(0, i - int(Tw / DT))
            tw = tt[lo: i + 1]
            Un = (np.exp(-1j * np.outer(wn, tw)) @ vv[lo: i + 1]) * DT
            approx[i] = np.real(np.sum(Hn * Pn * Un * np.exp(1j * wn * tt[i]))) * dw / np.pi
        idx = np.arange(0, len(exact), 10)
        err = np.sqrt(np.sum((approx[idx] - exact[idx]) ** 2) / np.sum(exact[idx] ** 2))
        out[f"r={r:.0f}m"] = {"rel_rms_error": float(err), "n_w": sz["n_w"], "window_s": float(Tw)}
    return out


def timing(n_bodies, n_dof, n_harm, sz, reps=20):
    rng = np.random.default_rng(0)
    n_r, n_w = sz["n_r"], sz["n_w"]
    P = (rng.standard_normal((n_harm + 1, n_r, n_w)) + 1j * rng.standard_normal((n_harm + 1, n_r, n_w))).astype(np.complex64)
    a = (rng.standard_normal((n_bodies, 2 * n_harm + 1, n_dof, n_w))).astype(np.complex64)
    U = (rng.standard_normal((n_bodies, n_dof, n_w))).astype(np.complex64)
    ph = np.exp(1j * np.arange(n_w) * 0.1).astype(np.complex64)
    t0 = time.perf_counter()
    for _ in range(reps):
        S = np.einsum("bmjn,bjn->bmn", a, U) * ph                  # source spectra
        prof = np.empty((n_bodies, 2 * n_harm + 1, n_r), np.float32)
        for mm in range(2 * n_harm + 1):
            m = abs(mm - n_harm)
            prof[:, mm, :] = np.real(S[:, mm, :] @ P[m].T)
    t_spec = (time.perf_counter() - t0) / reps
    # direct time-domain kernels (one table per DOF and harmonic actually used)
    n_ch = n_bodies * n_dof
    Kt = rng.standard_normal((n_r, sz["n_tau"])).astype(np.float32)
    hist = rng.standard_normal((sz["n_tau"], n_ch)).astype(np.float32)
    t0 = time.perf_counter()
    for _ in range(reps):
        _ = Kt @ hist
    t_direct = (time.perf_counter() - t0) / reps
    return t_spec, t_direct


def main():
    summary = {"assumptions": {"patch_radius_m": PATCH_RADIUS, "w_max_rad_s": 3.0, "dr": "lambda_min/8",
                               "dtau": "min(0.1 s, pi/(4 w_max))", "window": "r_max / c_g(w_max) + 20 s",
                               "dw": "pi / window"}}
    sz = sizes(3.0)
    summary["grid"] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in sz.items()}
    print("grid", summary["grid"])
    cost = {}
    for name, (nb, nd, M) in CASES.items():
        n_ch = nb * (4 if M == 2 else 2 * M + 1)               # approx: m0, m1c, m1s, m2s
        harm = 4 if M == 2 else 2 * M + 1
        mac_spec = nb * harm * sz["n_r"] * sz["n_w"] + nb * harm * nd * sz["n_w"]
        mac_direct = nb * nd * (1 if M == 2 else harm) * sz["n_r"] * sz["n_tau"]
        mem_spec = (M + 1) * sz["n_r"] * sz["n_w"] * 8 + nb * harm * nd * sz["n_w"] * 8 + nb * nd * sz["n_w"] * 16
        mem_direct = nb * nd * (1 if M == 2 else harm) * sz["n_r"] * sz["n_tau"] * 4
        hist = nb * 6 * sz["n_tau"] * 8
        t_spec, t_direct = timing(nb, nd, 1 if M == 2 else M, sz)
        if M != 2:
            t_direct *= harm
        gpu_flops_per_vertex = nb * (12 + harm * 6)
        cost[name] = {
            "profiles_per_frame": n_ch,
            "spectral_MAC_per_frame_M": round(mac_spec / 1e6, 2),
            "direct_MAC_per_frame_M": round(mac_direct / 1e6, 2),
            "spectral_memory_MB": round(mem_spec / 2 ** 20, 2),
            "direct_kernel_memory_MB": round(mem_direct / 2 ** 20, 1),
            "velocity_history_kB": round(hist / 1024, 1),
            "numpy_spectral_ms": round(t_spec * 1e3, 2),
            "numpy_direct_ms": round(t_direct * 1e3, 2),
            "upload_per_frame_kB": round(n_ch * sz["n_r"] * 4 / 1024, 1),
            "gpu_ops_per_vertex": gpu_flops_per_vertex,
            "gpu_Mops_per_frame_512x512": round(gpu_flops_per_vertex * 512 * 512 / 1e6, 1),
        }
        print(name, cost[name])
    summary["cost"] = cost
    print("accuracy of the spectral form vs exact convolution (sphere decay):")
    acc = accuracy_check()
    print(acc)
    summary["spectral_accuracy"] = acc
    (R.OUT / "s08_summary.json").write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
