#!/usr/bin/env python3
"""
Compare kPolar vs kCartesian excitation interpolation for regular-wave regression cases.

Reports L2 and L-infinity norms between polar and cartesian results for all 27 cases.
Also reports polar vs reference and cartesian vs reference to confirm pass status.

Usage:
    python compare_polar_vs_cartesian.py <polar_dir> <cartesian_dir> <reference_base_dir>

Example:
    python compare_polar_vs_cartesian.py \\
        build/bin/Release/results_polar \\
        build/bin/Release/results_cartesian \\
        data/reference_data
"""

import sys
import numpy as np
from pathlib import Path

def clip_to_common_time(ref_data, test_data):
    """Clip both datasets to their overlapping time range."""
    t_start = max(ref_data[0, 0], test_data[0, 0])
    t_end = min(ref_data[-1, 0], test_data[-1, 0])
    if t_end <= t_start:
        return ref_data, test_data
    ref_mask = (ref_data[:, 0] >= t_start) & (ref_data[:, 0] <= t_end)
    test_mask = (test_data[:, 0] >= t_start) & (test_data[:, 0] <= t_end)
    return ref_data[ref_mask], test_data[test_mask]


def find_data_start(filename):
    """Return line index where numeric data starts."""
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            try:
                float(line.split()[0])
                return i
            except (ValueError, IndexError):
                continue
    return 0


def load_data(fpath):
    """Load data file, skipping header lines."""
    skip = find_data_start(str(fpath))
    return np.loadtxt(str(fpath), skiprows=skip)


def compute_norms(ref_data, test_data, ncols=1):
    """Compute L2 and L-inf per column. ref_data and test_data have time in col 0."""
    ref_clip, test_clip = clip_to_common_time(ref_data, test_data)
    nval = test_clip.shape[0]
    x = np.linspace(test_clip[0, 0], test_clip[-1, 0], nval)
    norms = []
    for c in range(1, min(ncols + 1, ref_clip.shape[1], test_clip.shape[1])):
        y1 = np.interp(x, ref_clip[:, 0], ref_clip[:, c])
        y2 = np.interp(x, test_clip[:, 0], test_clip[:, c])
        yd = y1 - y2
        n1 = np.linalg.norm(yd) / nval
        n2 = np.linalg.norm(yd, np.inf)
        norms.append((n1, n2))
    return norms


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)

    polar_dir = Path(sys.argv[1])
    cartesian_dir = Path(sys.argv[2])
    ref_base = Path(sys.argv[3])

    if not polar_dir.exists():
        print(f"Error: Polar results dir not found: {polar_dir}")
        sys.exit(1)
    if not cartesian_dir.exists():
        print(f"Error: Cartesian results dir not found: {cartesian_dir}")
        sys.exit(1)

    pass_criteria = (1e-4, 0.02)  # L2, L-inf

    results = []

    # Sphere: 10 cases
    for i in range(1, 11):
        polar_f = polar_dir / f"results_sphere_reg_waves_{i}.txt"
        cart_f = cartesian_dir / f"results_sphere_reg_waves_{i}.txt"
        ref_f = ref_base / "sphere" / f"ss_ref_sphere_reg_waves_{i}.txt"
        if not polar_f.exists() or not cart_f.exists():
            continue
        polar_data = load_data(polar_f)
        cart_data = load_data(cart_f)
        ref_data = load_data(ref_f) if ref_f.exists() else None

        # Polar vs Cartesian
        norms_pc = compute_norms(polar_data, cart_data, 1)
        l2_pc, linf_pc = norms_pc[0]
        passed_pc = l2_pc <= pass_criteria[0] and linf_pc <= pass_criteria[1]

        # Polar vs reference
        passed_polar = True
        if ref_data is not None:
            norms_pr = compute_norms(ref_data, polar_data, 1)
            l2_pr, linf_pr = norms_pr[0]
            passed_polar = l2_pr <= pass_criteria[0] and linf_pr <= pass_criteria[1]

        # Cartesian vs reference
        passed_cart = True
        if ref_data is not None:
            norms_cr = compute_norms(ref_data, cart_data, 1)
            l2_cr, linf_cr = norms_cr[0]
            passed_cart = l2_cr <= pass_criteria[0] and linf_cr <= pass_criteria[1]

        results.append({
            "model": "sphere",
            "case": i,
            "l2_polar_cart": l2_pc,
            "linf_polar_cart": linf_pc,
            "passed_polar_cart": passed_pc,
            "passed_polar_ref": passed_polar,
            "passed_cart_ref": passed_cart,
        })

    # OSWEC: 16 cases
    for i in range(1, 17):
        polar_f = polar_dir / f"results_oswec_reg_waves_{i}.txt"
        cart_f = cartesian_dir / f"results_oswec_reg_waves_{i}.txt"
        ref_f = ref_base / "oswec" / f"ss_ref_oswec_reg_waves_{i}.txt"
        if not polar_f.exists() or not cart_f.exists():
            continue
        polar_data = load_data(polar_f)
        cart_data = load_data(cart_f)
        ref_data = load_data(ref_f) if ref_f.exists() else None

        norms_pc = compute_norms(polar_data, cart_data, 1)
        l2_pc, linf_pc = norms_pc[0]
        passed_pc = l2_pc <= pass_criteria[0] and linf_pc <= pass_criteria[1]

        passed_polar = True
        passed_cart = True
        if ref_data is not None:
            norms_pr = compute_norms(ref_data, polar_data, 1)
            norms_cr = compute_norms(ref_data, cart_data, 1)
            passed_polar = (norms_pr[0][0] <= pass_criteria[0] and
                           norms_pr[0][1] <= pass_criteria[1])
            passed_cart = (norms_cr[0][0] <= pass_criteria[0] and
                          norms_cr[0][1] <= pass_criteria[1])

        results.append({
            "model": "oswec",
            "case": i,
            "l2_polar_cart": l2_pc,
            "linf_polar_cart": linf_pc,
            "passed_polar_cart": passed_pc,
            "passed_polar_ref": passed_polar,
            "passed_cart_ref": passed_cart,
        })

    # RM3: 1 case, 3 columns
    polar_f = polar_dir / "results_rm3_reg_waves.txt"
    cart_f = cartesian_dir / "results_rm3_reg_waves.txt"
    ref_f = ref_base / "rm3" / "ss_ref_rm3_reg_waves.txt"
    if polar_f.exists() and cart_f.exists():
        polar_data = load_data(polar_f)
        cart_data = load_data(cart_f)
        ref_data = load_data(ref_f) if ref_f.exists() else None

        norms_pc = compute_norms(polar_data, cart_data, 3)
        # Use worst column for pass
        l2_pc = max(n[0] for n in norms_pc)
        linf_pc = max(n[1] for n in norms_pc)
        passed_pc = l2_pc <= pass_criteria[0] and linf_pc <= pass_criteria[1]

        passed_polar = True
        passed_cart = True
        if ref_data is not None:
            norms_pr = compute_norms(ref_data, polar_data, 3)
            norms_cr = compute_norms(ref_data, cart_data, 3)
            passed_polar = all(n[0] <= pass_criteria[0] and n[1] <= pass_criteria[1]
                              for n in norms_pr)
            passed_cart = all(n[0] <= pass_criteria[0] and n[1] <= pass_criteria[1]
                              for n in norms_cr)

        results.append({
            "model": "rm3",
            "case": 1,
            "l2_polar_cart": l2_pc,
            "linf_polar_cart": linf_pc,
            "passed_polar_cart": passed_pc,
            "passed_polar_ref": passed_polar,
            "passed_cart_ref": passed_cart,
        })

    # Report
    print("=" * 80)
    print("kPolar vs kCartesian Excitation Interpolation Analysis")
    print("=" * 80)
    print(f"Pass criteria: L2 <= {pass_criteria[0]}, L-inf <= {pass_criteria[1]}")
    print()

    print("Case                    | L2(Polar-Cart) | L-inf(Polar-Cart) | P-C match | Polar vs Ref | Cart vs Ref")
    print("-" * 95)

    for r in results:
        label = f"{r['model']}_{r['case']}"
        pc_ok = "PASS" if r["passed_polar_cart"] else "FAIL"
        pr_ok = "PASS" if r["passed_polar_ref"] else "FAIL"
        cr_ok = "PASS" if r["passed_cart_ref"] else "FAIL"
        print(f"{label:23} | {r['l2_polar_cart']:14.2e} | {r['linf_polar_cart']:17.2e} | {pc_ok:9} | {pr_ok:11} | {cr_ok}")

    print("-" * 95)
    n = len(results)
    n_pc = sum(1 for r in results if r["passed_polar_cart"])
    n_pr = sum(1 for r in results if r["passed_polar_ref"])
    n_cr = sum(1 for r in results if r["passed_cart_ref"])

    print(f"\nSummary: {n} cases")
    print(f"  Polar vs Cartesian within tolerance: {n_pc}/{n}")
    print(f"  kPolar vs SEA-Stack regression reference (ss_ref_):     {n_pr}/{n}")
    print(f"  kCartesian vs SEA-Stack regression reference (ss_ref_): {n_cr}/{n}")

    max_l2 = max(r["l2_polar_cart"] for r in results)
    max_linf = max(r["linf_polar_cart"] for r in results)
    print(f"\nMax Polar-Cartesian difference: L2={max_l2:.2e}, L-inf={max_linf:.2e}")

    if n_pr == n and n_cr == n:
        print("\nCONCLUSION: Both kPolar and kCartesian pass all SEA-Stack regression reference cases.")
    elif n_pr == n:
        print("\nCONCLUSION: Only kPolar passes all SEA-Stack regression reference cases.")
    elif n_cr == n:
        print("\nCONCLUSION: Only kCartesian passes all SEA-Stack regression reference cases.")
    else:
        print("\nCONCLUSION: Neither passes all cases (unexpected).")

    sys.exit(0 if n_pr == n and n_cr == n else 1)


if __name__ == "__main__":
    main()
