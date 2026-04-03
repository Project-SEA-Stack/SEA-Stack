#!/usr/bin/env python3
"""
Comparison analysis: LinearPTO vs RectifiedHydraulicPTO on RM3 in irregular waves.

Reads CSV output from the C++ comparison test and produces:
  - Scalar summary metrics (printed and written to .status.json)
  - Overlay time-series plots for human review

This script does NOT return a non-zero exit code based on metric values.
Its role is reporting, not gating.  See the implementation plan Section 6.4
for the rationale.
"""

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "utilities"))
from compare_template_internal import (
    plot_comparison,
    write_comparison_status,
)

import numpy as np

# Import shared plotting helpers for custom plots
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "utilities"))
try:
    from plot_helpers import apply_modern_style, FIGURE_DPI
    HAS_PLOT_HELPERS = True
except ImportError:
    HAS_PLOT_HELPERS = False
    FIGURE_DPI = 150  # Fallback

TEST_NAME = "compare_linear_vs_hydraulic_pto_rm3_irreg"

# Transient skip: accumulators charge + controller settling
SKIP_TIME = 45.0


def load_csv(path):
    """Load a CSV file with header row, returning a dict of columns."""
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data


def compute_scalar_metrics(linear, hydraulic, skip_time):
    """Compute summary scalars for both runs (post-transient window)."""
    mask_l = linear["time"] >= skip_time
    mask_h = hydraulic["time"] >= skip_time

    metrics = {}

    # Linear
    l_force = linear["pto_force"][mask_l]
    l_power = linear["pto_power"][mask_l]
    l_disp  = linear["rel_disp"][mask_l]
    l_vel   = linear["rel_vel"][mask_l]

    metrics["linear_mean_power_W"]    = float(np.mean(l_power))
    metrics["linear_peak_force_N"]    = float(np.max(np.abs(l_force)))
    metrics["linear_rms_disp_m"]      = float(np.sqrt(np.mean(l_disp**2)))
    metrics["linear_rms_vel_m_s"]     = float(np.sqrt(np.mean(l_vel**2)))

    # Hydraulic — common signals
    h_force = hydraulic["pto_force"][mask_h]
    h_power = hydraulic["pto_power"][mask_h]
    h_disp  = hydraulic["rel_disp"][mask_h]
    h_vel   = hydraulic["rel_vel"][mask_h]

    metrics["hydraulic_mean_mech_power_W"] = float(np.mean(h_power))
    metrics["hydraulic_peak_force_N"]      = float(np.max(np.abs(h_force)))
    metrics["hydraulic_rms_disp_m"]        = float(np.sqrt(np.mean(h_disp**2)))
    metrics["hydraulic_rms_vel_m_s"]       = float(np.sqrt(np.mean(h_vel**2)))

    # Hydraulic-specific signals
    h_hp   = hydraulic["hp_pressure"][mask_h]
    h_lp   = hydraulic["lp_pressure"][mask_h]
    h_omega = hydraulic["motor_speed"][mask_h]
    h_tgen  = hydraulic["gen_torque"][mask_h]
    h_elec  = hydraulic["elec_power"][mask_h]

    metrics["hydraulic_mean_elec_power_W"]  = float(np.mean(h_elec))
    metrics["hydraulic_mean_motor_speed_rad_s"] = float(np.mean(h_omega))
    metrics["hydraulic_mean_hp_pressure_Pa"] = float(np.mean(h_hp))
    metrics["hydraulic_mean_lp_pressure_Pa"] = float(np.mean(h_lp))
    metrics["hydraulic_hp_gt_lp"]            = bool(np.mean(h_hp) > np.mean(h_lp))
    metrics["hydraulic_motor_speed_positive"] = bool(np.all(h_omega >= 0))

    return metrics


def make_plots(linear, hydraulic, output_dir):
    """Generate overlay comparison plots."""
    t_l = linear["time"]
    t_h = hydraulic["time"]
    min_len = min(len(t_l), len(t_h))

    plot_dir = output_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    # Relative displacement
    plot_comparison(
        t_l[:min_len], linear["rel_disp"][:min_len],
        hydraulic["rel_disp"][:min_len],
        label_a="LinearPTO", label_b="HydraulicPTO",
        title=f"{TEST_NAME}: Relative Displacement",
        y_label="Relative displacement (m)",
        output_path=str(plot_dir / "rel_disp.png"),
        skip_time=SKIP_TIME,
    )

    # Relative velocity
    plot_comparison(
        t_l[:min_len], linear["rel_vel"][:min_len],
        hydraulic["rel_vel"][:min_len],
        label_a="LinearPTO", label_b="HydraulicPTO",
        title=f"{TEST_NAME}: Relative Velocity",
        y_label="Relative velocity (m/s)",
        output_path=str(plot_dir / "rel_vel.png"),
        skip_time=SKIP_TIME,
    )

    # PTO force
    plot_comparison(
        t_l[:min_len], linear["pto_force"][:min_len],
        hydraulic["pto_force"][:min_len],
        label_a="LinearPTO", label_b="HydraulicPTO",
        title=f"{TEST_NAME}: PTO Force",
        y_label="PTO force (N)",
        output_path=str(plot_dir / "pto_force.png"),
        skip_time=SKIP_TIME,
    )

    # PTO power
    plot_comparison(
        t_l[:min_len], linear["pto_power"][:min_len],
        hydraulic["pto_power"][:min_len],
        label_a="LinearPTO", label_b="HydraulicPTO",
        title=f"{TEST_NAME}: Instantaneous PTO Power",
        y_label="PTO power (W)",
        output_path=str(plot_dir / "pto_power.png"),
        skip_time=SKIP_TIME,
    )

    # Hydraulic-specific: HP/LP pressure, motor speed, generator torque
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

        ax = axes[0]
        ax.plot(t_h, hydraulic["hp_pressure"] / 1e6, label="HP pressure")
        ax.plot(t_h, hydraulic["lp_pressure"] / 1e6, label="LP pressure")
        ax.set_ylabel("Pressure (MPa)")
        ax.legend()
        if HAS_PLOT_HELPERS:
            apply_modern_style(ax)
        else:
            ax.grid(True, alpha=0.3)
        ax.set_title(f"{TEST_NAME}: Hydraulic PTO Internal State")

        ax = axes[1]
        ax.plot(t_h, hydraulic["motor_speed"], label="Motor speed")
        ax.axhline(104.72, color="gray", linestyle=":", alpha=0.5, label="Setpoint")
        ax.set_ylabel("Motor speed (rad/s)")
        ax.legend()
        if HAS_PLOT_HELPERS:
            apply_modern_style(ax)
        else:
            ax.grid(True, alpha=0.3)

        ax = axes[2]
        ax.plot(t_h, hydraulic["gen_torque"], label="Generator torque")
        ax.set_ylabel("Torque (N·m)")
        ax.set_xlabel("Time (s)")
        ax.legend()
        if HAS_PLOT_HELPERS:
            apply_modern_style(ax)
        else:
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(str(plot_dir / "hydraulic_internals.png"), dpi=FIGURE_DPI)
        plt.close()

        # Force-vs-velocity scatter: reveals the force-velocity characteristic
        # and whether the smooth regularization is working correctly.
        h_vel = hydraulic["rel_vel"][:min_len]
        h_force = hydraulic["pto_force"][:min_len]

        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

        ax = axes[0]
        sc = ax.scatter(h_vel, h_force / 1e3, c=t_h[:min_len], s=1, alpha=0.5)
        ax.set_xlabel("Relative velocity (m/s)")
        ax.set_ylabel("PTO force (kN)")
        ax.set_title(f"{TEST_NAME}: Force vs Velocity")
        if HAS_PLOT_HELPERS:
            apply_modern_style(ax)
        else:
            ax.grid(True, alpha=0.3)
        plt.colorbar(sc, ax=ax, label="Time (s)")

        ax = axes[1]
        vel_near_zero = np.abs(h_vel) < 0.05
        ax.scatter(h_vel[vel_near_zero], h_force[vel_near_zero] / 1e3,
                   c=t_h[:min_len][vel_near_zero], s=3, alpha=0.7)
        ax.set_xlabel("Relative velocity (m/s)")
        ax.set_ylabel("PTO force (kN)")
        ax.set_title("Zoom: |v| < 0.05 m/s (smoothing region)")
        if HAS_PLOT_HELPERS:
            apply_modern_style(ax)
        else:
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(str(plot_dir / "force_vs_velocity.png"), dpi=FIGURE_DPI)
        plt.close()

        # Force rate of change (dF/dt): highlights any remaining sharp transitions
        dt = np.diff(t_h[:min_len])
        df = np.diff(h_force)
        dfdt = np.where(dt > 0, df / dt, 0.0)

        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(t_h[:min_len - 1], dfdt / 1e6, linewidth=0.5)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("dF/dt (MN/s)")
        ax.set_title(f"{TEST_NAME}: Hydraulic PTO Force Rate of Change")
        if HAS_PLOT_HELPERS:
            apply_modern_style(ax)
        else:
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(str(plot_dir / "force_rate.png"), dpi=FIGURE_DPI)
        plt.close()

    except ImportError:
        pass


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {Path(__file__).name} <results_base_path>")
        sys.exit(1)

    base_dir = Path(sys.argv[1])
    output_dir = base_dir

    linear_path    = base_dir / "linear.csv"
    hydraulic_path = base_dir / "hydraulic.csv"

    if not linear_path.exists():
        print(f"ERROR: Missing {linear_path}")
        sys.exit(1)
    if not hydraulic_path.exists():
        print(f"ERROR: Missing {hydraulic_path}")
        sys.exit(1)

    linear    = load_csv(str(linear_path))
    hydraulic = load_csv(str(hydraulic_path))

    print(f"Loaded {len(linear)} linear records, {len(hydraulic)} hydraulic records\n")

    metrics = compute_scalar_metrics(linear, hydraulic, SKIP_TIME)

    print("=== Scalar Summary (post-transient, t >= {:.0f} s) ===".format(SKIP_TIME))
    print(f"  Linear mean power:          {metrics['linear_mean_power_W']:.1f} W")
    print(f"  Linear peak |force|:        {metrics['linear_peak_force_N']:.1f} N")
    print(f"  Linear RMS displacement:    {metrics['linear_rms_disp_m']:.4f} m")
    print()
    print(f"  Hydraulic mean mech power:  {metrics['hydraulic_mean_mech_power_W']:.1f} W")
    print(f"  Hydraulic mean elec power:  {metrics['hydraulic_mean_elec_power_W']:.1f} W")
    print(f"  Hydraulic peak |force|:     {metrics['hydraulic_peak_force_N']:.1f} N")
    print(f"  Hydraulic RMS displacement: {metrics['hydraulic_rms_disp_m']:.4f} m")
    print(f"  Hydraulic mean motor speed: {metrics['hydraulic_mean_motor_speed_rad_s']:.2f} rad/s")
    print(f"  HP > LP (expected: True):   {metrics['hydraulic_hp_gt_lp']}")
    print(f"  Motor speed >= 0:           {metrics['hydraulic_motor_speed_positive']}")

    # Ratios for human review
    if abs(metrics["linear_mean_power_W"]) > 1.0:
        ratio_power = metrics["hydraulic_mean_mech_power_W"] / metrics["linear_mean_power_W"]
        print(f"\n  Power ratio (hydraulic/linear): {ratio_power:.2f}")
    if abs(metrics["linear_rms_disp_m"]) > 1e-6:
        ratio_disp = metrics["hydraulic_rms_disp_m"] / metrics["linear_rms_disp_m"]
        print(f"  RMS disp ratio (hydraulic/linear): {ratio_disp:.2f}")

    make_plots(linear, hydraulic, output_dir)

    write_comparison_status(str(output_dir), TEST_NAME, metrics, passed=None)
    print(f"\nComparison complete. Status and plots written to {output_dir}")


if __name__ == "__main__":
    main()
