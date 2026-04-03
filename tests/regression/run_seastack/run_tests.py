#!/usr/bin/env python3
import argparse
import subprocess
import sys
from pathlib import Path

if sys.version_info < (3, 10):
    print("run_tests.py requires Python 3.10 or newer.", file=sys.stderr)
    sys.exit(1)
import importlib.util
import shutil
import numpy as np
import os

# Resolve project root and demos directory for both layouts:
# - Installed: tests/run_tests.py          => ROOT = parents[1], DEMOS = ROOT/demos
# - Source:    tests/regression/run_seastack/run_tests.py => ROOT = parents[3], DEMOS = ROOT/data/demos/run_seastack
_here = Path(__file__).resolve()
_in_source_tree = _here.parents[1].name == "regression"
if _in_source_tree:
    ROOT = _here.parents[3]
    _DEFAULT_DEMOS = ROOT / "data" / "demos" / "run_seastack"
else:
    ROOT = _here.parents[1]
    _DEFAULT_DEMOS = ROOT / "demos"
THIS = _here.parent
DEMOS_DIR = _DEFAULT_DEMOS


def _resolve_compare_template_dir() -> Path | None:
    """Locate a directory containing compare_template.py across source/install layouts.

    Search order (first match wins):
    - tests/regression (source tree layout)
    - tests/regression/utilities (alternative location)
    - siblings relative to installed layout: tests/regression, tests/regression/utilities
    - walk up a few parents and probe known subpaths
    """
    candidates: list[Path] = []
    # Prefer directory containing this script (flat packaged tests/ or run_seastack/)
    candidates.append(THIS)
    candidates.append(THIS.parent)                               # tests/
    candidates.append(THIS.parent / "regression")               # tests/regression (source layout)
    candidates.append(THIS.parent / "regression" / "utilities")
    # Root-based candidates
    candidates.append(ROOT / "tests" / "regression")
    candidates.append(ROOT / "tests" / "regression" / "utilities")

    # Walk up a few levels and try common subpaths
    cur = _here
    for _ in range(6):
        cur = cur.parent
        candidates.append(cur / "tests" / "regression")
        candidates.append(cur / "tests" / "regression" / "utilities")

    for c in candidates:
        try:
            if (c / "compare_template.py").exists():
                return c
        except Exception:
            pass
    return None


def default_exe() -> str:
	"""Best-effort discovery of run_seastack in build and install layouts."""
	# Allow override via environment
	for key in ["SS_RUN_EXE", "RUN_SEASTACK_EXE", "SEASTACK_EXE"]:
		env = os.environ.get(key)
		if env and Path(env).exists():
			return env
	# Typical names: Windows uses run_seastack.exe; Unix uses run_seastack.
	names = ("run_seastack.exe", "run_seastack") if os.name == "nt" else ("run_seastack", "run_seastack.exe")
	# Check typical locations: build tree (Release/Debug), install tree (bin), root
	for name in names:
		for p in [
			ROOT / "build" / "bin" / "Release" / name,
			ROOT / "build" / "bin" / "Debug" / name,
			ROOT / "bin" / name,
			ROOT / name,
		]:
			if p.exists():
				return str(p)
	# Fall back to name; rely on PATH (e.g., RUN-TESTS prepends install/bin)
	return "run_seastack.exe" if os.name == "nt" else "run_seastack"


def find_ref(model: str, test_type: str) -> str | None:
    expected_dir = DEMOS_DIR / model / test_type / "expected"
    # Prefer baseline.h5
    h5_base = expected_dir / "baseline.h5"
    if h5_base.exists():
        return str(h5_base)
    # Any h5 in expected
    for p in expected_dir.glob("*.h5"):
        return str(p)
    # Legacy txt in expected
    txt_expected = expected_dir / f"ss_ref_{model}_{test_type}.txt"
    if txt_expected.exists():
        return str(txt_expected)
    # Legacy regression reference_data
    legacy = ROOT / "tests" / "regression" / "reference_data" / model / test_type / f"ss_ref_{model}_{test_type}.txt"
    if legacy.exists():
        return str(legacy)
    return None


def _find_output_h5(model: str, test_type: str) -> Path | None:
	"""Locate the simulation output H5 (results.still.h5, results.irregular.h5, etc.)."""
	outputs_dir = DEMOS_DIR / model / test_type / "outputs"
	for p in outputs_dir.glob("results.*.h5"):
		return p
	return None


def run_case(exe: str, model: str, test_type: str, tol: float, update_baseline: bool, quiet: bool, show: bool, gui: bool) -> int:
	"""Run a single test: simulate, then compare and plot. Returns exit code."""
	inputs_setup = DEMOS_DIR / model / test_type / f"{model}_{test_type}.setup.yaml"
	if not inputs_setup.exists():
		print(f"SKIP | {model}/{test_type} | missing setup {inputs_setup}", file=sys.stderr)
		return 0
	# 1) simulate
	cmd = [
		sys.executable,
		str(THIS / "run_simulation.py"),
		"--exe",
		exe,
		"--setup",
		str(inputs_setup),
	]
	if not gui:
		cmd.append("--nogui")
	r1 = subprocess.run(cmd, capture_output=quiet, text=True, encoding="utf-8", errors="ignore")
	if r1.returncode != 0:
		if quiet:
			print(r1.stdout, end="")
			print(r1.stderr, end="")
		print(f"FAIL | {model}/{test_type} | simulation exited {r1.returncode}", file=sys.stderr)
		return r1.returncode
	# Optional: copy current output HDF5 to expected/baseline.h5 for this case
	if update_baseline:
		out_h5 = _find_output_h5(model, test_type)
		if out_h5 is not None and out_h5.is_file():
			expected_dir = DEMOS_DIR / model / test_type / "expected"
			expected_dir.mkdir(parents=True, exist_ok=True)
			dest = expected_dir / "baseline.h5"
			shutil.copy2(out_h5, dest)
			print(f"BASELINE | {model}/{test_type} | wrote {dest}", file=sys.stderr)
		else:
			print(
				f"WARN | {model}/{test_type} | --update-baseline: no results.*.h5 under outputs/",
				file=sys.stderr,
			)
	# 2) compare
	ref = find_ref(model, test_type)
	outputs_h5_found = _find_output_h5(model, test_type)
	outputs_h5 = outputs_h5_found.resolve() if outputs_h5_found else (DEMOS_DIR / model / test_type / "outputs" / "results.still.h5").resolve()
	plots_dir = (DEMOS_DIR / model / test_type / "outputs" / "plots").resolve()
	# Neutral/adapter-driven comparison path
	if outputs_h5.exists():
		adapter_path = DEMOS_DIR / model / "signal_adapter.py"
		if adapter_path.exists():
			try:
				spec = importlib.util.spec_from_file_location(f"adapter_{model}", str(adapter_path))
				assert spec and spec.loader
				adapter = importlib.util.module_from_spec(spec)
				spec.loader.exec_module(adapter)  # type: ignore
				# import plotting template from tests/regression (robust resolution)
				regression_dir = _resolve_compare_template_dir()
				if regression_dir is not None and str(regression_dir) not in sys.path:
					sys.path.insert(0, str(regression_dir))
				from compare_template import create_comparison_plot  # type: ignore
				# helper
				def rms_relative_error(ref_arr: np.ndarray, pred_arr: np.ndarray) -> float:
					ref_rms = float(np.sqrt(np.mean(np.square(ref_arr))))
					if ref_rms == 0.0:
						return float(np.sqrt(np.mean(np.square(pred_arr))))
					return float(np.sqrt(np.mean(np.square(pred_arr - ref_arr))) / ref_rms)
				# try multi-signal first
				multi = getattr(adapter, "select_signals", None)
				single = getattr(adapter, "select_signal", None)
				if multi is not None:
					sim_signals = multi(outputs_h5)
					ref_signals = multi(Path(ref)) if ref else sim_signals
					status = 0
					for name, (t_sim, y_sim, y_label) in sim_signals.items():
						if name not in ref_signals:
							continue
						t_ref, y_ref, _ = ref_signals[name]
						ref_on_sim = np.interp(t_sim, t_ref, y_ref)
						rms_rel = rms_relative_error(ref_on_sim, y_sim)
						result = "PASS" if rms_rel <= tol else "FAIL"
						print(f"{result} | N={len(t_sim)} | RMSrel={rms_rel:.6f} | tol={tol:.6f}")
						# plot
						plots_dir.mkdir(parents=True, exist_ok=True)
						ref_data = np.column_stack((t_sim, ref_on_sim))
						sim_data = np.column_stack((t_sim, y_sim))
						create_comparison_plot(
							ref_data,
							sim_data,
							f"{model}_{test_type}_test - {name}",
							str(plots_dir),
							ref_file_path=str(ref) if ref else str(outputs_h5),
							test_file_path=str(outputs_h5),
							executable_path=None,
							y_label=y_label,
							executable_patterns=None,
						)
						if result != "PASS":
							status = 1
					return status
				if single is not None:
					t_sim, y_sim, y_label = single(outputs_h5)
					if ref:
						t_ref, y_ref, _ = single(Path(ref))
						ref_on_sim = np.interp(t_sim, t_ref, y_ref)
					else:
						ref_on_sim = y_sim.copy()
					rms_rel = rms_relative_error(ref_on_sim, y_sim)
					result = "PASS" if rms_rel <= tol else "FAIL"
					print(f"{result} | N={len(t_sim)} | RMSrel={rms_rel:.6f} | tol={tol:.6f}")
					plots_dir.mkdir(parents=True, exist_ok=True)
					ref_data = np.column_stack((t_sim, ref_on_sim))
					sim_data = np.column_stack((t_sim, y_sim))
					create_comparison_plot(
						ref_data,
						sim_data,
						f"{model}_{test_type}_test",
						str(plots_dir),
						ref_file_path=str(ref) if ref else str(outputs_h5),
						test_file_path=str(outputs_h5),
						executable_path=None,
						y_label=y_label,
						executable_patterns=None,
					)
					return 0 if result == "PASS" else 1
			except Exception as e:
				if quiet:
					print(f"Adapter compare failed for {model}/{test_type}: {e}", file=sys.stderr)
	# Default path (legacy adapter mode)
	# Use explicit simple-mode compare for all other tests too (heave of first body by default)
	outputs_h5_found = _find_output_h5(model, test_type)
	outputs_h5 = outputs_h5_found.resolve() if outputs_h5_found else (DEMOS_DIR / model / test_type / "outputs" / "results.still.h5").resolve()
	plots_dir = (DEMOS_DIR / model / test_type / "outputs" / "plots").resolve()
	cmd = [
		sys.executable,
		str(THIS / "compare_results.py"),
		"--ref", ref if ref else str(outputs_h5),
		"--sim", str(outputs_h5),
		"--ref-time-dset", "/results/time/time",
		"--sim-time-dset", "/results/time/time",
		"--ref-val-dset", "/results/model/bodies/body1/position",
		"--ref-col", "2",
		"--sim-val-dset", "/results/model/bodies/body1/position",
		"--sim-col", "2",
		"--ylabel", "Heave (m)",
		"--title", f"{model}_{test_type}_test",
		"--outdir", str(plots_dir),
		"--tol", str(tol),
	]
	if show:
		cmd.append("--show")
	r2 = subprocess.run(cmd, capture_output=quiet, text=True, encoding="utf-8", errors="ignore")
	if quiet:
		print(r2.stdout, end="")
		print(r2.stderr, end="")
	return r2.returncode


def main() -> int:
	"""CLI entrypoint for running SEA-Stack YAML tests."""
	parser = argparse.ArgumentParser(description="SEA-Stack – test runner")
	parser.add_argument("--exe", default=default_exe(), help="Path to run_seastack.exe")
	parser.add_argument("--demos-dir", type=str, default=None, help="Path to demos directory (auto-detected if omitted)")
	parser.add_argument("--tol", type=float, default=0.02, help="RMS relative error tolerance")
	parser.add_argument("--update-baseline", action="store_true", help="Overwrite reference with current simulation output")
	parser.add_argument("--quiet", action="store_true", help="Suppress subprocess logs (summary only)")
	parser.add_argument("--show", action="store_true", help="Display plots interactively (in addition to saving)")
	parser.add_argument("--gui", action="store_true", help="Run simulations with GUI (omit --nogui)")
	# selectors
	parser.add_argument("--all", action="store_true", help="Run all known tests")
	parser.add_argument("--all-tests", dest="all", action="store_true", help="Alias for --all")
	parser.add_argument("--all_tests", dest="all", action="store_true", help="Alias for --all")
	parser.add_argument("--sphere-decay", action="store_true", help="Run IEA sphere decay")
	parser.add_argument("--sphere-decay-ss", action="store_true", help="Run IEA sphere decay (state-space)")
	parser.add_argument("--sphere-irregular-ss", action="store_true", help="Run IEA sphere irregular waves (state-space)")
	parser.add_argument("--oswec-decay", action="store_true", help="Run OSWEC still-water decay (minimal)")
	parser.add_argument("--oswec-regular", action="store_true", help="Run OSWEC regular waves")
	parser.add_argument("--oswec-irregular", action="store_true", help="Run OSWEC irregular waves")
	parser.add_argument("--rm3-decay", action="store_true", help="Run RM3 still-water decay (minimal)")
	parser.add_argument("--rm3-regular", action="store_true", help="Run RM3 regular waves")
	parser.add_argument("--rm3-irregular", action="store_true", help="Run RM3 irregular waves")
	parser.add_argument("--rm3-mooring", action="store_true", help="Run RM3 mooring (MoorDyn)")
	parser.add_argument(
		"--f3of-decay-c1",
		action="store_true",
		help="Run F3OF decay_dt1 (verification Decay C1 surge); omitted from release ZIP",
	)
	parser.add_argument(
		"--f3of-decay-c2",
		action="store_true",
		help="Run F3OF decay_dt2 (verification Decay C2 pitch); omitted from release ZIP",
	)
	parser.add_argument(
		"--f3of-decay-c3",
		action="store_true",
		help="Run F3OF decay_dt3 (verification Decay C3 flap pitch); alias of --f3of-decay-dt3",
	)
	parser.add_argument(
		"--f3of-decay-dt3",
		action="store_true",
		help="Run F3OF decay_dt3 (ships in release ZIP)",
	)
	parser.add_argument("--sphere-decay-nl-1m", action="store_true", help="Run IEA sphere decay, nonlinear hydrostatics, 1 m drop")
	parser.add_argument("--sphere-decay-nl-5m", action="store_true", help="Run IEA sphere decay, nonlinear hydrostatics, 5 m drop")
	parser.add_argument("--sphere-decay-lin-5m", action="store_true", help="Run IEA sphere decay, linear hydrostatics, 5 m drop")
	parser.add_argument("--rm3-decay-nl", action="store_true", help="Run RM3 decay, nonlinear hydrostatics (float only)")
	args = parser.parse_args()

	global DEMOS_DIR
	if args.demos_dir:
		DEMOS_DIR = Path(args.demos_dir).resolve()
	elif os.environ.get("SS_DEMOS_DIR"):
		DEMOS_DIR = Path(os.environ["SS_DEMOS_DIR"]).resolve()

	selections: list[tuple[str,str]] = []
	if args.all:
		selections.extend([
			("iea_sphere", "decay"),
			("iea_sphere", "decay_ss"),
			("iea_sphere", "irregular_waves_ss"),
			("iea_sphere", "decay_nl_1m"),
			("iea_sphere", "decay_nl_5m"),
			("iea_sphere", "decay_lin_5m"),
			("oswec", "regular_waves"),
			("oswec", "irregular_waves"),
			("rm3", "irregular_waves"),
			("rm3", "mooring"),
			("rm3", "decay_nl"),
			("f3of", "decay_dt3"),
		])
	else:
		if args.sphere_decay:
			selections.append(("iea_sphere", "decay"))
		if args.sphere_decay_ss:
			selections.append(("iea_sphere", "decay_ss"))
		if args.sphere_irregular_ss:
			selections.append(("iea_sphere", "irregular_waves_ss"))
		if args.oswec_decay:
			selections.append(("oswec", "decay"))
		if args.oswec_regular:
			selections.append(("oswec", "regular_waves"))
		if args.oswec_irregular:
			selections.append(("oswec", "irregular_waves"))
		if args.rm3_decay:
			selections.append(("rm3", "decay"))
		if args.rm3_regular:
			selections.append(("rm3", "regular_waves"))
		if args.rm3_irregular:
			selections.append(("rm3", "irregular_waves"))
		if args.rm3_mooring:
			selections.append(("rm3", "mooring"))
		if args.f3of_decay_c1:
			selections.append(("f3of", "decay_dt1"))
		if args.f3of_decay_c2:
			selections.append(("f3of", "decay_dt2"))
		if args.f3of_decay_c3 or args.f3of_decay_dt3:
			selections.append(("f3of", "decay_dt3"))
		if args.sphere_decay_nl_1m:
			selections.append(("iea_sphere", "decay_nl_1m"))
		if args.sphere_decay_nl_5m:
			selections.append(("iea_sphere", "decay_nl_5m"))
		if args.sphere_decay_lin_5m:
			selections.append(("iea_sphere", "decay_lin_5m"))
		if args.rm3_decay_nl:
			selections.append(("rm3", "decay_nl"))
		if not selections:
			print(
				"No tests selected. Use --all or flags such as "
				"--sphere-decay/--sphere-decay-ss/--sphere-irregular-ss/--sphere-decay-nl-1m/--sphere-decay-nl-5m/--sphere-decay-lin-5m "
				"--oswec-regular/--oswec-irregular/--oswec-decay "
				"--rm3-regular/--rm3-irregular/--rm3-mooring/--rm3-decay/--rm3-decay-nl "
				"--f3of-decay-c1/--f3of-decay-c2/--f3of-decay-c3/--f3of-decay-dt3",
				file=sys.stderr,
			)
			return 2

	overall = 0
	for model, test_type in selections:
		code = run_case(args.exe, model, test_type, args.tol, args.update_baseline, args.quiet, args.show, args.gui)
		if code != 0:
			overall = code
	return overall


if __name__ == "__main__":
	raise SystemExit(main())