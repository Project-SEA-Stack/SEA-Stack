"""Signal adapter for RM3 demo cases (decay and mooring)."""
from pathlib import Path
import numpy as np
import h5py


def _read_time(f):
	for key in ["/results/time/time", "/results/time", "/time"]:
		if key in f:
			return np.asarray(f[key][:], dtype=float).reshape(-1)
	raise KeyError("time vector not found")


def _body_dof(f, name_candidates, col, label):
	"""Read a single DOF column from body position data."""
	for name in name_candidates:
		for key in [
			f"/results/model/bodies/{name}/position",
			f"/results/bodies/{name}/position",
		]:
			if key in f:
				arr = np.asarray(f[key][:])
				if arr.ndim == 2 and arr.shape[1] > col:
					return arr[:, col]
				if arr.ndim == 1 and col == 0:
					return arr
	raise KeyError(f"RM3: DOF col {col} ({label}) not found for: {name_candidates}")


def _is_mooring_case(h5_path: Path) -> bool:
	return "mooring" in h5_path.parts


def _load_fairlead_tension(h5_path: Path, col=1):
	"""Load a fairlead tension column from MoorDyn's lines_rm3.out.

	The .out file sits in the mooring/ subdirectory next to the MoorDyn
	input file.  The H5 lives under outputs/, so we search both the H5's
	parent and one level up (the case directory).
	Column 0 is time, columns 1-3 are FairTen4/5/6.
	"""
	candidates = [
		h5_path.parent.parent / "mooring" / "lines_rm3.out",
		h5_path.parent / "mooring" / "lines_rm3.out",
	]
	moordyn_out = next((p for p in candidates if p.exists()), None)
	if moordyn_out is None:
		return None, None
	data = np.loadtxt(str(moordyn_out), skiprows=1)
	t = data[:, 0]
	_, unique_idx = np.unique(t, return_index=True)
	return data[unique_idx, 0], data[unique_idx, col]


def select_signal(h5_path: Path):
	with h5py.File(h5_path, "r") as f:
		t = _read_time(f)
		if _is_mooring_case(h5_path):
			return t, _body_dof(f, ["body1", "float"], 0, "Surge"), "Surge (m)"
		return t, _body_dof(f, ["body2", "plate", "body1", "float"], 2, "Heave"), "Heave (m)"


def select_signals(h5_path: Path):
	with h5py.File(h5_path, "r") as f:
		t = _read_time(f)

		if _is_mooring_case(h5_path):
			signals = {
				"float_surge": (t, _body_dof(f, ["body1", "float"], 0, "Surge"), "Surge (m)"),
			}
			t_ft, ft4 = _load_fairlead_tension(h5_path, col=1)
			if t_ft is not None:
				signals["fairlead_tension_4"] = (t_ft, ft4, "FairTen4 (N)")
			return signals

		return {
			"float": (t, _body_dof(f, ["body1", "float"], 2, "Heave"), "Heave (m)"),
			"plate": (t, _body_dof(f, ["body2", "plate"], 2, "Heave"), "Heave (m)"),
		}


