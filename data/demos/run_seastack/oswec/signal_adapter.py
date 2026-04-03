"""Signal adapter for OSWEC YAML demos (flap pitch preferred, heave fallback)."""
from pathlib import Path
import numpy as np
import h5py


def _read_time(f):
    for key in ["/results/time/time", "/results/time", "/time"]:
        if key in f:
            return np.asarray(f[key][:], dtype=float).reshape(-1)
    raise KeyError("time vector not found")


def _flap_pitch_or_heave(f):
    if "/results/model/bodies/body1/orientation_xyz" in f:
        arr = np.asarray(f["/results/model/bodies/body1/orientation_xyz"][:])
        if arr.ndim == 2 and arr.shape[1] >= 2:
            return arr[:, 1], "Pitch (rad)"
    if "/results/model/bodies/body1/orientation" in f:
        q = np.asarray(f["/results/model/bodies/body1/orientation"][:])
        if q.ndim == 2 and q.shape[1] == 4:
            w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
            r20 = 2 * (x * z - w * y)
            r00 = 1 - 2 * (y * y + z * z)
            r10 = 2 * (x * y + w * z)
            theta_y = np.arctan2(-r20, np.hypot(r00, r10))
            return theta_y, "Pitch (rad)"
    for key in [
        "/results/model/bodies/body1/position",
        "/results/bodies/body1/position",
    ]:
        if key in f:
            arr = np.asarray(f[key][:])
            if arr.ndim == 2 and arr.shape[1] >= 3:
                return arr[:, 2], "Heave (m)"
            if arr.ndim == 1:
                return arr, "Heave (m)"
    raise KeyError("OSWEC: no suitable signal found")


def select_signal(h5_path: Path):
    with h5py.File(h5_path, "r") as f:
        t = _read_time(f)
        y, label = _flap_pitch_or_heave(f)
        return t, y, label


def select_signals(h5_path: Path):
    """Primary flap pitch (or heave fallback) plus float heave when orientation is available."""
    with h5py.File(h5_path, "r") as f:
        t = _read_time(f)
        y_primary, primary_label = _flap_pitch_or_heave(f)
        out = {"flap_primary": (t, y_primary, primary_label)}
        key = "/results/model/bodies/body1/position"
        if key in f:
            arr = np.asarray(f[key][:])
            if arr.ndim == 2 and arr.shape[1] >= 3:
                out["float_heave"] = (t, arr[:, 2], "Float heave (m)")
        return out
