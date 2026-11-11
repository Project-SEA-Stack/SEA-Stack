"""Tests for the demo feature tools in data/demos/run_seastack/features.

h5_reader.py  : HDF5 results plotter (needs h5py, numpy, matplotlib).
MHKiT_writer.py: NDBC / spectrum-file to hydro YAML writer (needs mhkit,
                 ruamel.yaml, PyYAML). Its tests are skipped when MHKiT is
                 not installed; nothing here downloads NDBC data.

Run with pytest, or as a script (CTest): exit code 77 means pytest is
missing and the test is skipped.

Analytic references:
  - Significant wave height Hm0 = 4 * sqrt(m0), m0 = integral S(f) df.
  - Bretschneider (Pierson-Moskowitz) spectrum
        S(f) = (5/16) Hs^2 fp^4 f^-5 exp(-5/4 (fp/f)^4)   [m^2/Hz]
    has m0 = Hs^2 / 16 over 0 < f < inf.
  - Energy-equivalent regular wave height H = Hm0 / sqrt(2).
"""

import importlib.util
import math
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("MPLBACKEND", "Agg")

try:
    import pytest
except ImportError:  # CTest SKIP_RETURN_CODE
    if __name__ == "__main__":
        print("pytest is not installed; skipping test_demo_features")
        sys.exit(77)
    raise

REPO_ROOT = Path(__file__).resolve().parents[3]
FEATURES_DIR = REPO_ROOT / "data" / "demos" / "run_seastack" / "features"
SPHERE_DIR = REPO_ROOT / "data" / "demos" / "run_seastack" / "iea_sphere"


def _import_from(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def bretschneider(freq, hs, tp):
    """Closed-form Bretschneider spectrum S(f) [m^2/Hz]."""
    import numpy as np

    fp = 1.0 / tp
    f = np.asarray(freq, dtype=float)
    return (5.0 / 16.0) * hs**2 * fp**4 * f**-5 * np.exp(-1.25 * (fp / f) ** 4)


# --------------------------------------------------------------------------
# h5_reader.py
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def reader():
    pytest.importorskip("h5py")
    pytest.importorskip("matplotlib")
    return _import_from(FEATURES_DIR / "h5_reader.py", "h5_reader")


@pytest.fixture()
def results_h5(tmp_path):
    """Minimal results file with the layout run_seastack writes."""
    h5py = pytest.importorskip("h5py")
    import numpy as np

    n = 11
    t = np.linspace(0.0, 1.0, n)
    path = tmp_path / "results.test.h5"
    with h5py.File(path, "w") as f:
        f["results/time/time"] = t
        for i, name in enumerate(("body1", "body2", "ground"), start=1):
            pos = np.zeros((n, 3))
            pos[:, 2] = i * t  # heave = i * t
            f[f"results/model/bodies/{name}/position"] = pos
            f[f"results/model/bodies/{name}/velocity"] = np.zeros((n, 3))
        torque = np.column_stack([t, 2 * t, 3 * t])
        f["results/model/joints/hinge/reaction1_force"] = np.zeros((n, 3))
        f["results/model/joints/hinge/reaction1_torque"] = torque
        f["results/model/tsdas/names"] = np.array([b"pto_a", b"pto_b"])
        for k, name in enumerate(("pto_a", "pto_b"), start=1):
            f[f"results/model/tsdas/{name}/absorbed_energy"] = k * t
            f[f"results/model/tsdas/{name}/absorbed_power"] = np.full(n, float(k))
            f[f"results/model/tsdas/{name}/reaction_force_body1"] = np.zeros((n, 3))
    return path


def test_reader_loads_bodies_joints_tsdas(reader, results_h5):
    import numpy as np

    time, data = reader.load_data(str(results_h5), None, ["position", "joint_torque", "energy"], None, None)
    assert time.shape == (11,)
    assert set(k for k in data if k.startswith("body")) == {"body1", "body2"}  # ground excluded
    np.testing.assert_allclose(data["body2"]["position"][:, 2], 2 * time)
    np.testing.assert_allclose(data["joint_hinge"]["joint_torque"][:, 1], 2 * time)
    np.testing.assert_allclose(data["tsda_pto_b"]["energy"], 2 * time)


def test_reader_selects_bodies_and_sums_tsdas(reader, results_h5):
    import numpy as np

    time, data = reader.load_data(str(results_h5), [2], ["position", "energy", "power"], None, ["all"])
    assert "body1" not in data and "body2" in data
    np.testing.assert_allclose(data["TSDA_total"]["energy"], 3 * time)  # 1*t + 2*t
    np.testing.assert_allclose(data["TSDA_total"]["power"], 3.0)


def test_reader_joint_torque_plot_title(reader, results_h5, monkeypatch):
    import matplotlib.pyplot as plt

    monkeypatch.setattr(plt, "show", lambda *a, **k: None)
    plt.close("all")
    time, data = reader.load_data(str(results_h5), None, ["joint_torque"], None, None)
    reader.plot_data(time, data, ["joint_torque"], ["heave"], None)
    ax = plt.gcf().axes[0]
    assert ax.get_title() == "Reaction Torque"
    assert ax.get_ylabel() == "Torque [N m]"
    plt.close("all")


def test_reader_list_outputs(reader, results_h5, capsys):
    reader.list_outputs(str(results_h5))
    out = capsys.readouterr().out
    assert "hinge" in out and "joint_torque" in out
    assert "pto_a" in out and "tsda_force" in out


def test_reader_without_path_gives_clear_error(reader, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["h5_reader.py"])
    with pytest.raises(SystemExit) as exc:
        reader.main()
    assert "path to a results .h5 file" in str(exc.value)


# --------------------------------------------------------------------------
# MHKiT_writer.py
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def writer():
    pytest.importorskip("mhkit")
    pytest.importorskip("ruamel.yaml")
    pytest.importorskip("yaml")
    return _import_from(FEATURES_DIR / "MHKiT_writer.py", "MHKiT_writer")


def test_writer_regular_height_is_energy_equivalent(writer, monkeypatch):
    monkeypatch.setattr(writer, "args", SimpleNamespace(height=None, significant_height=None), raising=False)
    h_reg, _ = writer._resolve_height("regular", 2.0)
    h_irr, _ = writer._resolve_height("irregular", 2.0)
    assert h_reg == pytest.approx(2.0 / math.sqrt(2.0), rel=1e-12)
    assert h_irr == pytest.approx(2.0, rel=1e-12)

    monkeypatch.setattr(writer, "args", SimpleNamespace(height=None, significant_height=3.0), raising=False)
    assert writer._resolve_height("regular", 2.0)[0] == pytest.approx(3.0 / math.sqrt(2.0), rel=1e-12)


@pytest.mark.parametrize("hs,tp", [(1.0, 6.0), (2.5, 9.0), (6.0, 14.0)])
def test_writer_spectra_match_hm0_and_bretschneider(writer, hs, tp):
    import numpy as np
    from scipy import integrate

    f = np.linspace(0.005, 1.0, 4000)
    s_pm = writer.pm_spectrum(f, hs, tp)
    s_js = writer.jonswap_spectrum(f, hs, tp, gamma=3.3)
    for s in (s_pm, s_js):
        assert 4.0 * math.sqrt(integrate.trapezoid(s, f)) == pytest.approx(hs, rel=0.01)
    # PM shape against the closed form (independent of the writer's normalisation)
    s_ref = bretschneider(f, hs, tp)
    assert np.max(np.abs(s_pm - s_ref)) < 0.01 * np.max(s_ref)
    # JONSWAP peak at fp
    assert f[np.argmax(s_js)] == pytest.approx(1.0 / tp, abs=2 * (f[1] - f[0]))


def test_writer_load_spectrum_file_layouts(writer, tmp_path):
    import numpy as np

    freq = np.array([0.30, 0.10, 0.20, 0.40])
    dens = np.array([3.0, 1.0, 2.0, 4.0])
    rows = tmp_path / "rows.txt"
    rows.write_text("# two rows\n" + " ".join(map(str, freq)) + "\n" + ",".join(map(str, dens)) + "\n")
    cols = tmp_path / "cols.txt"
    cols.write_text("freq dens\n" + "\n".join(f"{a} {b}  % c" for a, b in zip(freq, dens)) + "\n")
    for path in (rows, cols):
        f, s = writer.load_spectrum_file(str(path))
        np.testing.assert_allclose(f, [0.1, 0.2, 0.3, 0.4])
        np.testing.assert_allclose(s, [1.0, 2.0, 3.0, 4.0])

    bad = tmp_path / "bad.txt"
    bad.write_text("0.1 0.2 0.3\n1 -2 3\n")
    with pytest.raises(ValueError):
        writer.load_spectrum_file(str(bad))


def test_writer_yaml_round_trip_keeps_comments(writer, tmp_path):
    path = tmp_path / "case.hydro.yaml"
    path.write_text(
        "# header comment\n"
        "hydrodynamics:\n"
        "  bodies:\n"
        "    - name: body1   # inline body comment\n"
        "      h5_file: ../assets/hydroData/sphere.h5\n"
        "  waves:\n"
        "    # waves comment\n"
        "    type: regular\n"
        "    height: 2.0  # crest-to-trough [m]\n"
        "    period: 8.0\n"
    )
    data = writer._load_hydro_yaml(str(path))
    data["hydrodynamics"]["waves"]["height"] = 1.25
    writer._dump_hydro_yaml(data, str(path))
    text = path.read_text()
    for comment in ("# header comment", "# inline body comment", "# waves comment", "# crest-to-trough [m]"):
        assert comment in text
    assert re.search(r"height:\s*1\.25", text)


def _find_run_seastack():
    exe = os.environ.get("SEASTACK_RUN_SEASTACK")
    if exe and Path(exe).is_file():
        return Path(exe)
    for cand in ("build/bin/Release/run_seastack.exe", "build/bin/run_seastack", "build/bin/Release/run_seastack"):
        p = REPO_ROOT / cand
        if p.is_file():
            return p
    return None


def test_writer_spectrum_file_end_to_end(writer, tmp_path):
    """Spectrum file -> eta record + hydro YAML -> short run_seastack run (offline)."""
    import numpy as np
    import yaml

    hs, tp, duration, dt = 2.0, 8.0, 60.0, 0.05

    # Case: sphere regular-wave case with an extra comment, plus its H5 file
    (tmp_path / "assets" / "hydroData").mkdir(parents=True)
    shutil.copy(SPHERE_DIR / "assets" / "hydroData" / "sphere.h5", tmp_path / "assets" / "hydroData")
    case = tmp_path / "case"
    shutil.copytree(SPHERE_DIR / "regular_waves_lin", case, ignore=shutil.ignore_patterns("outputs"))
    hydro = case / "iea_sphere_regular_waves_lin.hydro.yaml"
    hydro.write_text("# Sphere case - comment must survive the writer\n" + hydro.read_text())

    f = np.linspace(0.02, 1.0, 400)
    spec = case / "pm_spectrum.txt"
    spec.write_text("# Bretschneider Hs=2 m Tp=8 s\n" + "\n".join(f"{a:.6f} {b:.8e}" for a, b in zip(f, bretschneider(f, hs, tp))))

    proc = subprocess.run(
        [sys.executable, str(FEATURES_DIR / "MHKiT_writer.py"), str(hydro), "--spectrum_file", spec.name,
         "--no_plot", "--elevation_duration", str(duration), "--elevation_dt", str(dt), "--seed", "3"],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=600)
    assert proc.returncode == 0, proc.stdout[-3000:] + proc.stderr[-3000:]

    # Hm0 of the file as reported by the writer: 4 sqrt(m0), m0 = Hs^2/16 (closed form)
    m = re.search(r"Hm0 = ([0-9.]+) m", proc.stdout)
    assert m, proc.stdout[-2000:]
    assert float(m.group(1)) == pytest.approx(hs, rel=0.01)

    # Hydro YAML points at the eta file and kept its comment
    text = hydro.read_text()
    assert "comment must survive the writer" in text
    waves = yaml.safe_load(text)["hydrodynamics"]["waves"]
    assert waves["type"] == "irregular"
    eta_path = case / waves["eta_file"]
    assert eta_path.is_file()

    # Eta record: time:elevation, covers the duration, realised Hm0 near the target
    rec = np.array([[float(x) for x in line.split(":")] for line in eta_path.read_text().split()])
    assert rec[0, 0] == pytest.approx(0.0, abs=1e-9)
    assert rec[-1, 0] >= duration - dt
    assert 4.0 * np.std(rec[:, 1]) == pytest.approx(hs, rel=0.15)

    sim = yaml.safe_load(next(case.glob("*.simulation.yaml")).read_text())["simulation"]
    assert float(sim["end_time"]) == pytest.approx(duration)

    exe = _find_run_seastack()
    if exe is None:
        pytest.skip("run_seastack not found (set SEASTACK_RUN_SEASTACK); YAML checks passed")
    run = subprocess.run([str(exe), "--nogui", "--nobanner", "--quiet", str(case)],
                         capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=600)
    assert run.returncode == 0, run.stdout[-3000:] + run.stderr[-3000:]
    assert list((case / "outputs").glob("results*.h5")), "run_seastack wrote no results file"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q", "-rs"]))
