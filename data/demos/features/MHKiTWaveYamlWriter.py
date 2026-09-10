
import os
import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import yaml
from scipy import integrate
from scipy.optimize import minimize, minimize_scalar
from mhkit import wave
from mhkit.wave.io import ndbc

# Redirected stdout defaults to cp1252 on Windows, which cannot encode the gamma
# and superscript characters used in the printed diagnostics.
sys.stdout.reconfigure(encoding="utf-8", errors="replace")


#~~~~~~~~~~~~~~ Command-line Arguments ~~~~~~~~~~~~~~

parser = argparse.ArgumentParser(
    description="Update YAML hydro file with NDBC buoy wave data"
)
parser.add_argument(
    "path", nargs="?", default=None,
    help="Path to the YAML hydro file to update (e.g., 5sa/regular_waves/5sa_regular.hydro.yaml)"
)
parser.add_argument(
    "--buoy",
    type=str,
    default=None,
    help="NDBC buoy number (e.g., 46050). Required unless --elevation_file points at an existing eta file."
)
parser.add_argument(
    "--type",
    type=str,
    default="regular",
    choices=["regular", "irregular"],
    help="Wave type: 'regular' (default) or 'irregular'"
)
parser.add_argument(
    "--year",
    nargs="*",
    default=None,
    help="Year(s) to use for wave statistics. Options: no args (latest year), from:YYYY (years >= YYYY), YYYY (single year), YYYY YYYY... (multiple years). Default: 2025 or latest available."
)
parser.add_argument(
    "--plot_spectrum",
    nargs="*",
    default=None,
    help="Plot wave spectrum. Optional: specify a year for monthly spectra (e.g., --plot_spectrum 2020), or one or more months to overlay (e.g., --plot_spectrum jan feb mar), or omit for the --year flag"
)
parser.add_argument(
    "--plot_heatmap",
    nargs="*",
    default=None,
    help="Plot sea state frequency distribution (Hm0 vs Te) for the selection. Optional: a year (e.g., --plot_heatmap 2020) or one or more months (e.g., --plot_heatmap jan feb), or omit to use --year / --date."
)
parser.add_argument(
    "--plot_wavestats",
    nargs="*",
    default=None,
    help="Plot statistics (median + IQR) of Hm0, Te, Tp, energy flux J and steepness Sm. The resolution follows the selection: hourly for a day (--date), daily for up to ~2 months, monthly for longer. Optional: a year (e.g., --plot_wavestats 2020) or one or more months (e.g., --plot_wavestats jan feb), or omit to use --year / --date."
)
parser.add_argument(
    "--plot_wavedirection",
    nargs="?",
    const="elevation",
    default=None,
    choices=["elevation", "energy", "spread"],
    help="Plot directional wave spectrum (elevation/energy/spread). Default: elevation"
)
parser.add_argument(
    "--spectrum",
    type=str,
    default=None,
    choices=["pm", "pierson_moskowitz", "jonswap", "js", "custom"],
    help="Spectrum type for irregular waves (pm, pierson_moskowitz, jonswap, js, custom). Only used with --type irregular. 'custom' generates a surface elevation time series from the measured buoy spectrum and writes an eta file next to the YAML."
)
parser.add_argument(
    "--spectrum_file",
    type=str,
    default=None,
    help="Read the custom spectrum from a text file instead of downloading it from a buoy, so --buoy is not needed. Accepts a bare name, a path relative to the YAML directory, or an absolute path. Two layouts are recognised: two rows (frequencies then spectral densities) or two columns (frequency, density) - blank lines and lines starting with # or %% are ignored. Units are Hz and m^2/Hz. Implies --type irregular --spectrum custom."
)
parser.add_argument(
    "--elevation_duration",
    type=float,
    default=None,
    help="Simulation duration in seconds, written to simulation.end_time for any spectrum type. With --spectrum custom it also sets the length of the generated elevation record. Default: 600 (10 minutes) for the eta record; the simulation YAML is left alone unless this flag is given. Ignored when --elevation_file points at an existing record."
)
parser.add_argument(
    "--elevation_dt",
    type=float,
    default=None,
    help="Solver time step in seconds, written to simulation.time_step for any spectrum type. With --spectrum custom it also sets the sample rate of the generated elevation record. Default: the existing simulation.time_step (falls back to 0.05 if no simulation YAML is found), so the record is sampled at the solver step."
)
parser.add_argument(
    "--elevation_nfreq",
    type=int,
    default=512,
    help="[deprecated] Ignored. The spectrum grid is fixed by the record: df = 1/(duration+dt), up to the Nyquist frequency, because that is the grid surface_elevation's ifft assumes."
)
parser.add_argument(
    "--elevation_file",
    type=str,
    default=None,
    help="[--spectrum custom] Eta file: a bare name, a path relative to the YAML directory, or an absolute path. If the file ALREADY EXISTS it is used as-is - no buoy data is downloaded, no record is generated and the file is not rewritten. Otherwise it is the name of the eta file written next to the YAML. Default: <yaml stem>_eta.txt"
)
parser.add_argument(
    "--eta_method",
    type=str,
    default="dft",
    choices=["dft", "irf"],
    help="[--spectrum custom] How sea-stack consumes the eta file. 'dft' (default) extracts discrete wave components, giving a spatial free surface so the GUI wireframe renders. 'irf' uses EtaTableWaveField, a point time series with no spatial variation - the body moves but no wave surface is drawn."
)
parser.add_argument(
    "--seed",
    type=int,
    default=None,
    help="Random seed for wave generation (optional)"
)
parser.add_argument(
    "--ramp_time",
    type=float,
    default=None,
    help="Ramp time in seconds for wave generation (optional)"
)
parser.add_argument(
    "--height",
    type=float,
    default=None,
    help="Override calculated wave height in meters (optional)"
)
parser.add_argument(
    "--period",
    type=float,
    default=None,
    help="Override calculated wave period in seconds (optional)"
)
parser.add_argument(
    "--depth",
    type=float,
    default=None,
    help="Override calculated water depth in meters (optional)"
)
parser.add_argument(
    "--nofill",
    action="store_true",
    help="Remove fill from directional spectrum plot (show contours only)"
)
parser.add_argument(
    "--plot_spectrum_pm", "--plot_spectrum_PM",
    dest="plot_spectrum_pm",
    nargs="*",
    default=None,
    help="Plot wave spectrum against Pierson-Moskowitz estimate. Optional: a year (e.g., --plot_spectrum_pm 2020) or one or more months to overlay (e.g., --plot_spectrum_pm jan feb), or omit to use --year flag"
)
parser.add_argument(
    "--plot_spectrum_js", "--plot_spectrum_JS",
    "--plot_spectrum_jonswap", "--plot_spectrum_JONSWAP",
    dest="plot_spectrum_js",
    nargs="*",
    default=None,
    help="Plot wave spectrum against fitted JONSWAP spectrum. Optional: a year (e.g., --plot_spectrum_js 2020) or one or more months to overlay (e.g., --plot_spectrum_js jan feb), or omit to use --year flag"
)
parser.add_argument(
    "--gamma",
    nargs="*",
    type=float,
    action='append',
    help="Manual gamma (peak enhancement factor) values to plot alongside fitted JONSWAP. Can be used as: --gamma 1.0 1.5 or --gamma 1.0 --gamma 1.5 (e.g., --gamma 0.5 1.0 2.5)"
)
parser.add_argument(
    "--plot_wind",
    nargs="*",
    default=None,
    help="Plot wind speed, gusts and direction. With --date: daily 10-minute interval time series. Otherwise median + IQR at a resolution that follows the selection: daily for up to ~2 months, monthly for longer. Optional: a year (e.g., --plot_wind 2020) or one or more months (e.g., --plot_wind jan feb). Does not update the YAML file."
)
parser.add_argument(
    "--date",
    nargs="*",
    default=None,
    help="Extract data for one or more dates. Format: YYYY-MM-DD (e.g., 2024-06-15), MM-DD-YYYY (e.g., 06-15-2024), or MM-DD with --year (e.g., --date 06-15 --year 2024). Multiple dates are overlaid on one plot (e.g., --date 01-01 06-15). Without --time, each date is a daily average. With --time, each date/time combination is a single measurement."
)
parser.add_argument(
    "--time",
    nargs="*",
    default=None,
    help="[Optional] One or more times for single measurement extraction. Format: HH:MM in 24-hour format (e.g., 18:30), or a range HH:MM-HH:MM (e.g., 05:20-16:40) which averages every measurement in that window. 24:00 means end of day. Multiple times or ranges are overlaid (e.g., --time 00:00-12:00 12:00-24:00). Each date is combined with each time. NDBC wave data is sampled at :10 and :40 past each hour. If omitted with --date, daily averages are used."
)

args = parser.parse_args()

user_set_spectrum = args.spectrum is not None
user_set_type = args.type != "regular"

if args.spectrum_file:
    if args.spectrum is None:
        args.spectrum = "custom"
    elif args.spectrum != "custom":
        parser.error("--spectrum_file requires --spectrum custom")
    if args.type != "irregular":
        args.type = "irregular"
    if args.date or args.time:
        parser.error("--date / --time select a buoy measurement and cannot be used with --spectrum_file")
    if args.plot_wind is not None:
        parser.error("wind plots need buoy data and cannot be used with --spectrum_file")

if args.buoy is None and not (args.spectrum == "custom" and (args.elevation_file or args.spectrum_file)):
    parser.error("--buoy is required, unless --spectrum custom is reading an existing "
                 "--elevation_file or a --spectrum_file")

# Flatten gamma list if it was provided (handles both --gamma 1.0 1.5 and --gamma 1.0 --gamma 1.5)
if args.gamma:
    args.gamma = [g for sublist in args.gamma for g in sublist] if any(isinstance(i, list) for i in args.gamma) else args.gamma
else:
    args.gamma = []

# --date alone gives a daily average per date; --date + --time gives a single raw
# measurement per date/time pair. Multiple values form a cross product, overlaid.
single_measurement_date = None
daily_start = None
daily_end = None
extract_single = False
extract_daily = False
all_wind_data = None
extract_targets = []  # list of dicts: {'kind': 'single'|'daily', 'timestamp': ..., 'label': ...}

if args.date is not None and len(args.date) > 0:
    def _parse_one_date(date_str):
        """Parse YYYY-MM-DD, MM-DD-YYYY or MM-DD (with --year) into (year, month, day)."""
        parts = date_str.split('-')
        y = m = d = None

        if len(parts) == 3:
            try:
                a, b, c = (int(p) for p in parts)
            except ValueError:
                parser.error(f"Invalid date format: {date_str}. Use YYYY-MM-DD, MM-DD-YYYY or MM-DD")
            # The 4-digit field tells the two orderings apart
            if len(parts[0]) == 4:
                y, m, d = a, b, c
            elif len(parts[2]) == 4:
                y, m, d = c, a, b
            else:
                parser.error(f"Invalid date format: {date_str}. The year must be 4 digits, "
                             f"as YYYY-MM-DD or MM-DD-YYYY")
        elif len(parts) == 2:
            if args.year is None or len(args.year) == 0:
                parser.error("--date in MM-DD format requires --year to be specified")
            try:
                m, d = map(int, parts)
                y = int(args.year[0]) if isinstance(args.year[0], str) else args.year[0]
            except (ValueError, IndexError):
                parser.error(f"Invalid date or year format: {date_str} with year {args.year}")
        else:
            parser.error(f"Invalid date format: {date_str}. Use YYYY-MM-DD, MM-DD-YYYY or MM-DD")

        try:
            pd.Timestamp(year=y, month=m, day=d)
        except ValueError as e:
            parser.error(f"Invalid date: {date_str} ({e})")
        return y, m, d

    def _parse_one_time(time_str):
        """Parse HH:MM into (hour, minute); 24:00 is accepted as end of day."""
        try:
            h, mi = map(int, time_str.split(':'))
        except ValueError:
            parser.error(f"Invalid time format: {time_str}. Use HH:MM (24-hour format)")
        if (h, mi) == (24, 0):
            return 23, 59
        if not (0 <= h < 24 and 0 <= mi < 60):
            parser.error(f"Invalid time format: {time_str}. Use HH:MM (24-hour format), "
                         f"or 24:00 for end of day")
        return h, mi

    def _parse_time_token(token):
        """Parse HH:MM or HH:MM-HH:MM into ('point', (h, mi)) or ('range', start, end)."""
        if '-' in token:
            lo, hi = token.split('-', 1)
            start, end = _parse_one_time(lo), _parse_one_time(hi)
            if end <= start:
                parser.error(f"Invalid time range: {token}. The end time must be after the start")
            return ('range', start, end)
        return ('point', _parse_one_time(token))

    parsed_dates = [_parse_one_date(d) for d in args.date]

    if args.time is not None and len(args.time) > 0:
        # Cross product of dates x times; a range collapses to one averaged spectrum
        parsed_times = [_parse_time_token(t) for t in args.time]
        for (y, m, d) in parsed_dates:
            for spec in parsed_times:
                if spec[0] == 'range':
                    (h1, mi1), (h2, mi2) = spec[1], spec[2]
                    start = pd.Timestamp(year=y, month=m, day=d, hour=h1, minute=mi1)
                    end = pd.Timestamp(year=y, month=m, day=d, hour=h2, minute=mi2)
                    extract_targets.append({
                        'kind': 'range',
                        'timestamp': start,
                        'end': end,
                        'label': f"{start:%Y-%m-%d %H:%M}-{end:%H:%M}",
                    })
                else:
                    h, mi = spec[1]
                    ts = pd.Timestamp(year=y, month=m, day=d, hour=h, minute=mi)
                    extract_targets.append({
                        'kind': 'single',
                        'timestamp': ts,
                        'label': ts.strftime('%Y-%m-%d %H:%M'),
                    })
        extract_single = True
        single_measurement_date = extract_targets[0]['timestamp']
        mode_word = "Time range" if all(t['kind'] == 'range' for t in extract_targets) else "Single measurement"
        if len(extract_targets) == 1:
            print(f"{mode_word} mode: {extract_targets[0]['label']}")
        else:
            print(f"{mode_word} mode: {len(extract_targets)} selections")
            for t in extract_targets:
                print(f"  - {t['label']}")
        print(f"  (All plotting and writing operations will use these measurements)\n")
    else:
        # Daily average mode: one target per date
        for (y, m, d) in parsed_dates:
            ts = pd.Timestamp(year=y, month=m, day=d, hour=0, minute=0)
            extract_targets.append({
                'kind': 'daily',
                'timestamp': ts,
                'label': ts.strftime('%Y-%m-%d'),
            })
        extract_daily = True
        daily_start = extract_targets[0]['timestamp']
        daily_end = daily_start.replace(hour=23, minute=59)
        if len(extract_targets) == 1:
            print(f"Daily average mode: {extract_targets[0]['label']} (00:00 to 23:59)")
        else:
            print(f"Daily average mode: {len(extract_targets)} days")
            for t in extract_targets:
                print(f"  - {t['label']}")
        print(f"  (All plotting and writing operations will use the daily average(s))\n")



# nargs="*" hands back a list; downstream code expects None/True/str, so keep both
def normalize_plot_arg(value):
    """Returns (normalized_arg, values_list). normalized_arg is None/True/str."""
    if value is None:
        return None, []
    if isinstance(value, list):
        if len(value) == 0:
            return True, []
        if len(value) == 1:
            return value[0], [value[0]]
        return value[0], list(value)
    return value, [value] if isinstance(value, str) else []

args.plot_spectrum, plot_spectrum_values = normalize_plot_arg(args.plot_spectrum)
args.plot_spectrum_pm, plot_spectrum_pm_values = normalize_plot_arg(args.plot_spectrum_pm)
args.plot_spectrum_js, plot_spectrum_js_values = normalize_plot_arg(args.plot_spectrum_js)
args.plot_wavestats, plot_wavestats_values = normalize_plot_arg(args.plot_wavestats)
args.plot_heatmap, plot_heatmap_values = normalize_plot_arg(args.plot_heatmap)
args.plot_wind, plot_wind_values = normalize_plot_arg(args.plot_wind)


def parse_month_list(values, min_count=2):
    """Parse a list of month tokens. Returns list of month numbers, or None if not all months.

    min_count guards how many values are required before the list branch is used, so
    flags that already have dedicated single-month handling keep it.
    """
    if not values or len(values) < min_count:
        return None
    months = []
    for v in values:
        num, is_month = parse_month(v)
        if not is_month:
            return None
        months.append(num)
    return months


# Plot-only flags do not by themselves imply intent to rewrite the YAML
plot_only_options = (
    args.plot_spectrum is not None or 
    args.plot_heatmap is not None or 
    args.plot_wavestats is not None or 
    args.plot_wavedirection is not None or 
    args.plot_spectrum_pm is not None or 
    args.plot_spectrum_js is not None or
    args.plot_wind is not None
)

# Check if user explicitly requested a YAML update (not just plot)
# These are parameters that indicate intent to update YAML
explicit_update_intent = (
    user_set_spectrum or
    args.depth is not None or
    user_set_type
)

# If plot-only and no explicit update intent, skip YAML update (height/period used only for plotting)
# If no plotting options, then height/period alone means update YAML
skip_yaml_update = plot_only_options and not explicit_update_intent
should_update_yaml = not skip_yaml_update

# Validate that path was provided, but only if updating YAML (not for plot-only)
if args.path is None and not skip_yaml_update:
    parser.error("Please provide a path to the YAML hydro file")

yaml_file_path = args.path if args.path is not None else ""

# Fail fast on a bad YAML path rather than after downloading buoy data and
# generating a full elevation record
if yaml_file_path and not skip_yaml_update:
    if not os.path.exists(yaml_file_path):
        target_dir = os.path.dirname(os.path.abspath(yaml_file_path))
        print(f"ERROR: YAML file not found: {yaml_file_path}")
        if not os.path.isdir(target_dir):
            print(f"       The directory does not exist either: {target_dir}")
            print(f"       Create the directory and place a .hydro.yaml file in it first.")
        else:
            existing = sorted(f for f in os.listdir(target_dir) if f.endswith('.yaml'))
            if existing:
                print(f"       YAML files present in {target_dir}:")
                for name in existing:
                    print(f"         - {name}")
            else:
                print(f"       No .yaml files found in {target_dir}")
        print("       This script updates an existing hydro YAML; it does not create one.")
        exit(1)

#~~~~~~~~~~~~~~ Existing eta file -> YAML (no buoy data needed) ~~~~~~~~~~~~~~~~

# An --elevation_file that already exists is used exactly as it is: nothing is
# downloaded, nothing is generated and the file is never rewritten.
existing_eta_path = None
if args.spectrum == "custom" and args.elevation_file and yaml_file_path:
    _yaml_dir = os.path.dirname(os.path.abspath(yaml_file_path))
    _candidate = args.elevation_file if os.path.isabs(args.elevation_file) else \
        os.path.normpath(os.path.join(_yaml_dir, args.elevation_file))
    if not os.path.isfile(_candidate) and not os.path.splitext(_candidate)[1]:
        _candidate += ".txt"
    if os.path.isfile(_candidate):
        existing_eta_path = _candidate

if existing_eta_path:
    print("=" * 60)
    print("Existing elevation file -> YAML (buoy data not used)")
    print("=" * 60)
    print(f"eta file: {existing_eta_path}")

    if args.elevation_duration is not None or args.elevation_dt is not None:
        print("  NOTE: --elevation_duration / --elevation_dt are ignored; the record is used as-is")

    times, etas = [], []
    with open(existing_eta_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line[0] in '#%':
                continue
            parts = line.replace(':', ' ').replace(',', ' ').split()
            try:
                times.append(float(parts[0]))
                etas.append(float(parts[1]))
            except (IndexError, ValueError):
                if not times:
                    continue  # header row
                print(f"ERROR: cannot parse '{line}'")
                print("       Expected 'time:elevation' (':', ',' or whitespace separated)")
                exit(1)

    if len(times) < 2:
        print(f"ERROR: {existing_eta_path} has fewer than 2 samples")
        exit(1)

    eta_values = np.asarray(etas)
    n_samples = len(times)
    duration = float(times[-1] - times[0])
    dt = duration / (n_samples - 1)
    print(f"  {n_samples} samples, {duration:.2f} s at dt={dt:g} s")
    print(f"  Elevation range {eta_values.min():.3f} to {eta_values.max():.3f} m, "
          f"Hm0 (4*sigma) = {4.0 * np.std(eta_values):.4f} m")

    # nfrequencies must span the default 0.001 - 1.0 Hz eta band: the C++ default of
    # 1000 strides the Fourier grid, silently dropping bins and attenuating the record.
    df_natural = 1.0 / (n_samples * dt)
    k_min = max(1, int(np.ceil(0.001 / df_natural)))
    k_max = int(np.floor(min(1.0, 1.0 / (2.0 * dt)) / df_natural))
    eta_nfrequencies = max(1, k_max - k_min + 1)
    print(f"  Fourier grid: df = {df_natural:.3e} Hz -> {eta_nfrequencies} bins "
          f"across the default 0.001 - 1.0 Hz eta band")
    if args.eta_method == "dft" and eta_nfrequencies > 4000:
        print(f"  WARNING: {eta_nfrequencies} wave components is a lot. The DFT is O(nf*N) and")
        print(f"           the GUI evaluates every component per surface vertex per frame.")

    eta_ref = os.path.relpath(existing_eta_path, _yaml_dir).replace('\\', '/')

    waves_block = {'type': 'irregular', 'eta_file': eta_ref}
    if args.eta_method == "irf":
        waves_block['method'] = "irf_convolution"
    else:
        waves_block['nfrequencies'] = int(eta_nfrequencies)

    ramp = float(args.ramp_time) if args.ramp_time is not None else 60.0
    if ramp > 0.0:
        waves_block['ramp_duration'] = ramp
        waves_block['ramp_type'] = "cosine"
    if args.depth is not None:
        waves_block['depth'] = float(args.depth)

    with open(yaml_file_path, 'r') as f:
        yaml_data = yaml.safe_load(f) or {}
    if 'waves' not in (yaml_data.get('hydrodynamics') or {}):
        print(f"ERROR: 'hydrodynamics.waves' section not found in {yaml_file_path}")
        exit(1)
    yaml_data['hydrodynamics']['waves'] = waves_block

    class IndentDumper(yaml.SafeDumper):
        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow, False)

    with open(yaml_file_path, 'w') as f:
        yaml.dump(yaml_data, f, Dumper=IndentDumper, default_flow_style=False, sort_keys=False)

    print(f"\nUpdated hydro YAML: {yaml_file_path}")
    for key, value in waves_block.items():
        print(f"  {key}: {value}")

    # The wave field returns zero elevation outside the table range, so the
    # companion simulation end_time must not exceed the record.
    base_name = os.path.basename(yaml_file_path)
    sim_stem = base_name[:-len('.hydro.yaml')] if base_name.endswith('.hydro.yaml') \
        else os.path.splitext(base_name)[0]
    sim_yaml_path = os.path.join(_yaml_dir, f"{sim_stem}.simulation.yaml")
    if not os.path.exists(sim_yaml_path):
        import glob
        matches = sorted(glob.glob(os.path.join(_yaml_dir, "*.simulation.yaml")))
        sim_yaml_path = matches[0] if len(matches) == 1 else None

    if sim_yaml_path is None:
        print(f"\nNOTE: no companion *.simulation.yaml found; end_time not updated.")
    else:
        with open(sim_yaml_path, 'r') as f:
            sim_lines = f.read().splitlines()

        sim_step = None
        in_simulation = False
        updated = False
        old_end_time = None
        for i, line in enumerate(sim_lines):
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            indent = len(line) - len(line.lstrip())
            if indent == 0:
                in_simulation = stripped.rstrip(':') == 'simulation'
                continue
            if not in_simulation:
                continue
            key = stripped.split(':')[0].strip()
            if key == 'time_step':
                try:
                    sim_step = float(stripped.split(':', 1)[1].split('#')[0].strip())
                except ValueError:
                    pass
            elif key == 'end_time':
                old_end_time = stripped.split(':', 1)[1].strip()
                sim_lines[i] = f"{' ' * indent}end_time: {duration:.1f}"
                updated = True

        if updated:
            with open(sim_yaml_path, 'w', newline='\n') as f:
                f.write("\n".join(sim_lines) + "\n")
            print(f"\nUpdated simulation YAML: {sim_yaml_path}")
            print(f"  end_time: {old_end_time} -> {duration:.1f} s (matches eta record length)")
        else:
            print(f"\nWARNING: no 'end_time' key found under 'simulation:' in {sim_yaml_path}")

        if sim_step and dt > sim_step:
            print(f"  WARNING: eta dt ({dt} s) is coarser than simulation.time_step ({sim_step} s)")

    exit(0)

#~~~~~~~~~~~from buoy number~~~~~~~~~~~~~~~~~~~~~~~~~~~

# # use if SSLcertificateError / VPN blocks from downloading data from NDBC website
os.environ["REQUESTS_CA_BUNDLE"] = r"C:\Users\ariley\OneDrive - NREL\Documents\nrel_root_ca.cer"
os.environ["SSL_CERT_FILE"] = r"C:\Users\ariley\OneDrive - NREL\Documents\nrel_root_ca.cer"

#~~~~~~~~~~~~~~~INPUTS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

buoy_number = args.buoy
yaml_file_path = args.path
wave_type = args.type

#~~~~~~~~~~~~~~ Custom spectrum from a file (no buoy data) ~~~~~~~~~~~~~~~~~~~~~

def load_spectrum_file(path):
    """Read a two-row or two-column frequency/spectral-density file."""
    rows = []
    with open(path, 'r') as f:
        for line in f:
            line = line.split('#')[0].split('%')[0].strip()
            if not line:
                continue
            try:
                rows.append([float(tok) for tok in line.replace(',', ' ').split()])
            except ValueError:
                if not rows:
                    continue  # header row
                raise ValueError(f"cannot parse numbers from line: {line!r}")

    if len(rows) == 2 and len(rows[0]) == len(rows[1]) and len(rows[0]) > 2:
        freq, dens = np.asarray(rows[0]), np.asarray(rows[1])
    elif len(rows) > 2 and all(len(r) == 2 for r in rows):
        arr = np.asarray(rows)
        freq, dens = arr[:, 0], arr[:, 1]
    else:
        raise ValueError(
            "unrecognised layout. Expected either two rows (frequencies, then "
            "spectral densities) or two columns (frequency, density)")

    if np.any(np.diff(freq) <= 0):
        order = np.argsort(freq)
        freq, dens = freq[order], dens[order]
    if np.any(freq < 0) or np.any(dens < 0):
        raise ValueError("frequencies and spectral densities must be non-negative")
    return freq, dens


spectrum_from_file = bool(args.spectrum_file)
custom_freq = custom_S = None
spectrum_label = None
# Titles say "NDBC Buoy <n>" for downloaded data and the file name otherwise
source_title = f"NDBC Buoy {buoy_number}"

if spectrum_from_file:
    _yaml_dir_for_spec = os.path.dirname(os.path.abspath(yaml_file_path)) if yaml_file_path else os.getcwd()
    spectrum_file_path = args.spectrum_file if os.path.isabs(args.spectrum_file) else \
        os.path.normpath(os.path.join(_yaml_dir_for_spec, args.spectrum_file))
    if not os.path.isfile(spectrum_file_path):
        # Also accept a path relative to the working directory
        alt = os.path.abspath(args.spectrum_file)
        if os.path.isfile(alt):
            spectrum_file_path = alt
    if not os.path.isfile(spectrum_file_path):
        print(f"ERROR: spectrum file not found: {args.spectrum_file}")
        print(f"       Looked in {_yaml_dir_for_spec} and {os.getcwd()}")
        exit(1)

    try:
        custom_freq, custom_S = load_spectrum_file(spectrum_file_path)
    except (ValueError, OSError) as e:
        print(f"ERROR: could not read spectrum file {spectrum_file_path}: {e}")
        exit(1)

    spectrum_label = os.path.basename(spectrum_file_path)
    source_title = spectrum_label
    _m0 = float(integrate.trapezoid(custom_S, custom_freq))
    print("=" * 60)
    print("Custom spectrum from file (buoy data not used)")
    print("=" * 60)
    print(f"File: {spectrum_file_path}")
    print(f"  {len(custom_freq)} bins, {custom_freq.min():.4f} - {custom_freq.max():.4f} Hz")
    print(f"  Hm0 = {4.0 * np.sqrt(_m0):.4f} m")

# ========== Define spectrum helper functions EARLY (before YAML update) ==========

def plot_heading(label, model=""):
    """Plot title head, without repeating the source when it already is the label."""
    head = label if label == source_title else f"{source_title} - {label}"
    return f"{head} - {model}" if model else head


# JONSWAP is only defined for gamma >= 1 by the standard definition; 0.9 is the
# practical lower limit allowed here (empirical best fits sit just below 1.0).
GAMMA_MIN = 0.9
GAMMA_MAX = 10.0

def parse_month(month_input):
    """Parse month name or number. Returns (month_number, is_month) tuple."""
    month_map = {
        'jan': 1, 'january': 1,
        'feb': 2, 'february': 2,
        'mar': 3, 'march': 3,
        'apr': 4, 'april': 4,
        'may': 5,
        'jun': 6, 'june': 6,
        'jul': 7, 'july': 7,
        'aug': 8, 'august': 8,
        'sep': 9, 'september': 9,
        'oct': 10, 'october': 10,
        'nov': 11, 'november': 11,
        'dec': 12, 'december': 12
    }
    
    if isinstance(month_input, str):
        lower_input = month_input.lower()
        if lower_input in month_map:
            return month_map[lower_input], True
        try:
            month_num = int(month_input)
            if 1 <= month_num <= 12:
                return month_num, True
        except ValueError:
            pass
    return None, False


def pm_spectrum(freq, hm0, tp):
    """
    Unified Pierson-Moskowitz spectrum function.
    
    Parameters:
    - freq: frequency array [Hz]
    - hm0: significant wave height [m]
    - tp: peak period [s]
    
    Returns: spectrum S(f) normalized so that 4*sqrt(integral(S)) = hm0
    """
    freq = np.asarray(freq)
    fp = 1.0 / tp  # peak frequency
    
    # Avoid division by zero
    safe_freq = np.where(freq > 1e-10, freq, 1e-10)
    
    # Unnormalized PM shape: f^(-5) * exp(-5/4 * (fp/f)^4)
    exponent = -5.0/4.0 * (fp / safe_freq)**4
    S_unnormalized = (safe_freq**(-5)) * np.exp(exponent)
    
    # Integrate to get unnormalized zeroth moment
    m0_unnormalized = integrate.trapezoid(S_unnormalized, freq)
    
    # Scale so that 4*sqrt(m0) = hm0 (significant wave height definition)
    target_m0 = (hm0 / 4.0) ** 2.0
    if m0_unnormalized > 0:
        S = S_unnormalized * (target_m0 / m0_unnormalized)
    else:
        S = S_unnormalized
    
    return S


def jonswap_spectrum(freq, hm0, tp, gamma=3.3):
    """
    Unified JONSWAP spectrum function.
    
    JONSWAP = C(gamma) * S_PM(f) * gamma^[exp(-(f-fp)²/(2σ²fp²))]
    
    where C(gamma) = integral(S_PM) / integral(S_PM * gamma^peak_factor)
    
    Parameters:
    - freq: frequency array [Hz]
    - hm0: significant wave height [m]
    - tp: peak period [s]
    - gamma: peak enhancement factor (1-5, typical 3.3)
    
    Returns: spectrum S(f) normalized so that 4*sqrt(integral(S)) = hm0
    """
    freq = np.asarray(freq)
    fp = 1.0 / tp  # peak frequency
    
    # Get base PM spectrum
    S_pm = pm_spectrum(freq, hm0, tp)
    
    # Peak enhancement factor: gamma^[-(f - fp)^2 / (2*sigma^2 * fp^2)]
    sigma = np.where(freq <= fp, 0.07, 0.09)
    exponent = -((freq - fp)**2.0) / (2.0 * (sigma**2.0) * (fp**2.0))
    exponent = np.clip(exponent, -100, 100)
    
    # Compute gamma^exponent
    if gamma > 0:
        log_gamma = np.log(gamma)
        peak_factor = np.exp(np.clip(exponent * log_gamma, -100, 100))
    else:
        peak_factor = np.ones_like(freq)
    
    # Compute C(gamma) normalization constant to maintain m0 when peak is enhanced
    safe_freq = np.where(freq > 1e-10, freq, 1e-10)
    S_pm_unnormalized = (safe_freq**(-5.0)) * np.exp(-5.0/4.0 * (fp / safe_freq)**4.0)
    
    integral_pm_unnormalized = integrate.trapezoid(S_pm_unnormalized, freq)
    integral_pm_enhanced = integrate.trapezoid(S_pm_unnormalized * peak_factor, freq)
    
    if integral_pm_enhanced > 0 and np.isfinite(integral_pm_enhanced):
        c_gamma = integral_pm_unnormalized / integral_pm_enhanced
    else:
        c_gamma = 1.0
    
    # Apply peak enhancement to normalized PM spectrum
    # Then renormalize to target Hm0
    S_jonswap_before_norm = c_gamma * S_pm * peak_factor
    
    # Renormalize to target Hm0
    m0_jonswap = integrate.trapezoid(S_jonswap_before_norm, freq)
    target_m0 = (hm0 / 4.0) ** 2.0
    
    if m0_jonswap > 0 and np.isfinite(m0_jonswap):
        S = S_jonswap_before_norm * (target_m0 / m0_jonswap)
    else:
        S = S_jonswap_before_norm
    
    # Ensure no NaNs or infs
    S = np.nan_to_num(S, nan=0.0, posinf=0.0, neginf=0.0)
    
    return S


def jonswap_spectrum_custom(freq, Tp, Hs, gamma=3.3, alpha=0.0081):
    """
    Legacy wrapper - calls unified jonswap_spectrum for backwards compatibility.
    """
    return jonswap_spectrum(freq, Hs, Tp, gamma=gamma)


def pierson_moskowitz_spectrum(freq, hm0, tp):
    """
    Legacy wrapper - calls unified pm_spectrum for backwards compatibility.
    """
    return pm_spectrum(freq, hm0, tp)


def fit_jonswap_gamma(measured_spectrum, freq, Tp, Hs, gamma_initial=3.3, fit_tp=True):
    """
    Fit a JONSWAP spectrum to a measured spectrum by least squares.

    Fits gamma (peak enhancement), and by default also refines Tp (peak period).
    Refining Tp matters because the measured Tp comes from the single highest
    spectral bin, which is noisy and can land on the wrong lobe of a broad or
    double-peaked sea. Hs is held fixed so the fitted spectrum keeps the same
    total energy (m0) as the measurement.

    gamma is constrained to [GAMMA_MIN, GAMMA_MAX]; JONSWAP is undefined below 1
    by the standard definition.

    Returns: (gamma_fit, error, tp_fit, hit_bound)
    """
    freq_array = np.asarray(freq)
    Tp = float(Tp)
    Hs = float(Hs)

    if hasattr(measured_spectrum, 'values'):
        measured_values = measured_spectrum.values
    else:
        measured_values = np.asarray(measured_spectrum)

    def sse(gamma, tp):
        """Sum of squared error between measured spectrum and JONSWAP."""
        if not np.isfinite(gamma) or not np.isfinite(tp) or tp <= 0:
            return 1e12
        js_spectrum = jonswap_spectrum(freq_array, Hs, tp, gamma=float(gamma))
        return float(np.sum((measured_values - js_spectrum) ** 2))

    if fit_tp:
        # Allow the peak period to move +/-30% around the measured value
        tp_lo, tp_hi = 0.7 * Tp, 1.3 * Tp

        def objective(params):
            gamma, tp = float(params[0]), float(params[1])
            if not (GAMMA_MIN <= gamma <= GAMMA_MAX) or not (tp_lo <= tp <= tp_hi):
                return 1e12
            return sse(gamma, tp)

        best = None
        # Multi-start so the optimizer does not settle in a local minimum
        for g0 in (1.0, 2.0, 3.3, 5.0):
            for t0 in (Tp, 0.9 * Tp, 1.1 * Tp):
                res = minimize(objective, x0=[g0, t0], method='Nelder-Mead',
                               options={'maxiter': 800, 'xatol': 1e-6, 'fatol': 1e-9})
                if best is None or res.fun < best.fun:
                    best = res

        gamma_fit = float(np.clip(best.x[0], GAMMA_MIN, GAMMA_MAX))
        tp_fit = float(np.clip(best.x[1], tp_lo, tp_hi))
        final_error = sse(gamma_fit, tp_fit)
    else:
        # Gamma-only fit on a bounded 1-D problem
        res = minimize_scalar(lambda g: sse(g, Tp), bounds=(GAMMA_MIN, GAMMA_MAX),
                              method='bounded', options={'xatol': 1e-6})
        gamma_fit = float(np.clip(res.x, GAMMA_MIN, GAMMA_MAX))
        tp_fit = Tp
        final_error = sse(gamma_fit, tp_fit)

    # Flag when the fit is pinned to a gamma bound (means JONSWAP cannot match the shape)
    hit_bound = (abs(gamma_fit - GAMMA_MIN) < 1e-3) or (abs(gamma_fit - GAMMA_MAX) < 1e-3)

    return gamma_fit, final_error, tp_fit, hit_bound

# ========== End spectrum helper functions ==========

def find_simulation_yaml(hydro_path):
    """Locate the companion *.simulation.yaml next to a hydro YAML."""
    if not hydro_path:
        return None
    directory = os.path.dirname(os.path.abspath(hydro_path))
    base = os.path.basename(hydro_path)
    stem = base[:-len('.hydro.yaml')] if base.endswith('.hydro.yaml') \
        else os.path.splitext(base)[0]

    candidate = os.path.join(directory, f"{stem}.simulation.yaml")
    if os.path.exists(candidate):
        return candidate

    import glob
    matches = sorted(glob.glob(os.path.join(directory, "*.simulation.yaml")))
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        print(f"\nWARNING: multiple *.simulation.yaml files found in {directory}; "
              f"expected {stem}.simulation.yaml")
    return None


def update_simulation_yaml(sim_path, end_time=None, time_step=None, reason=""):
    """Rewrite end_time / time_step in place so comments and 1e-4 notation survive."""
    wanted = {}
    if end_time is not None:
        wanted['end_time'] = f"{float(end_time):.1f}"
    if time_step is not None:
        wanted['time_step'] = f"{float(time_step):g}"
    if not wanted:
        return

    with open(sim_path, 'r') as f:
        lines = f.read().splitlines()

    in_simulation = False
    changed = {}
    for i, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        indent = len(line) - len(line.lstrip())
        if indent == 0:
            in_simulation = stripped.rstrip(':') == 'simulation'
            continue
        if not in_simulation:
            continue
        key = stripped.split(':')[0].strip()
        if key in wanted:
            old = stripped.split(':', 1)[1].split('#')[0].strip()
            lines[i] = f"{' ' * indent}{key}: {wanted[key]}"
            changed[key] = (old, wanted[key])

    if changed:
        with open(sim_path, 'w', newline='\n') as f:
            f.write("\n".join(lines) + "\n")
        print(f"\nUpdated simulation YAML: {sim_path}")
        for key, (old, new) in changed.items():
            print(f"  {key}: {old} -> {new}{reason if key == 'end_time' else ''}")
    for key in wanted:
        if key not in changed:
            print(f"\nWARNING: no '{key}' key found under 'simulation:' in {sim_path}")


water_depth = None
if spectrum_from_file:
    water_depth = float(args.depth) if args.depth is not None else None
else:
    try:
        buoy_metadata = ndbc.get_buoy_metadata(buoy_number)
    except Exception as e:
        print(f"ERROR: Could not fetch buoy metadata for {buoy_number}: {e}")
        exit(1)

    # Extract water depth from metadata
    if 'Water depth' in buoy_metadata:
        depth_str = buoy_metadata['Water depth']
        # Parse depth from string like "149 m"
        try:
            water_depth = float(depth_str.split()[0])
        except (ValueError, IndexError):
            print(f"Warning: Could not parse water depth from '{depth_str}'")

if water_depth:
    print(f"\nExtracted Water Depth: {water_depth} m")

#~~~~~~~~~~~~~~ Parse year specification ~~~~~~~~~~~~~~

def parse_year_spec(year_args, available_years):
    """
    Parse --year arguments to determine which years to use
    
    Args:
        year_args: List of arguments from --year (or None if not provided)
        available_years: List of available years
    
    Returns:
        Tuple of (years_list, is_default)
    """
    if year_args is None or len(year_args) == 0:
        # Default: use latest year (2025 if available, otherwise most recent)
        if 2025 in available_years:
            return [2025], True
        else:
            return [available_years[-1]], True
    
    years_selected = []
    
    for arg in year_args:
        if arg.startswith('from:'):
            # Range: from:2020 means 2020 and onwards
            threshold_year = int(arg[5:])
            years_selected.extend([y for y in available_years if y >= threshold_year])
        else:
            # Single year
            year = int(arg)
            if year in available_years:
                years_selected.append(year)
    
    return sorted(list(set(years_selected))), False  # Remove duplicates and sort


parameter = "swden"
ndbc_data = {}

if spectrum_from_file:
    available_years = []
    selected_years = []
    is_default = False
else:
    ndbc_available_data = ndbc.available_data(parameter, buoy_number)
    available_years = sorted([int(y) for y in ndbc_available_data["year"].values])
    print(f"Available years: {available_years}")

    if not available_years:
        # Buoys operated by partners (e.g. CDIP/Scripps) can have a live realtime feed on
        # ndbc.noaa.gov while having nothing in https://www.ndbc.noaa.gov/data/historical/swden/
        print(f"ERROR: NDBC has no historical spectral wave density (swden) archive for buoy {buoy_number}.")
        print(f"       The buoy may still show data on the website - that is the realtime feed, which")
        print(f"       this script cannot use for wave statistics. Check for swden files at:")
        print(f"       https://www.ndbc.noaa.gov/station_history.php?station={buoy_number}")
        print(f"       Pick a buoy with a swden archive (e.g. 46050, 46042, 46086).")
        exit(1)

    # Parse year specification
    selected_years, is_default = parse_year_spec(args.year, available_years)

    # A YYYY-MM-DD passed to --date already pins the year, so don't fall back to "latest".
    if extract_targets and (args.year is None or len(args.year) == 0):
        date_years = sorted({t['timestamp'].year for t in extract_targets})
        unavailable = [y for y in date_years if y not in available_years]
        if unavailable:
            print(f"WARNING: year(s) {', '.join(map(str, unavailable))} from --date are not in the "
                  f"swden archive for buoy {buoy_number}.")
        selected_years = [y for y in date_years if y in available_years]
        is_default = False

    # Only warn about a defaulted year when no plot flag already pins one
    plot_spec_has_year = args.plot_spectrum is not None and isinstance(args.plot_spectrum, str)
    plot_spec_pm_has_year = args.plot_spectrum_pm is not None and isinstance(args.plot_spectrum_pm, str)
    plot_spec_js_has_year = args.plot_spectrum_js is not None and isinstance(args.plot_spectrum_js, str)

    if is_default and not plot_spec_has_year and not plot_spec_pm_has_year and not plot_spec_js_has_year:
        print(f"WARNING: No year specified, using latest available: {selected_years}")
    else:
        if not is_default:
            print(f"Using years: {selected_years}")

    filenames = ndbc_available_data["filename"]
    try:
        raw_data = ndbc.request_data(parameter, filenames)
    except Exception as e:
        print(f"ERROR: Could not fetch NDBC data: {e}")
        exit(1)
    for year in raw_data:
        year_data = raw_data[year]
        swden_data = ndbc.to_datetime_index(parameter, year_data)
        swden_data = swden_data.T
        swden_data = swden_data.sort_index()
        ndbc_data[year] = swden_data

wind_data = {}
if args.plot_wind is not None:
    print("\nFetching wind data (stdmet)...")
    try:
        wind_parameter = "stdmet"
        wind_ndbc_available = ndbc.available_data(wind_parameter, buoy_number)
        wind_filenames = wind_ndbc_available["filename"]
        wind_raw = ndbc.request_data(wind_parameter, wind_filenames)
        
        for year in wind_raw:
            year_wind_data = wind_raw[year]
            stdmet_data = ndbc.to_datetime_index(wind_parameter, year_wind_data)
            stdmet_data = stdmet_data.T
            stdmet_data = stdmet_data.sort_index()
            wind_data[year] = stdmet_data
        print(f"  Wind data loaded for years: {list(wind_data.keys())}")
    except Exception as e:
        print(f"  WARNING: Could not fetch wind data: {e}")

# Handle date/time extraction, building one labeled spectrum per requested target
extracted_spectra = []  # list of (label, pd.Series) in the order requested

if extract_single:
    for target in extract_targets:
        ts = target['timestamp']
        year_key = str(ts.year)
        if year_key not in ndbc_data:
            print(f"ERROR: Year {year_key} not found in available data")
            exit(1)

        year_data = ndbc_data[year_key]

        if target['kind'] == 'range':
            end_ts = target['end']
            window = [c for c in year_data.columns if ts <= c <= end_ts]
            window_spectra = [year_data[c].dropna() for c in window]
            window_spectra = [s for s in window_spectra if len(s) > 0]
            if not window_spectra:
                print(f"ERROR: No measurements between {ts:%Y-%m-%d %H:%M} and {end_ts:%H:%M}")
                exit(1)
            print(f"  Found {len(window_spectra)} measurements in {target['label']}. Using average.")
            extracted_spectra.append(
                (target['label'], pd.concat(window_spectra, axis=1).mean(axis=1)))
            continue

        try:
            spectrum_clean = year_data[ts].dropna()
            if len(spectrum_clean) == 0:
                raise KeyError(ts)
            print(f"  Found measurement at {ts.strftime('%Y-%m-%d %H:%M')} with {len(spectrum_clean)} frequencies")
            extracted_spectra.append((target['label'], spectrum_clean))
        except KeyError:
            # Fall back to the closest available time within 1 hour
            available_times = year_data.columns
            closest_time = min(available_times, key=lambda x: abs((x - ts).total_seconds()))
            time_diff = abs((closest_time - ts).total_seconds())

            if time_diff < 3600:
                print(f"  Exact time not found for {ts.strftime('%Y-%m-%d %H:%M')}. "
                      f"Using closest available: {closest_time.strftime('%Y-%m-%d %H:%M')} ({time_diff/60:.0f} min difference)")
                spectrum_clean = year_data[closest_time].dropna()
                extracted_spectra.append((closest_time.strftime('%Y-%m-%d %H:%M'), spectrum_clean))
            else:
                print(f"ERROR: No data near {ts.strftime('%Y-%m-%d %H:%M')}. "
                      f"Closest available: {closest_time.strftime('%Y-%m-%d %H:%M')} ({time_diff/3600:.1f} hours away)")
                exit(1)

elif extract_daily:
    for target in extract_targets:
        day_ts = target['timestamp']
        year_key = str(day_ts.year)
        if year_key not in ndbc_data:
            print(f"ERROR: Year {year_key} not found in available data")
            exit(1)

        year_data = ndbc_data[year_key]

        # Collect every measurement on this calendar date
        daily_spectra = []
        for timestamp, spectrum in year_data.items():
            if day_ts.date() == timestamp.date():
                spectrum_clean = spectrum.dropna()
                if len(spectrum_clean) > 0:
                    daily_spectra.append(spectrum_clean)

        if len(daily_spectra) == 0:
            print(f"ERROR: No data found for {day_ts.strftime('%Y-%m-%d')}")
            exit(1)

        daily_avg = pd.concat(daily_spectra, axis=1).mean(axis=1)
        print(f"  Found {len(daily_spectra)} measurements for {day_ts.strftime('%Y-%m-%d')}. Using daily average.")
        extracted_spectra.append((target['label'], daily_avg))

if extracted_spectra:
    # all_swden_data drives the wave statistics and YAML update; keep every extracted
    # spectrum so stats reflect the full selection
    all_swden_data = pd.concat(
        [s.rename(label) for label, s in extracted_spectra], axis=1
    )

# Extract wind data for the same measurement/day if wind flag is set
all_wind_data = None
if args.plot_wind is not None and len(wind_data) > 0:
    if extract_single:
        year_key = str(single_measurement_date.year)
        if year_key in wind_data:
            year_wind = wind_data[year_key]
            try:
                wind_measurement = year_wind[single_measurement_date]
                all_wind_data = pd.DataFrame({single_measurement_date: wind_measurement})
            except KeyError:
                # Try closest wind measurement
                available_times = year_wind.columns
                closest_time = min(available_times, key=lambda x: abs((x - single_measurement_date).total_seconds()))
                time_diff = abs((closest_time - single_measurement_date).total_seconds())
                if time_diff < 3600:
                    wind_measurement = year_wind[closest_time]
                    all_wind_data = pd.DataFrame({closest_time: wind_measurement})
    
    elif extract_daily:
        year_key = str(daily_start.year)
        if year_key in wind_data:
            year_wind = wind_data[year_key]
            # Filter for this day only
            daily_winds = []
            for timestamp, wind_row in year_wind.items():
                if daily_start.date() == timestamp.date():
                    wind_clean = wind_row.dropna()
                    if len(wind_clean) > 0:
                        daily_winds.append(wind_clean)
            
            if len(daily_winds) > 0:
                daily_wind_avg = pd.concat(daily_winds, axis=1).mean(axis=1)
                all_wind_data = pd.DataFrame({daily_start: daily_wind_avg})

# A file-supplied spectrum stands in for a single extracted measurement, so the
# statistics, plotting and eta-generation paths below need no special casing.
if spectrum_from_file:
    _series = pd.Series(custom_S, index=pd.Index(custom_freq, name="Frequency"))
    all_swden_data = pd.DataFrame({spectrum_label: _series})
    extracted_spectra = [(spectrum_label, _series)]
    extract_single = True

# Extract and print min/max frequencies only for irregular waves
if wave_type == "irregular" and (extract_single or extract_daily):
    # Get first spectrum from extracted data
    spectrum = all_swden_data.iloc[:, 0]
    spectrum = spectrum.dropna()
    
    # Calculate min and max frequencies
    freq_index = spectrum.index.values
    min_freq = freq_index.min()
    max_freq = freq_index.max()
    print(f"\nSpectrum Frequency Range:")
    print(f"  Min frequency: {min_freq:.6f} Hz")
    print(f"  Max frequency: {max_freq:.6f} Hz")

# Concatenate data from selected years only (if not in extract_single/daily mode)
if not extract_single and not extract_daily:
    selected_year_keys = [str(y) for y in selected_years]
    selected_swden_data = [ndbc_data[year_key] for year_key in selected_year_keys if year_key in ndbc_data]

    if not selected_swden_data:
        print(f"ERROR: No data found for selected years {selected_years}")
        exit(1)

    all_swden_data = pd.concat(selected_swden_data, axis=1, sort=False)
    all_swden_data = all_swden_data.sort_index(axis=0)
    all_swden_data = all_swden_data.sort_index(axis=1)
else:
    # all_swden_data already set in single measurement extraction above
    pass

Hm0 = wave.resource.significant_wave_height(
    all_swden_data)
Te = wave.resource.energy_period(
    all_swden_data)
Tp = wave.resource.peak_period(
    all_swden_data)

print("\nWave Statistics")
print("----------------")
print(
    f"Mean Significant Wave Height Hm0: "
    f"{Hm0.mean():.2f} m")
print(
    f"Mean Energy Period Te: "
    f"{Te.mean():.2f} s")
print(
    f"Mean Peak Period Tp: "
    f"{Tp.mean():.2f} s")

Hs_rep = Hm0.mean()
Te_rep = Te.mean()

# Display wind statistics if requested
if args.plot_wind is not None and len(wind_data) > 0:
    print("\nWind Statistics")
    print("----------------")
    
    # Use extracted wind data if available (single/daily mode) or fetch from selected years
    if all_wind_data is not None:
        wind_df = all_wind_data
    else:
        # Concatenate wind data from selected years
        selected_wind_keys = [str(y) for y in selected_years]
        selected_wind_data = [wind_data[year_key] for year_key in selected_wind_keys if year_key in wind_data]
        
        if selected_wind_data:
            wind_df = pd.concat(selected_wind_data, axis=1, sort=False)
        else:
            wind_df = None
    
    if wind_df is not None:
        # NDBC stdmet columns: WSPD (wind speed m/s), WDIR (direction deg), GST (gusts m/s)
        # Missing values use sentinels: 99.0 for speed/gust, 999 for direction
        wind_speed = wind_df.loc['WSPD'] if 'WSPD' in wind_df.index else None
        wind_direction = wind_df.loc['WDIR'] if 'WDIR' in wind_df.index else None
        wind_gusts = wind_df.loc['GST'] if 'GST' in wind_df.index else None
        
        if wind_speed is not None:
            valid_speed = wind_speed[wind_speed < 90]
            if len(valid_speed) > 0:
                print(f"Mean Wind Speed: {valid_speed.mean():.2f} m/s")
        
        if wind_direction is not None:
            valid_dir = wind_direction[wind_direction <= 360]
            if len(valid_dir) > 0:
                print(f"Mean Wind Direction: {valid_dir.mean():.1f} deg")
        
        if wind_gusts is not None:
            valid_gusts = wind_gusts[wind_gusts < 90]
            if len(valid_gusts) > 0:
                print(f"Mean Wind Gusts: {valid_gusts.mean():.2f} m/s")

#~~~~~~~~~~~~~~ Custom spectrum -> surface elevation time series ~~~~~~~~~~~~~~

# EtaTableWaveField / ComponentSampler::BuildFromEtaFile require one "time:elevation"
# pair per line, colon delimited, no header and no blank lines.
eta_file_name = None

if wave_type == "irregular" and args.spectrum == "custom" and not skip_yaml_update:
    from mhkit.wave.resource import surface_elevation

    print("\n" + "=" * 60)
    print("Custom spectrum -> surface elevation")
    print("=" * 60)

    # Pick the source spectrum: an extracted measurement/day if one was requested,
    # otherwise the average over the selected years
    if extracted_spectra:
        source_label, source_spectrum = extracted_spectra[0]
        if len(extracted_spectra) > 1:
            print(f"NOTE: {len(extracted_spectra)} spectra selected; using the first ({source_label})")
    else:
        source_spectrum = all_swden_data.mean(axis=1).dropna()
        source_label = f"Average of {', '.join(map(str, selected_years))}"

    freq_measured = source_spectrum.index.astype(float).values
    S_measured = source_spectrum.values.astype(float)

    print(f"Source spectrum: {source_label}")
    print(f"  Measured bins: {len(freq_measured)} "
          f"({freq_measured.min():.4f} - {freq_measured.max():.4f} Hz, non-uniform spacing)")

    # The companion simulation YAML sets the eta sample rate via time_step, and its
    # end_time must match the record: the wave field is zero outside the table range.
    yaml_dir = os.path.dirname(os.path.abspath(yaml_file_path)) if yaml_file_path else os.getcwd()
    sim_yaml_path = find_simulation_yaml(yaml_file_path)

    sim_time_step = None
    if sim_yaml_path:
        with open(sim_yaml_path, 'r') as f:
            sim_data = yaml.safe_load(f) or {}
        raw_step = (sim_data.get('simulation') or {}).get('time_step')
        if raw_step is not None:
            sim_time_step = float(raw_step)

    # Build the elevation time series
    duration = float(args.elevation_duration) if args.elevation_duration is not None else 600.0
    if args.elevation_dt is not None:
        dt = float(args.elevation_dt)
    elif sim_time_step and sim_time_step > 0.0:
        dt = sim_time_step
        print(f"\nEta dt = {dt} s (from simulation.time_step in "
              f"{os.path.basename(sim_yaml_path)})")
    else:
        dt = 0.05

    # Only a concern when the solver step is left alone; --elevation_dt rewrites it below
    if args.elevation_dt is None and sim_time_step and dt > sim_time_step:
        print(f"\nWARNING: eta dt ({dt} s) is coarser than simulation.time_step "
              f"({sim_time_step} s). The solver steps faster than the record is sampled.")

    n_samples = int(round(duration / dt)) + 1
    time_vec = np.arange(n_samples) * dt

    # The frequency grid is dictated by the time vector, NOT chosen freely.
    # surface_elevation's ifft calls np.fft.irfftn(..., s=[n_samples]), which reads bin
    # i as i/(n_samples*dt) and ignores the index on S. Any other spacing silently
    # rescales every frequency (a 512-bin 0-0.485 Hz grid over 120 s stretched the
    # spectrum 8.8x, turning 17 s swell into 2 s chop).
    df_uniform = 1.0 / (n_samples * dt)
    n_freq = n_samples // 2 + 1
    freq_uniform = np.arange(n_freq) * df_uniform
    f_max = float(freq_measured.max())

    nyquist = 1.0 / (2.0 * dt)
    if f_max > nyquist:
        print(f"  WARNING: spectrum extends to {f_max:.3f} Hz but the Nyquist frequency for "
              f"dt={dt} s is {nyquist:.3f} Hz. Energy above Nyquist will alias; "
              f"reduce --elevation_dt to resolve it.")

    # Interpolate; energy outside the measured band is zero (no extrapolation)
    S_uniform = np.interp(freq_uniform, freq_measured, S_measured, left=0.0, right=0.0)
    S_uniform[freq_uniform < freq_measured.min()] = 0.0

    # Preserve total variance (m0) so Hm0 of the realization matches the measurement
    m0_measured = float(integrate.trapezoid(S_measured, freq_measured))
    m0_uniform = float(integrate.trapezoid(S_uniform, freq_uniform))
    if m0_uniform > 0:
        S_uniform *= (m0_measured / m0_uniform)

    hm0_measured = 4.0 * np.sqrt(m0_measured)
    hm0_uniform = 4.0 * np.sqrt(float(integrate.trapezoid(S_uniform, freq_uniform)))
    n_in_band = int(np.count_nonzero(S_uniform > 0.0))
    print(f"  Record grid: {n_freq} bins (0 - {freq_uniform[-1]:.4f} Hz, df = {df_uniform:.6f} Hz)")
    print(f"  Bins carrying measured energy: {n_in_band}")
    print(f"  Hm0 measured = {hm0_measured:.4f} m  |  interpolated = {hm0_uniform:.4f} m")
    if n_in_band < 30:
        print(f"  WARNING: only {n_in_band} bins fall inside the measured band. The sea state will")
        print(f"           look repetitive; lengthen --elevation_duration for a finer df.")

    # Plot measured vs interpolated spectrum
    plt.figure(figsize=(12, 6))
    plt.plot(freq_measured, S_measured, 'o-', label=f"Measured ({len(freq_measured)} bins)",
             linewidth=2.0, markersize=4, color='steelblue', alpha=0.9)
    plt.plot(freq_uniform, S_uniform, '-', label=f"Interpolated (df = {df_uniform:.5f} Hz)",
             linewidth=1.5, color='coral', alpha=0.85)
    plt.xlim(0.0, f_max)
    plt.xlabel("Frequency [Hz]", fontsize=12)
    plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
    plt.title(f"{source_title} - Measured vs Interpolated Spectrum\n{source_label}", fontsize=13)
    plt.legend(fontsize=10, loc='upper right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
    plt.close()

    S_df = pd.DataFrame({0: S_uniform}, index=pd.Index(freq_uniform, name="Frequency"))
    seed = args.seed if args.seed is not None else 42

    print(f"\nGenerating elevation: {duration:.0f} s at dt={dt} s ({n_samples} samples), seed={seed}")
    eta = surface_elevation(S_df, time_vec, seed=seed)
    eta_values = np.asarray(eta).squeeze()

    hm0_realized = 4.0 * np.std(eta_values)
    print(f"  Elevation range: {eta_values.min():.3f} to {eta_values.max():.3f} m")
    print(f"  Realized Hm0 (4*sigma) = {hm0_realized:.4f} m (target {hm0_measured:.4f} m)")

    # Write the eta file next to the YAML
    if args.elevation_file:
        eta_file_name = args.elevation_file.replace('\\', '/')
    else:
        yaml_stem = os.path.splitext(os.path.basename(yaml_file_path))[0] if yaml_file_path else "waves"
        yaml_stem = yaml_stem.replace('.hydro', '')
        eta_file_name = f"{yaml_stem}_eta.txt"

    eta_path = os.path.join(yaml_dir, eta_file_name)
    # Do not create directories; the target must already exist
    eta_parent = os.path.dirname(eta_path)
    if eta_parent and not os.path.isdir(eta_parent):
        print(f"\nERROR: directory does not exist: {eta_parent}")
        print(f"       Create it first, or choose a different --elevation_file.")
        exit(1)

    with open(eta_path, 'w', newline='\n') as f:
        # No header, no comments, no trailing blank line: the C++ reader parses
        # every line with (ss >> t >> ':' >> eta) and throws on anything else
        f.write("\n".join(f"{t:.6f}:{e:.6f}" for t, e in zip(time_vec, eta_values)))
        f.write("\n")

    print(f"\nWrote eta file: {eta_path}")
    print(f"  {n_samples} samples, format 'time:elevation'")

    # BuildFromEtaFile only evaluates the orthogonal Fourier frequencies k/(N*dt) and
    # strides, dropping energy, when nfrequencies is below the in-band bin count.
    t_window = n_samples * dt
    df_natural = 1.0 / t_window
    k_min = max(1, int(np.ceil(0.001 / df_natural)))
    k_max = int(np.floor(min(1.0, 1.0 / (2.0 * dt)) / df_natural))
    eta_nfrequencies = max(1, k_max - k_min + 1)

    if args.eta_method == "dft":
        print(f"  nfrequencies: {eta_nfrequencies} (every Fourier bin below 1 Hz; "
              f"df = {df_natural:.3e} Hz)")
        if eta_nfrequencies > 4000:
            print(f"  WARNING: {eta_nfrequencies} wave components is a lot. The DFT is O(nf*N)")
            print(f"           and the GUI evaluates every component per surface vertex per frame.")
            print(f"           Shorten --elevation_duration or increase --elevation_dt to reduce it.")
    else:
        print(f"  method: irf_convolution (point elevation table, no spatial free surface)")

    # Keep the companion simulation YAML in sync: the wave field returns zero
    # elevation outside the table range, so end_time must not exceed the record.
    if sim_yaml_path is None:
        print(f"\nNOTE: no companion *.simulation.yaml found; timing not updated.")
    else:
        update_simulation_yaml(sim_yaml_path, end_time=duration,
                               time_step=args.elevation_dt,
                               reason=" s (matches eta record length)")


#~~~~~~~~~~~~~~ Update YAML file with calculated wave parameters ~~~~~~~~~~~~~~

if not skip_yaml_update and os.path.exists(yaml_file_path):
    print(f"\n\nUpdating YAML file: {yaml_file_path}")
    
    with open(yaml_file_path, 'r') as f:
        yaml_data = yaml.safe_load(f)
    
    if 'hydrodynamics' in yaml_data and 'waves' in yaml_data['hydrodynamics']:
        # With an eta file the sea state comes entirely from the record, so the waves
        # block is rebuilt rather than layered on top of what a previous run left.
        custom_eta = (wave_type == "irregular" and args.spectrum == "custom")

        yaml_data['hydrodynamics']['waves']['type'] = wave_type

        if not custom_eta:
            # Use override values if provided, otherwise use calculated values
            height_value = args.height if args.height is not None else Hs_rep
            period_value = args.period if args.period is not None else Te_rep

            yaml_data['hydrodynamics']['waves']['height'] = float(round(height_value, 2))
            yaml_data['hydrodynamics']['waves']['period'] = float(round(period_value, 2))
        
        # Handle spectrum: only for irregular waves
        if wave_type == "irregular":
            if args.spectrum == "custom":
                # dft -> BuildFromEtaFile gives discrete components and a spatial free
                # surface, so the GUI wireframe renders. irf -> EtaTableWaveField is a
                # point time series: kinematics are zero and no wireframe is drawn.
                waves_block = {'type': wave_type}

                if args.eta_method == "irf":
                    waves_block['method'] = "irf_convolution"
                    waves_block['eta_file'] = eta_file_name
                else:
                    # Legacy DFT path: waves.method is omitted entirely.
                    waves_block['eta_file'] = eta_file_name
                    # nfrequencies must span the default 0.001 - 1.0 Hz eta band; the
                    # C++ default of 1000 strides the grid and attenuates the record.
                    waves_block['nfrequencies'] = int(eta_nfrequencies)

                ramp = float(args.ramp_time) if args.ramp_time is not None else 60.0
                if ramp > 0.0:
                    waves_block['ramp_duration'] = ramp
                    waves_block['ramp_type'] = "cosine"

                if args.depth is not None:
                    # Only when asked for. With no depth key the DFT path assumes deep
                    # water and EtaTableWaveField falls back to the H5 water depth.
                    waves_block['depth'] = float(args.depth)

                yaml_data['hydrodynamics']['waves'] = waves_block
            elif args.spectrum:
                # Normalize spectrum name to lowercase single char or full name
                spectrum_type = args.spectrum.lower()
                if spectrum_type in ["pm", "pierson_moskowitz"]:
                    yaml_data['hydrodynamics']['waves']['spectrum'] = "pierson_moskowitz"
                    # For PM, use peak period (Tp) if not overridden
                    if args.period is None:
                        yaml_data['hydrodynamics']['waves']['period'] = float(round(Tp.mean(), 2))
                elif spectrum_type in ["jonswap", "js"]:
                    yaml_data['hydrodynamics']['waves']['spectrum'] = "jonswap"
                    # For JONSWAP, use peak period (Tp) if not overridden
                    if args.period is None:
                        yaml_data['hydrodynamics']['waves']['period'] = float(round(Tp.mean(), 2))
                    
                    # Check if gamma was provided as override
                    if args.gamma:
                        # Use first gamma value as override
                        yaml_data['hydrodynamics']['waves']['gamma'] = float(round(args.gamma[0], 3))
                    else:
                        # Calculate fitted gamma for JONSWAP
                        # IMPORTANT: Average spectrum first, then calculate Hm0/Tp (same as plotting)
                        avg_spectrum = all_swden_data.mean(axis=1)
                        avg_spectrum_clean = avg_spectrum.dropna()
                        
                        if len(avg_spectrum_clean) > 0:
                            # Use period override if provided, otherwise calculate from averaged spectrum
                            if args.period is not None:
                                Tp_mean = float(args.period)
                            else:
                                Tp_mean = float(wave.resource.peak_period(avg_spectrum_clean))
                            
                            # Use height override if provided, otherwise calculate from averaged spectrum
                            if args.height is not None:
                                Hs_mean = float(args.height)
                            else:
                                Hs_mean = float(wave.resource.significant_wave_height(avg_spectrum_clean))
                            
                            freq_array = avg_spectrum_clean.index.astype(float).values
                            gamma_fit, error, tp_fit, hit_bound = fit_jonswap_gamma(avg_spectrum_clean, freq_array, Tp_mean, Hs_mean)
                            yaml_data['hydrodynamics']['waves']['gamma'] = float(round(gamma_fit, 3))
                        else:
                            # Fallback to default gamma if no spectrum data
                            yaml_data['hydrodynamics']['waves']['gamma'] = 3.3
            else:
                # Default spectrum for irregular waves is PM
                yaml_data['hydrodynamics']['waves']['spectrum'] = "pierson_moskowitz"
                yaml_data['hydrodynamics']['waves']['period'] = float(round(Tp.mean(), 2))
        else:
            # Regular waves: deterministic, so spectrum shape keys do not apply
            for stale_key in ('spectrum', 'gamma'):
                if stale_key in yaml_data['hydrodynamics']['waves']:
                    del yaml_data['hydrodynamics']['waves'][stale_key]

        # Drop eta-import leftovers whenever a parametric spectrum is in use, so a
        # previous --spectrum custom run does not keep overriding the sea state.
        # ramp keys are re-added below when --ramp_time asks for them.
        if args.spectrum != "custom":
            for stale_key in ('eta_file', 'eta_file_path', 'method', 'nfrequencies',
                              'ramp_duration', 'ramp_type'):
                if stale_key in yaml_data['hydrodynamics']['waves']:
                    del yaml_data['hydrodynamics']['waves'][stale_key]
        

        if not custom_eta:
            # Add water depth if available
            if args.depth is not None:
                yaml_data['hydrodynamics']['waves']['depth'] = float(args.depth)
            elif water_depth:
                yaml_data['hydrodynamics']['waves']['depth'] = float(water_depth)

            # Add optional parameters if provided
            if args.seed is not None:
                yaml_data['hydrodynamics']['waves']['seed'] = args.seed

            if args.ramp_time is not None:
                # sea-stack reads waves.ramp_duration / waves.ramp_type; there is no
                # waves.ramp_time key.
                yaml_data['hydrodynamics']['waves']['ramp_duration'] = float(args.ramp_time)
                yaml_data['hydrodynamics']['waves']['ramp_type'] = "cosine"
        
        print(f"Updated wave parameters:")
        print(f"  Type: {yaml_data['hydrodynamics']['waves']['type']}")
        if custom_eta:
            pass
        elif args.height is not None:
            print(f"  Height: {yaml_data['hydrodynamics']['waves']['height']} m (OVERRIDE)")
        else:
            print(f"  Height: {yaml_data['hydrodynamics']['waves']['height']} m (from {Hs_rep:.4f} m)")
        
        # Period handling - varies by spectrum type
        if custom_eta:
            wb = yaml_data['hydrodynamics']['waves']
            print(f"  eta_file: {wb['eta_file']}")
            if 'method' in wb:
                print(f"  method: {wb['method']}")
            else:
                print(f"  nfrequencies: {wb['nfrequencies']}")
            if 'ramp_duration' in wb:
                print(f"  ramp: {wb['ramp_duration']} s ({wb['ramp_type']})")
            if 'depth' in wb:
                print(f"  depth: {wb['depth']} m")
        elif wave_type == "irregular" and args.spectrum:
            spectrum_type = args.spectrum.lower()
            if spectrum_type in ["pm", "pierson_moskowitz"]:
                if args.period is not None:
                    print(f"  Period: {yaml_data['hydrodynamics']['waves']['period']} s (OVERRIDE)")
                else:
                    print(f"  Period: {yaml_data['hydrodynamics']['waves']['period']} s (Tp, from {Tp.mean():.4f} s)")
            elif spectrum_type in ["jonswap", "js"]:
                if args.period is not None:
                    print(f"  Period: {yaml_data['hydrodynamics']['waves']['period']} s (OVERRIDE)")
                else:
                    print(f"  Period: {yaml_data['hydrodynamics']['waves']['period']} s (Tp, from {Tp.mean():.4f} s)")
                if 'gamma' in yaml_data['hydrodynamics']['waves']:
                    print(f"  Spectrum: JONSWAP")
                    gamma_source = "override" if args.gamma else "fitted"
                    print(f"  Gamma: {yaml_data['hydrodynamics']['waves']['gamma']} ({gamma_source} peak enhancement factor)")
        elif wave_type == "irregular":
            # Default PM for irregular
            print(f"  Period: {yaml_data['hydrodynamics']['waves']['period']} s (Tp, from {Tp.mean():.4f} s, default PM)")
        else:
            # Regular waves
            if args.period is not None:
                print(f"  Period: {yaml_data['hydrodynamics']['waves']['period']} s (OVERRIDE)")
            else:
                print(f"  Period: {yaml_data['hydrodynamics']['waves']['period']} s (from {Te_rep:.4f} s)")
        
        if 'spectrum' in yaml_data['hydrodynamics']['waves']:
            print(f"  Spectrum: {yaml_data['hydrodynamics']['waves']['spectrum']}")
        if not custom_eta:
            if 'depth' in yaml_data['hydrodynamics']['waves']:
                if args.depth is not None:
                    print(f"  Depth: {yaml_data['hydrodynamics']['waves']['depth']} m (OVERRIDE)")
                else:
                    print(f"  Depth: {yaml_data['hydrodynamics']['waves']['depth']} m")
            if args.seed is not None:
                print(f"  Seed: {yaml_data['hydrodynamics']['waves']['seed']}")
            if args.ramp_time is not None:
                print(f"  Ramp: {yaml_data['hydrodynamics']['waves']['ramp_duration']} s (cosine)")
        
        # with open(yaml_file_path, 'w') as f:
        #     yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)

        class IndentDumper(yaml.SafeDumper):
            def increase_indent(self, flow=False, indentless=False):
                return super().increase_indent(flow, False)
        
        with open(yaml_file_path, "w") as f:
            yaml.dump(
                yaml_data,
                f,
                Dumper=IndentDumper,
                default_flow_style=False,
                sort_keys=False,
            )

        print(f"\nYAML file updated successfully!")
    else:
        print("ERROR: 'hydrodynamics.waves' section not found in YAML file")
elif not skip_yaml_update:
    # Trying to update but file not found
    print(f"ERROR: YAML file not found at {yaml_file_path}")
    exit(1)

# Notify user if YAML was not updated
if skip_yaml_update:
    print(f"\nNo YAML update requested (plot only). Skipping YAML file update.")

# --elevation_duration / --elevation_dt set the simulation timing for every spectrum
# type. On the custom path they also size the eta record and were applied with it.
if (not skip_yaml_update and args.spectrum != "custom"
        and (args.elevation_duration is not None or args.elevation_dt is not None)):
    _sim_path = find_simulation_yaml(yaml_file_path)
    if _sim_path is None:
        print("\nNOTE: no companion *.simulation.yaml found; timing not updated.")
    else:
        update_simulation_yaml(_sim_path, end_time=args.elevation_duration,
                               time_step=args.elevation_dt)


#~~~~~~~~~~~~~~ Plot spectrum if requested ~~~~~~~~~~~~~~

# Handle spectrum plotting if requested
if args.plot_spectrum is not None:
    # If in extract_single or extract_daily mode, plot every extracted spectrum
    if extract_single or extract_daily:
        plt.figure(figsize=(12, 6))
        colors = plt.cm.tab10(np.linspace(0, 1, max(len(extracted_spectra), 1)))

        for idx, (label, spectrum) in enumerate(extracted_spectra):
            plt.plot(
                spectrum.index.astype(float),
                spectrum.values,
                label=label,
                linewidth=2.5,
                color=colors[idx],
                alpha=0.9
            )

        mode_name = "Measurement" if extract_single else "Daily Average"
        if len(extracted_spectra) == 1:
            title = plot_heading(extracted_spectra[0][0])
        else:
            title = f"{source_title} - {len(extracted_spectra)} {mode_name}s"

        plt.xlabel("Frequency [Hz]", fontsize=12)
        plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
        plt.title(title, fontsize=14)
        if len(extracted_spectra) > 1:
            plt.legend(fontsize=10, loc='upper right')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
        plt.close()
        print(f"Spectrum plot displayed")
    
    # One or more months requested: overlay each month's average spectrum
    elif parse_month_list(plot_spectrum_values, min_count=1):
        month_nums = parse_month_list(plot_spectrum_values, min_count=1)
        month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
        if not selected_years:
            print("No valid years found to plot")
        else:
            plot_year = selected_years[0]
            print(f"\nPlotting {len(month_nums)} monthly averaged spectra for {plot_year}...")

            if str(plot_year) not in ndbc_data:
                print(f"Year {plot_year} not found in available data")
            else:
                year_data = ndbc_data[str(plot_year)]
                plt.figure(figsize=(12, 6))
                colors = plt.cm.tab10(np.linspace(0, 1, max(len(month_nums), 1)))
                plotted_any = False

                for idx, month_num in enumerate(month_nums):
                    month_spectra = []
                    for timestamp, spectrum in year_data.items():
                        if timestamp.month == month_num:
                            spectrum_clean = spectrum.dropna()
                            if len(spectrum_clean) > 0:
                                month_spectra.append(spectrum_clean)

                    if not month_spectra:
                        print(f"  No data for {month_names[month_num-1]} {plot_year}")
                        continue

                    month_avg = pd.concat(month_spectra, axis=1).mean(axis=1)
                    month_avg_series = pd.Series(month_avg.values, index=month_avg.index)

                    hm0 = float(wave.resource.significant_wave_height(month_avg_series))
                    te = float(wave.resource.energy_period(month_avg_series))
                    tp = float(wave.resource.peak_period(month_avg_series))

                    label = month_names[month_num-1]
                    plt.plot(month_avg.index.astype(float), month_avg.values,
                             label=label, linewidth=2.5, color=colors[idx], alpha=0.9)

                    print(f"{label} {plot_year}: Hm0={hm0:6.2f} m | Te={te:6.2f} s | Tp={tp:6.2f} s")
                    plotted_any = True

                if plotted_any:
                    plt.xlabel("Frequency [Hz]", fontsize=12)
                    plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
                    plt.title(f"NDBC Buoy {buoy_number} - Monthly Averaged Spectra ({plot_year})", fontsize=14)
                    plt.legend(fontsize=10, loc='upper right')
                    plt.grid(True, alpha=0.3)
                    plt.tight_layout()
                    plt.show()
                    plt.close()
                    print(f"Spectrum plot displayed")
                else:
                    plt.close()
                    print("No month data available to plot")

    # Normal plotting logic (when not in extract mode)
    elif isinstance(args.plot_spectrum, str):
        # Plot monthly spectra for specified year
        year_for_monthly = args.plot_spectrum
        print(f"\nPlotting monthly averaged spectra for year {year_for_monthly}...")
        
        if year_for_monthly in ndbc_data:
            year_data = ndbc_data[year_for_monthly]
            
            # Group data by month
            monthly_spectra = {}
            for timestamp, spectrum in year_data.items():
                month = timestamp.month
                if month not in monthly_spectra:
                    monthly_spectra[month] = []
                spectrum_clean = spectrum.dropna()
                if len(spectrum_clean) > 0:
                    monthly_spectra[month].append(spectrum_clean)
            
            # Plot monthly spectra
            plt.figure(figsize=(12, 6))
            colors = plt.cm.viridis(np.linspace(0, 1, len(monthly_spectra)))
            month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
            
            print(f"\nMonthly Wave Metrics for {year_for_monthly}:")
            print("-" * 50)
            
            for idx, (month, spectra_list) in enumerate(sorted(monthly_spectra.items())):
                if len(spectra_list) > 0:
                    # Average spectra for this month
                    month_avg = pd.concat(spectra_list, axis=1).mean(axis=1)
                    
                    # Calculate wave metrics for this month
                    month_avg_series = pd.Series(month_avg.values, index=month_avg.index)
                    hm0 = wave.resource.significant_wave_height(month_avg_series)
                    te = wave.resource.energy_period(month_avg_series)
                    tp = wave.resource.peak_period(month_avg_series)
                    
                    print(f"{month_names[month-1]:>3}: Hm0={hm0:6.2f} m  |  Te={te:6.2f} s  |  Tp={tp:6.2f} s")
                    
                    plt.plot(
                        month_avg.index.astype(float),
                        month_avg.values,
                        label=f"{month_names[month-1]} (n={len(spectra_list)})",
                        linewidth=2,
                        color=colors[idx],
                        alpha=0.8
                    )
            
            plt.xlabel("Frequency [Hz]", fontsize=12)
            plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
            plt.title(f"NDBC Buoy {buoy_number} - Monthly Averaged Wave Spectra ({year_for_monthly})", fontsize=14)
            plt.legend(fontsize=10, loc='upper right')
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.show()
            plt.close()
            print(f"Monthly spectrum plot displayed")
        else:
            print(f"Year {year_for_monthly} not found in available data")
    
    elif args.plot_spectrum is True:
        if selected_years:
            plt.figure(figsize=(12, 6))
            colors = plt.cm.viridis(np.linspace(0, 1, len(selected_years)))
            
            all_year_avg_spectra = []
            
            for idx, year in enumerate(selected_years):
                year_key = str(year)
                if year_key in ndbc_data:
                    year_data = ndbc_data[year_key]
                    year_avg_spectrum = year_data.mean(axis=1)
                    year_avg_spectrum = year_avg_spectrum.dropna()
                    all_year_avg_spectra.append(year_avg_spectrum)
                    plt.plot(
                        year_avg_spectrum.index.astype(float),
                        year_avg_spectrum.values,
                        label=f"Year {year}",
                        linewidth=2,
                        color=colors[idx],
                        alpha=0.8
                    )
            
            if len(all_year_avg_spectra) > 1:
                overall_avg_spectrum = pd.concat(all_year_avg_spectra, axis=1).mean(axis=1)
                plt.plot(
                    overall_avg_spectrum.index.astype(float),
                    overall_avg_spectrum.values,
                    label="Overall Average",
                    linewidth=2.5,
                    linestyle="--",
                    color="red",
                    alpha=0.9
                )
            
            plt.xlabel("Frequency [Hz]", fontsize=12)
            plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
            plt.title(f"NDBC Buoy {buoy_number} - Year-Averaged Wave Spectra", fontsize=14)
            plt.legend(fontsize=10)
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.show()
            plt.close()
        else:
            print("No valid years found to plot")


#~~~~~~~~~~~~~~ Plot spectrum vs Pierson-Moskowitz if requested ~~~~~~~~~~~~~~

if args.plot_spectrum_pm is not None:
    # If in extract_single or extract_daily mode, plot each extracted spectrum with its PM estimate
    if extract_single or extract_daily:
        plt.figure(figsize=(12, 6))
        colors = plt.cm.tab10(np.linspace(0, 1, max(len(extracted_spectra), 1)))
        multi = len(extracted_spectra) > 1

        for idx, (label, spectrum) in enumerate(extracted_spectra):
            spectrum_series = pd.Series(spectrum.values, index=spectrum.index)

            hm0 = float(wave.resource.significant_wave_height(spectrum_series))
            te = float(wave.resource.energy_period(spectrum_series))
            tp = float(wave.resource.peak_period(spectrum_series))

            # Use override values if provided
            hm0_plot = float(args.height) if args.height is not None else hm0
            tp_plot = float(args.period) if args.period is not None else tp

            freq = spectrum.index.astype(float).values

            measured_label = f"{label} (measured)" if multi else "Measured"
            plt.plot(
                freq,
                spectrum.values,
                label=measured_label,
                linewidth=2.5,
                color=colors[idx],
                alpha=0.9
            )

            pm_spec = pierson_moskowitz_spectrum(freq, hm0_plot, tp_plot)
            pm_label = f"{label} PM" if multi else f"Pierson-Moskowitz (Hm0={hm0_plot:.2f} m, Tp={tp_plot:.2f} s)"
            plt.plot(
                freq,
                pm_spec,
                label=pm_label,
                linewidth=2.0,
                color=colors[idx],
                alpha=0.75,
                linestyle='--'
            )

            error = float(np.sum((spectrum.values - pm_spec) ** 2))
            print(f"\n{label}")
            print(f"Hm0={hm0:.2f} m  |  Te={te:.2f} s  |  Tp={tp:.2f} s")
            print(f"Pierson-Moskowitz: Error={error:.6e}")

        mode_name = "Measurement" if extract_single else "Daily Average"
        if multi:
            title = f"{source_title} - {len(extracted_spectra)} {mode_name}s vs Pierson-Moskowitz"
        else:
            title = plot_heading(extracted_spectra[0][0], "Pierson-Moskowitz")

        plt.xlabel("Frequency [Hz]", fontsize=12)
        plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
        plt.title(title, fontsize=14)
        plt.legend(fontsize=9, loc='upper right', ncol=2 if multi else 1)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
        plt.close()
        print(f"\nPierson-Moskowitz spectrum plot displayed")

    # Multiple months requested: overlay each month's average with its PM estimate
    elif parse_month_list(plot_spectrum_pm_values):
        month_nums = parse_month_list(plot_spectrum_pm_values)
        month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
        if not selected_years:
            print("No valid years found to plot")
        else:
            plot_year = selected_years[0]
            print(f"\nPlotting spectrum vs Pierson-Moskowitz for {len(month_nums)} months of {plot_year}...")

            if str(plot_year) not in ndbc_data:
                print(f"Year {plot_year} not found in available data")
            else:
                year_data = ndbc_data[str(plot_year)]
                plt.figure(figsize=(12, 6))
                colors = plt.cm.tab10(np.linspace(0, 1, max(len(month_nums), 1)))
                plotted_any = False

                for idx, month_num in enumerate(month_nums):
                    month_spectra = []
                    for timestamp, spectrum in year_data.items():
                        if timestamp.month == month_num:
                            spectrum_clean = spectrum.dropna()
                            if len(spectrum_clean) > 0:
                                month_spectra.append(spectrum_clean)

                    if not month_spectra:
                        print(f"  No data for {month_names[month_num-1]} {plot_year}")
                        continue

                    month_avg = pd.concat(month_spectra, axis=1).mean(axis=1)
                    month_avg_series = pd.Series(month_avg.values, index=month_avg.index)

                    hm0 = float(wave.resource.significant_wave_height(month_avg_series))
                    te = float(wave.resource.energy_period(month_avg_series))
                    tp = float(wave.resource.peak_period(month_avg_series))

                    hm0_plot = float(args.height) if args.height is not None else hm0
                    tp_plot = float(args.period) if args.period is not None else tp

                    freq = month_avg.index.astype(float).values
                    label = month_names[month_num-1]

                    plt.plot(freq, month_avg.values, label=f"{label} (measured)",
                             linewidth=2.5, color=colors[idx], alpha=0.9)

                    pm_spec = pierson_moskowitz_spectrum(freq, hm0_plot, tp_plot)
                    plt.plot(freq, pm_spec, label=f"{label} PM",
                             linewidth=2.0, color=colors[idx], alpha=0.75, linestyle='--')

                    error = float(np.sum((month_avg.values - pm_spec) ** 2))
                    print(f"{label} {plot_year}: Hm0={hm0:6.2f} m | Te={te:6.2f} s | Tp={tp:6.2f} s | Error={error:.6e}")
                    plotted_any = True

                if plotted_any:
                    plt.xlabel("Frequency [Hz]", fontsize=12)
                    plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
                    plt.title(f"NDBC Buoy {buoy_number} - Monthly Measured vs Pierson-Moskowitz ({plot_year})",
                              fontsize=14)
                    plt.legend(fontsize=9, loc='upper right', ncol=2)
                    plt.grid(True, alpha=0.3)
                    plt.tight_layout()
                    plt.show()
                    plt.close()
                    print(f"\nPierson-Moskowitz spectrum plot displayed")
                else:
                    plt.close()
                    print("No month data available to plot")

    # Check if year or month was provided as argument
    elif isinstance(args.plot_spectrum_pm, str):
        # Determine if input is a month or year
        month_num, is_month = parse_month(args.plot_spectrum_pm)
        
        if is_month:
            # Plot specific month for the selected year
            month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
            if selected_years:
                plot_year = selected_years[0]
                print(f"\nPlotting spectrum vs Pierson-Moskowitz for {month_names[month_num-1]} {plot_year}...")
                
                if str(plot_year) in ndbc_data:
                    year_data = ndbc_data[str(plot_year)]
                    
                    # Filter for just this month
                    month_spectra = []
                    for timestamp, spectrum in year_data.items():
                        if timestamp.month == month_num:
                            spectrum_clean = spectrum.dropna()
                            if len(spectrum_clean) > 0:
                                month_spectra.append(spectrum_clean)
                    
                    if month_spectra:
                        # Average spectra for this month
                        month_avg = pd.concat(month_spectra, axis=1).mean(axis=1)
                        month_avg_series = pd.Series(month_avg.values, index=month_avg.index)
                        
                        # Calculate wave metrics
                        hm0 = wave.resource.significant_wave_height(month_avg_series)
                        te = wave.resource.energy_period(month_avg_series)
                        tp = wave.resource.peak_period(month_avg_series)
                        
                        # Use override values if provided
                        hm0_plot = args.height if args.height is not None else hm0
                        tp_plot = args.period if args.period is not None else tp
                        
                        print(f"{month_names[month_num-1]} {plot_year}: Hm0={hm0:.2f} m  |  Te={te:.2f} s  |  Tp={tp:.2f} s")
                        if args.height is not None or args.period is not None:
                            print(f"  (Using overrides: Hm0={hm0_plot} m, Tp={tp_plot} s)")
                        
                        # Plot measured vs PM
                        plt.figure(figsize=(12, 6))
                        
                        plt.plot(
                            month_avg.index.astype(float),
                            month_avg.values,
                            label=f"{month_names[month_num-1]} (measured)",
                            linewidth=2.5,
                            color='steelblue',
                            alpha=0.9
                        )
                        
                        # Calculate and plot PM spectrum
                        freq = month_avg.index.astype(float)
                        pm_curve = pierson_moskowitz_spectrum(freq, hm0_plot, tp_plot)
                        
                        plt.plot(
                            freq,
                            pm_curve,
                            label=f"{month_names[month_num-1]} (PM)",
                            linewidth=1.5,
                            color='coral',
                            alpha=0.7,
                            linestyle='--'
                        )
                        
                        plt.xlabel("Frequency [Hz]", fontsize=12)
                        plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
                        plt.title(f"NDBC Buoy {buoy_number} - {month_names[month_num-1]} {plot_year} Measured vs Pierson-Moskowitz Spectrum", fontsize=14)
                        plt.legend(fontsize=10, loc='upper right')
                        plt.grid(True, alpha=0.3)
                        plt.tight_layout()
                        plt.show()
                        plt.close()
                    else:
                        print(f"No data found for {month_names[month_num-1]} {plot_year}")
                else:
                    print(f"Year {plot_year} not found in available data")
            else:
                print("No years found to plot")
        else:
            # Plot monthly spectra vs PM for specified year
            year_for_pm = args.plot_spectrum_pm
            print(f"\nPlotting spectrum vs Pierson-Moskowitz for year {year_for_pm}...")
            
            if year_for_pm in ndbc_data:
                year_data = ndbc_data[year_for_pm]
                
                # Group data by month
                monthly_spectra = {}
                for timestamp, spectrum in year_data.items():
                    month = timestamp.month
                    if month not in monthly_spectra:
                        monthly_spectra[month] = []
                    spectrum_clean = spectrum.dropna()
                    if len(spectrum_clean) > 0:
                        monthly_spectra[month].append(spectrum_clean)
                
                # Plot monthly spectra vs PM
                plt.figure(figsize=(14, 8))
                colors = plt.cm.viridis(np.linspace(0, 1, len(monthly_spectra)))
                month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
                
                print(f"\nMonthly Spectrum Comparison for {year_for_pm}:") 
                print("-" * 70)
                
                for idx, (month, spectra_list) in enumerate(sorted(monthly_spectra.items())):
                    if len(spectra_list) > 0:
                        # Average spectra for this month
                        month_avg = pd.concat(spectra_list, axis=1).mean(axis=1)
                        month_avg_series = pd.Series(month_avg.values, index=month_avg.index)
                        
                        # Calculate wave metrics
                        hm0 = wave.resource.significant_wave_height(month_avg_series)
                        te = wave.resource.energy_period(month_avg_series)
                        tp = wave.resource.peak_period(month_avg_series)
                        
                        # Use override values if provided
                        hm0_plot = args.height if args.height is not None else hm0
                        tp_plot = args.period if args.period is not None else tp
                        
                        print(f"{month_names[month-1]:>3}: Hm0={hm0:6.2f} m  |  Te={te:6.2f} s  |  Tp={tp:6.2f} s")
                        if args.height is not None or args.period is not None:
                            print(f"       (Using overrides: Hm0={hm0_plot} m, Tp={tp_plot} s)")
                        
                        # Plot measured spectrum
                        plt.plot(
                            month_avg.index.astype(float),
                            month_avg.values,
                            label=f"{month_names[month-1]} (measured)",
                            linewidth=2.5,
                            color=colors[idx],
                            alpha=0.9
                        )
                        
                        # Calculate and plot PM spectrum (using Tp, not Te)
                        freq = month_avg.index.astype(float)
                        pm_curve = pierson_moskowitz_spectrum(freq, hm0_plot, tp_plot)
                        
                        # Diagnostic: check spectrum properties
                        # Integrate to verify Hm0 matches
                        df = freq[1] - freq[0] if len(freq) > 1 else 1.0
                        m0_measured = integrate.trapezoid(month_avg.values, freq)
                        m0_pm = integrate.trapezoid(pm_curve, freq)
                        hm0_from_m0_measured = 4 * np.sqrt(m0_measured)
                        hm0_from_m0_pm = 4 * np.sqrt(m0_pm)
                        
                        print(f"  Diagnostics: m0_meas={m0_measured:.4f}, Hm0_calc={hm0_from_m0_measured:.2f} vs input={hm0:.2f}")
                        print(f"  PM diagnostics: m0_pm={m0_pm:.4f}, Hm0_pm={hm0_from_m0_pm:.2f}, peak_meas={month_avg.max():.4f}, peak_pm={pm_curve.max():.4f}")
                        
                        plt.plot(
                            freq,
                            pm_curve,
                            label=f"{month_names[month-1]} (PM)",
                            linewidth=1.5,
                            color=colors[idx],
                            alpha=0.5,
                            linestyle='--'
                        )
                
                plt.xlabel("Frequency [Hz]", fontsize=12)
                plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
                plt.title(f"NDBC Buoy {buoy_number} - Monthly Measured vs Pierson-Moskowitz Spectra ({year_for_pm})", fontsize=14)
                plt.legend(fontsize=9, loc='upper right', ncol=2)
                plt.grid(True, alpha=0.3)
                plt.tight_layout()
                plt.show()
                plt.close()
            else:
                print(f"Year {year_for_pm} not found in available data")
    
    elif args.plot_spectrum_pm is True:
        # Plot year-averaged spectra vs PM
        if selected_years:
            plt.figure(figsize=(14, 8))
            colors = plt.cm.viridis(np.linspace(0, 1, len(selected_years)))
            
            all_year_avg_spectra = []
            
            print(f"\nYear-Averaged Spectrum Comparison:")
            print("-" * 70)
            
            for idx, year in enumerate(selected_years):
                year_key = str(year)
                if year_key in ndbc_data:
                    year_data = ndbc_data[year_key]
                    year_avg_spectrum = year_data.mean(axis=1)
                    year_avg_spectrum = year_avg_spectrum.dropna()
                    year_avg_series = pd.Series(year_avg_spectrum.values, index=year_avg_spectrum.index)
                    
                    # Calculate wave metrics
                    hm0 = wave.resource.significant_wave_height(year_avg_series)
                    te = wave.resource.energy_period(year_avg_series)
                    tp = wave.resource.peak_period(year_avg_series)
                    
                    # Use override values if provided
                    hm0_plot = args.height if args.height is not None else hm0
                    tp_plot = args.period if args.period is not None else tp
                    
                    print(f"Year {year}: Hm0={hm0:6.2f} m  |  Te={te:6.2f} s  |  Tp={tp:6.2f} s")
                    if args.height is not None or args.period is not None:
                        print(f"  (Using overrides: Hm0={hm0_plot} m, Tp={tp_plot} s)")
                    
                    all_year_avg_spectra.append(year_avg_spectrum)
                    
                    # Plot measured spectrum
                    plt.plot(
                        year_avg_spectrum.index.astype(float),
                        year_avg_spectrum.values,
                        label=f"Year {year} (measured)",
                        linewidth=2.5,
                        color=colors[idx],
                        alpha=0.9
                    )
                    
                    # Calculate and plot PM spectrum (using Tp, not Te)
                    freq = year_avg_spectrum.index.astype(float)
                    pm_curve = pierson_moskowitz_spectrum(freq, hm0_plot, tp_plot)
                    plt.plot(
                        freq,
                        pm_curve,
                        label=f"Year {year} (PM)",
                        linewidth=1.5,
                        color=colors[idx],
                        alpha=0.5,
                        linestyle='--'
                    )
            
            if len(all_year_avg_spectra) > 1:
                overall_avg_spectrum = pd.concat(all_year_avg_spectra, axis=1).mean(axis=1)
                overall_avg_series = pd.Series(overall_avg_spectrum.values, index=overall_avg_spectrum.index)
                
                hm0_overall = wave.resource.significant_wave_height(overall_avg_series)
                te_overall = wave.resource.energy_period(overall_avg_series)
                tp_overall = wave.resource.peak_period(overall_avg_series)
                
                # Use override values if provided
                hm0_overall_plot = args.height if args.height is not None else hm0_overall
                tp_overall_plot = args.period if args.period is not None else tp_overall
                
                print(f"Overall: Hm0={hm0_overall:6.2f} m  |  Te={te_overall:6.2f} s  |  Tp={tp_overall:6.2f} s")
                if args.height is not None or args.period is not None:
                    print(f"  (Using overrides: Hm0={hm0_overall_plot} m, Tp={tp_overall_plot} s)")
                
                plt.plot(
                    overall_avg_spectrum.index.astype(float),
                    overall_avg_spectrum.values,
                    label="Overall Average (measured)",
                    linewidth=3,
                    linestyle="-",
                    color="black",
                    alpha=0.9
                )
                
                # PM for overall average (using Tp, not Te)
                freq = overall_avg_spectrum.index.astype(float)
                pm_overall = pierson_moskowitz_spectrum(freq, hm0_overall_plot, tp_overall_plot)
                plt.plot(
                    freq,
                    pm_overall,
                    label="Overall Average (PM)",
                    linewidth=2,
                    linestyle="--",
                    color="black",
                    alpha=0.5
                )
            
            plt.xlabel("Frequency [Hz]", fontsize=12)
            plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
            plt.title(f"NDBC Buoy {buoy_number} - Year-Averaged Measured vs Pierson-Moskowitz Spectra", fontsize=14)
            plt.legend(fontsize=10, loc='upper right', ncol=2)
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.show()
            plt.close()
        else:
            print("No valid years found to plot")


#~~~~~~~~~~~~~~ Plot JONSWAP spectrum if requested ~~~~~~~~~~~~~~

if args.plot_spectrum_js is not None:
    # If in extract_single or extract_daily mode, plot each extracted spectrum with its own JONSWAP fit
    if extract_single or extract_daily:
        plt.figure(figsize=(12, 6))
        colors = plt.cm.tab10(np.linspace(0, 1, max(len(extracted_spectra), 1)))
        multi = len(extracted_spectra) > 1

        for idx, (label, spectrum) in enumerate(extracted_spectra):
            spectrum_series = pd.Series(spectrum.values, index=spectrum.index)

            # Calculate wave metrics
            hm0 = float(wave.resource.significant_wave_height(spectrum_series))
            te = float(wave.resource.energy_period(spectrum_series))
            tp = float(wave.resource.peak_period(spectrum_series))

            # Use override values if provided
            hm0_plot = float(args.height) if args.height is not None else hm0
            tp_plot = float(args.period) if args.period is not None else tp

            # Fit JONSWAP gamma (and refine Tp, since the measured peak bin is noisy)
            freq = spectrum.index.astype(float).values
            gamma_fit, error, tp_fit, hit_bound = fit_jonswap_gamma(
                spectrum_series, freq, tp_plot, hm0_plot)

            measured_label = f"{label} (measured)" if multi else "Measured"
            plt.plot(
                freq,
                spectrum.values,
                label=measured_label,
                linewidth=2.5,
                color=colors[idx],
                alpha=0.9
            )

            # Generate fitted JONSWAP spectrum
            js_spectrum = jonswap_spectrum_custom(freq, tp_fit, hm0_plot, gamma=gamma_fit)

            js_label = (f"{label} JONSWAP (γ={gamma_fit:.3f}, Tp={tp_fit:.2f} s)" if multi
                        else f"JONSWAP fit (γ={gamma_fit:.3f}, Tp={tp_fit:.2f} s)")
            plt.plot(
                freq,
                js_spectrum,
                label=js_label,
                linewidth=2.0,
                color=colors[idx],
                alpha=0.75,
                linestyle='--'
            )

            print(f"\n{label}")
            print(f"Hm0={hm0:.2f} m  |  Te={te:.2f} s  |  Tp={tp:.2f} s")
            print(f"Fitted JONSWAP: gamma={gamma_fit:.3f}, Tp={tp_fit:.2f} s (measured {tp_plot:.2f} s), Error={error:.6e}")
            if hit_bound:
                print(f"  NOTE: gamma is pinned at the {GAMMA_MIN}-{GAMMA_MAX} bound. The measured")
                print(f"        spectrum is likely broad or multi-peaked (swell + wind sea), which a")
                print(f"        single-peak JONSWAP cannot represent. Check Te vs Tp: Te > Tp usually")
                print(f"        indicates a second low-frequency peak.")

            # Manual gamma overlays, drawn at the measured Hm0/Tp (or the overrides)
            if args.gamma:
                gamma_colors = ['lime', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
                gamma_linestyles = [':', '-.', '--', '-']
                for g_idx, gamma_val in enumerate(args.gamma):
                    gamma_val = float(gamma_val)
                    if gamma_val < GAMMA_MIN:
                        print(f"  ERROR: gamma={gamma_val:.3f} < {GAMMA_MIN} - outside valid range (skipping)")
                        continue

                    js_manual = jonswap_spectrum_custom(freq, tp_plot, hm0_plot, gamma=gamma_val)
                    if np.any(np.isnan(js_manual)) or np.any(np.isinf(js_manual)):
                        print(f"  WARNING: gamma={gamma_val:.3f} produced NaN/inf values")
                        continue

                    error_vs_fitted = float(np.sum((js_spectrum - js_manual) ** 2))
                    error_vs_measured = float(np.sum((spectrum.values - js_manual) ** 2))
                    manual_label = (f"{label} manual γ={gamma_val:.3f}" if multi
                                    else f"Manual γ={gamma_val:.3f}")
                    plt.plot(
                        freq,
                        js_manual,
                        label=f"{manual_label} (err={error_vs_measured:.2e})",
                        linewidth=1.6,
                        color=gamma_colors[g_idx % len(gamma_colors)],
                        alpha=0.9,
                        linestyle=gamma_linestyles[g_idx % len(gamma_linestyles)]
                    )
                    print(f"  Manual gamma={gamma_val:.3f}: Error vs measured = {error_vs_measured:.6e}, "
                          f"vs fitted = {error_vs_fitted:.6e}")

        mode_name = "Measurement" if extract_single else "Daily Average"
        if multi:
            title = f"{source_title} - {len(extracted_spectra)} {mode_name}s vs Fitted JONSWAP"
        else:
            title = plot_heading(extracted_spectra[0][0], "Fitted JONSWAP")

        plt.xlabel("Frequency [Hz]", fontsize=12)
        plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
        plt.title(title, fontsize=14)
        plt.legend(fontsize=9, loc='upper right', ncol=2 if multi else 1)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
        plt.close()
        print(f"\nJONSWAP spectrum plot displayed")
    
    # Multiple months requested: overlay each month's average with its own JONSWAP fit
    elif parse_month_list(plot_spectrum_js_values):
        month_nums = parse_month_list(plot_spectrum_js_values)
        month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
        if not selected_years:
            print("No valid years found to plot")
        else:
            plot_year = selected_years[0]
            print(f"\nPlotting spectrum vs fitted JONSWAP for {len(month_nums)} months of {plot_year}...")

            if str(plot_year) not in ndbc_data:
                print(f"Year {plot_year} not found in available data")
            else:
                year_data = ndbc_data[str(plot_year)]
                plt.figure(figsize=(12, 6))
                colors = plt.cm.tab10(np.linspace(0, 1, max(len(month_nums), 1)))
                plotted_any = False

                for idx, month_num in enumerate(month_nums):
                    month_spectra = []
                    for timestamp, spectrum in year_data.items():
                        if timestamp.month == month_num:
                            spectrum_clean = spectrum.dropna()
                            if len(spectrum_clean) > 0:
                                month_spectra.append(spectrum_clean)

                    if not month_spectra:
                        print(f"  No data for {month_names[month_num-1]} {plot_year}")
                        continue

                    month_avg = pd.concat(month_spectra, axis=1).mean(axis=1)
                    month_avg_series = pd.Series(month_avg.values, index=month_avg.index)

                    hm0 = float(wave.resource.significant_wave_height(month_avg_series))
                    te = float(wave.resource.energy_period(month_avg_series))
                    tp = float(wave.resource.peak_period(month_avg_series))

                    hm0_plot = float(args.height) if args.height is not None else hm0
                    tp_plot = float(args.period) if args.period is not None else tp

                    freq = month_avg.index.astype(float).values
                    gamma_fit, error, tp_fit, hit_bound = fit_jonswap_gamma(
                        month_avg_series, freq, tp_plot, hm0_plot)

                    label = month_names[month_num-1]
                    plt.plot(freq, month_avg.values, label=f"{label} (measured)",
                             linewidth=2.5, color=colors[idx], alpha=0.9)

                    js_spectrum = jonswap_spectrum_custom(freq, tp_fit, hm0_plot, gamma=gamma_fit)
                    plt.plot(freq, js_spectrum,
                             label=f"{label} JONSWAP (γ={gamma_fit:.3f})",
                             linewidth=2.0, color=colors[idx], alpha=0.75, linestyle='--')

                    print(f"{label} {plot_year}: Hm0={hm0:6.2f} m | Te={te:6.2f} s | Tp={tp:6.2f} s | "
                          f"gamma={gamma_fit:.3f} | Tp_fit={tp_fit:6.2f} s | Error={error:.6e}")
                    plotted_any = True

                if plotted_any:
                    plt.xlabel("Frequency [Hz]", fontsize=12)
                    plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
                    plt.title(f"NDBC Buoy {buoy_number} - Monthly Measured vs Fitted JONSWAP ({plot_year})",
                              fontsize=14)
                    plt.legend(fontsize=9, loc='upper right', ncol=2)
                    plt.grid(True, alpha=0.3)
                    plt.tight_layout()
                    plt.show()
                    plt.close()
                    print(f"\nJONSWAP spectrum plot displayed")
                else:
                    plt.close()
                    print("No month data available to plot")

    # Normal plotting logic (when not in extract mode)
    elif isinstance(args.plot_spectrum_js, str):
        # Determine if input is a month or year
        month_num, is_month = parse_month(args.plot_spectrum_js)
        
        if is_month:
            # Plot specific month for the selected year
            month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
            if selected_years:
                plot_year = selected_years[0]
                print(f"\nPlotting spectrum vs fitted JONSWAP for {month_names[month_num-1]} {plot_year}...")
                
                if str(plot_year) in ndbc_data:
                    year_data = ndbc_data[str(plot_year)]
                    
                    # Filter for just this month
                    month_spectra = []
                    for timestamp, spectrum in year_data.items():
                        if timestamp.month == month_num:
                            spectrum_clean = spectrum.dropna()
                            if len(spectrum_clean) > 0:
                                month_spectra.append(spectrum_clean)
                    
                    if month_spectra:
                        # Average spectra for this month
                        month_avg = pd.concat(month_spectra, axis=1).mean(axis=1)
                        month_avg_series = pd.Series(month_avg.values, index=month_avg.index)
                        
                        # Calculate wave metrics (ensure they're Python floats, not numpy types)
                        hm0 = float(wave.resource.significant_wave_height(month_avg_series))
                        te = float(wave.resource.energy_period(month_avg_series))
                        tp = float(wave.resource.peak_period(month_avg_series))
                        
                        print(f"{month_names[month_num-1]} {plot_year}: Hm0={hm0:.2f} m  |  Te={te:.2f} s  |  Tp={tp:.2f} s")
                        
                        # Fit JONSWAP gamma to MEASURED spectrum (NOT using overrides)
                        freq = month_avg.index.astype(float).values
                        gamma_fit, error, tp_fit, hit_bound = fit_jonswap_gamma(month_avg_series, freq, tp, hm0)
                        print(f"Fitted JONSWAP: gamma={gamma_fit:.3f}  |  Tp={tp_fit:.2f} s  |  Error={error:.6e}")
                        
                        # Plot measured vs JONSWAP
                        plt.figure(figsize=(12, 6))
                        
                        plt.plot(
                            freq,
                            month_avg.values,
                            label=f"{month_names[month_num-1]} (measured)",
                            linewidth=2.5,
                            color='steelblue',
                            alpha=0.9
                        )
                        
                        # Generate fitted JONSWAP spectrum using MEASURED frequency grid
                        js_spectrum = jonswap_spectrum_custom(freq, tp, hm0, gamma=gamma_fit)
                        
                        plt.plot(
                            freq,
                            js_spectrum,
                            label=f"{month_names[month_num-1]} (JONSWAP fit, γ={gamma_fit:.3f})",
                            linewidth=2.8,
                            color='coral',
                            alpha=0.9,
                            linestyle='--'
                        )
                        
                        # Plot manual gammas if provided (support multiple)
                        # These use override values if provided, otherwise measured values
                        if args.gamma:
                            hm0_manual = float(args.height) if args.height is not None else hm0
                            tp_manual = float(args.period) if args.period is not None else tp
                            print(f"  Manual gamma: Using Hm0={hm0_manual:.3f} m, Tp={tp_manual:.3f} s (from {'override' if args.height is not None else 'measured data'})")
                            gamma_colors = ['lime', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
                            gamma_linestyles = [':', '-.', '--', '-', ':', '-.', '--', '-']
                            print(f"\nManual gamma vs fitted comparison ({month_names[month_num-1]}):")
                            print(f"DEBUG: args.gamma = {args.gamma}, type = {type(args.gamma)}")
                            
                            for idx, gamma_val in enumerate(args.gamma):
                                print(f"DEBUG: Processing gamma_val = {gamma_val}, type = {type(gamma_val)}")
                                if isinstance(gamma_val, list):
                                    continue
                                gamma_val = float(gamma_val)
                                
                                # JONSWAP constrained to gamma >= GAMMA_MIN for physical validity
                                if gamma_val < GAMMA_MIN:
                                    print(f"  ERROR: gamma={gamma_val:.3f} < {GAMMA_MIN} - outside valid range (skipping)")
                                    continue
                                
                                try:
                                    js_manual = jonswap_spectrum_custom(freq, tp_manual, hm0_manual, gamma=gamma_val)
                                    # Check for NaN or inf values
                                    if np.any(np.isnan(js_manual)) or np.any(np.isinf(js_manual)):
                                        print(f"  WARNING: γ={gamma_val:.3f} produced NaN/inf values")
                                        continue
                                    error_vs_fitted = np.sum((js_spectrum - js_manual) ** 2)
                                    error_vs_measured = np.sum((month_avg.values - js_manual) ** 2)
                                    
                                    color = gamma_colors[idx % len(gamma_colors)]
                                    linestyle = gamma_linestyles[idx % len(gamma_linestyles)]
                                    
                                    plt.plot(
                                        freq,
                                        js_manual,
                                        label=f"{month_names[month_num-1]} (manual γ={gamma_val:.3f}, err={error_vs_fitted:.2e})",
                                        linewidth=1.8,
                                        color=color,
                                        alpha=0.95,
                                        linestyle=linestyle
                                    )
                                    print(f"  Manual γ={gamma_val:.3f}: Error vs fitted = {error_vs_fitted:.6e}, Error vs measured = {error_vs_measured:.6e}")
                                except Exception as e:
                                    print(f"  ERROR with γ={gamma_val:.3f}: {e}")
                                    import traceback
                                    traceback.print_exc()
                        
                        plt.xlabel("Frequency [Hz]", fontsize=12)
                        plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
                        plt.title(f"NDBC Buoy {buoy_number} - {month_names[month_num-1]} {plot_year} Measured vs Fitted JONSWAP Spectrum", fontsize=14)
                        plt.legend(fontsize=10, loc='upper right', ncol=2)
                        plt.grid(True, alpha=0.3)
                        plt.tight_layout()
                        
                        # Save to file before showing
                        plot_filename = f"spectrum_jonswap_{month_names[month_num-1].lower()}_{plot_year}.png"
                        plt.savefig(plot_filename, dpi=150)
                        print(f"✓ Plot saved: {plot_filename}")
                        
                        plt.show()
                        plt.close()
                    else:
                        print(f"No data found for {month_names[month_num-1]} {plot_year}")
                else:
                    print(f"Year {plot_year} not found in available data")
            else:
                print("No years found to plot")
        else:
            # Plot monthly spectra vs fitted JONSWAP for specified year
            year_for_js = args.plot_spectrum_js
            print(f"\nPlotting spectrum vs fitted JONSWAP for year {year_for_js}...")
            
            if year_for_js in ndbc_data:
                year_data = ndbc_data[year_for_js]
                
                # Group data by month
                monthly_spectra = {}
                for timestamp, spectrum in year_data.items():
                    month = timestamp.month
                    if month not in monthly_spectra:
                        monthly_spectra[month] = []
                    spectrum_clean = spectrum.dropna()
                    if len(spectrum_clean) > 0:
                        monthly_spectra[month].append(spectrum_clean)
                
                # Plot monthly spectra vs JONSWAP
                plt.figure(figsize=(14, 8))
                colors = plt.cm.viridis(np.linspace(0, 1, len(monthly_spectra)))
                month_names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
                
                print(f"\nMonthly Spectrum Comparison for {year_for_js}:") 
                print("-" * 80)
                
                for idx, (month, spectra_list) in enumerate(sorted(monthly_spectra.items())):
                    if len(spectra_list) > 0:
                        # Average spectra for this month
                        month_avg = pd.concat(spectra_list, axis=1).mean(axis=1)
                        month_avg_series = pd.Series(month_avg.values, index=month_avg.index)
                        
                        # Calculate wave metrics (ensure they're Python floats, not numpy types)
                        hm0 = float(wave.resource.significant_wave_height(month_avg_series))
                        te = float(wave.resource.energy_period(month_avg_series))
                        tp = float(wave.resource.peak_period(month_avg_series))
                        
                        # Use override values if provided
                        hm0_plot = float(args.height) if args.height is not None else hm0
                        tp_plot = float(args.period) if args.period is not None else tp
                        
                        # Fit JONSWAP gamma (peak enhancement)
                        freq = month_avg.index.astype(float).values
                        gamma_fit, error, tp_fit, hit_bound = fit_jonswap_gamma(month_avg_series, freq, tp_plot, hm0_plot)
                        
                        print(f"{month_names[month-1]:>3}: Hm0={hm0:6.2f} m  |  Te={te:6.2f} s  |  Tp={tp:6.2f} s | gamma={gamma_fit:.3f} | Tp_fit={tp_fit:6.2f} s | Error={error:.6e}")
                        if args.height is not None or args.period is not None:
                            print(f"       (Using overrides: Hm0={hm0_plot} m, Tp={tp_plot} s)")
                        
                        # Plot measured spectrum
                        plt.plot(
                            freq,
                            month_avg.values,
                            label=f"{month_names[month-1]} (measured)",
                            linewidth=2.5,
                            color=colors[idx],
                            alpha=0.9
                        )
                        
                        # Generate and plot fitted JONSWAP spectrum
                        js_spectrum = jonswap_spectrum_custom(freq, tp_plot, hm0_plot, gamma=gamma_fit)
                        
                        plt.plot(
                            freq,
                            js_spectrum,
                            label=f"{month_names[month-1]} (γ={gamma_fit:.3f})",
                            linewidth=1.5,
                            color=colors[idx],
                            alpha=0.5,
                            linestyle='--'
                        )
                
                plt.xlabel("Frequency [Hz]", fontsize=12)
                plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
                plt.title(f"NDBC Buoy {buoy_number} - Monthly Measured vs Fitted JONSWAP Spectra ({year_for_js})", fontsize=14)
                plt.legend(fontsize=9, loc='upper right', ncol=2)
                plt.grid(True, alpha=0.3)
                plt.tight_layout()
                plt.show()
                plt.close()
            else:
                print(f"Year {year_for_js} not found in available data")
    
    elif args.plot_spectrum_js is True:
        # Plot year-averaged spectra vs fitted JONSWAP
        if selected_years:
            plt.figure(figsize=(14, 8))
            colors = plt.cm.viridis(np.linspace(0, 1, len(selected_years)))
            
            all_year_avg_spectra_js = []
            
            print(f"\nYear-Averaged Spectrum Comparison:")
            print("-" * 80)
            
            for idx, year in enumerate(selected_years):
                year_key = str(year)
                if year_key in ndbc_data:
                    year_data = ndbc_data[year_key]
                    
                    # Collect all spectra for this year
                    year_spectra = []
                    for timestamp, spectrum in year_data.items():
                        spectrum_clean = spectrum.dropna()
                        if len(spectrum_clean) > 0:
                            year_spectra.append(spectrum_clean)
                    
                    if year_spectra:
                        # Year average
                        year_avg = pd.concat(year_spectra, axis=1).mean(axis=1)
                        year_avg_series = pd.Series(year_avg.values, index=year_avg.index)
                        all_year_avg_spectra_js.append(year_avg_series)
                        
                        # Calculate metrics (ensure they're Python floats, not numpy types)
                        hm0 = float(wave.resource.significant_wave_height(year_avg_series))
                        te = float(wave.resource.energy_period(year_avg_series))
                        tp = float(wave.resource.peak_period(year_avg_series))
                        
                        # Fit JONSWAP gamma to MEASURED spectrum (NOT using overrides)
                        freq = year_avg.index.astype(float).values
                        gamma_fit, error, tp_fit, hit_bound = fit_jonswap_gamma(year_avg_series, freq, tp, hm0)
                        
                        print(f"Year {year}: Hm0={hm0:6.2f} m  |  Te={te:6.2f} s  |  Tp={tp:6.2f} s | gamma={gamma_fit:.3f} | Tp_fit={tp_fit:6.2f} s | Error={error:.6e}")
                        
                        # Plot measured spectrum
                        plt.plot(
                            freq,
                            year_avg.values,
                            label=f"Year {year} (measured)",
                            linewidth=2.5,
                            color=colors[idx],
                            alpha=0.9
                        )
                        
                        # Generate and plot fitted JONSWAP spectrum using MEASURED Hm0/Tp
                        js_spectrum = jonswap_spectrum_custom(freq, tp, hm0, gamma=gamma_fit)
                        plt.plot(
                            freq,
                            js_spectrum,
                            label=f"Year {year} (γ={gamma_fit:.3f}, fitted)",
                            linewidth=2.0,
                            color=colors[idx],
                            alpha=0.6,
                            linestyle='--'
                        )
                        
                        # Plot manual gammas for each year if provided (support multiple)
                        # These use override values if provided, otherwise measured values
                        if args.gamma:
                            hm0_manual = float(args.height) if args.height is not None else hm0
                            tp_manual = float(args.period) if args.period is not None else tp
                            print(f"  Year {year} manual gamma: Using Hm0={hm0_manual:.3f} m, Tp={tp_manual:.3f} s (from {'override' if args.height is not None else 'measured data'})")
                            gamma_linestyles = ['-', '-.']
                            for g_idx, gamma_val in enumerate(args.gamma):
                                gamma_val = float(gamma_val)
                                
                                # JONSWAP constrained to gamma >= GAMMA_MIN for physical validity
                                if gamma_val < GAMMA_MIN:
                                    print(f"  WARNING: gamma={gamma_val:.3f} < {GAMMA_MIN} - outside valid range (skipping)")
                                    continue
                                
                                js_manual = jonswap_spectrum_custom(freq, tp_manual, hm0_manual, gamma=gamma_val)
                                # Calculate error vs fitted spectrum
                                error_vs_fitted = np.sum((js_spectrum - js_manual) ** 2)
                                linestyle = gamma_linestyles[g_idx % len(gamma_linestyles)]
                                plt.plot(
                                    freq,
                                    js_manual,
                                    label=f"Year {year} (manual γ={gamma_val:.3f}, err={error_vs_fitted:.2e})",
                                    linewidth=1.2,
                                    color=colors[idx],
                                    alpha=0.7,
                                    linestyle=linestyle
                                )
                                print(f"  Manual γ={gamma_val:.3f}: Error vs fitted = {error_vs_fitted:.6e}")
            
            if len(all_year_avg_spectra_js) > 1:
                overall_avg_spectrum_js = pd.concat(all_year_avg_spectra_js, axis=1).mean(axis=1)
                overall_avg_series_js = pd.Series(overall_avg_spectrum_js.values, index=overall_avg_spectrum_js.index)
                
                hm0_overall = float(wave.resource.significant_wave_height(overall_avg_series_js))
                te_overall = float(wave.resource.energy_period(overall_avg_series_js))
                tp_overall = float(wave.resource.peak_period(overall_avg_series_js))
                
                # Fit to MEASURED overall spectrum (NOT using overrides)
                freq = overall_avg_spectrum_js.index.astype(float).values
                gamma_fit, error, tp_fit, hit_bound = fit_jonswap_gamma(overall_avg_series_js, freq, tp_overall, hm0_overall)
                
                print(f"Overall: Hm0={hm0_overall:6.2f} m  |  Te={te_overall:6.2f} s  |  Tp={tp_overall:6.2f} s | gamma={gamma_fit:.3f} | Tp_fit={tp_fit:6.2f} s | Error={error:.6e}")
                
                plt.plot(
                    freq,
                    overall_avg_spectrum_js.values,
                    label="Overall Average (measured)",
                    linewidth=3,
                    linestyle="-",
                    color="black",
                    alpha=0.9
                )
                
                # JONSWAP for overall average using MEASURED Hm0/Tp
                js_overall = jonswap_spectrum_custom(freq, tp_overall, hm0_overall, gamma=gamma_fit)
                
                # Check peak frequency of fitted spectrum
                fp_overall = 1.0 / tp_overall
                peak_idx_fit = np.argmax(js_overall)
                peak_freq_fit = freq[peak_idx_fit]
                peak_height_fit = js_overall[peak_idx_fit]
                
                plt.plot(
                    freq,
                    js_overall,
                    label=f"Overall Average (γ={gamma_fit:.3f}, fitted)",
                    linewidth=2,
                    linestyle="--",
                    color="black",
                    alpha=0.5
                )
                print(f"  Fitted γ={gamma_fit:.3f}: Peak at f={peak_freq_fit:.6f} Hz (expect {fp_overall:.6f}), height={peak_height_fit:.4f} m²/Hz")
                
                # Plot manual gammas if provided (support multiple)
                # These use override values if provided, otherwise measured values
                if args.gamma:
                    hm0_manual = float(args.height) if args.height is not None else hm0_overall
                    tp_manual = float(args.period) if args.period is not None else tp_overall
                    gamma_colors = ['green', 'blue', 'red', 'purple', 'orange', 'brown', 'pink', 'gray']
                    gamma_linestyles = [':', '-.', '--', '-', ':', '-.', '--', '-']
                    fp_expected = 1.0 / tp_manual
                    print(f"\nManual gamma vs fitted comparison (overall):")
                    print(f"  Using Hm0={hm0_manual:.3f} m, Tp={tp_manual:.3f} s (from {'override' if args.height is not None else 'measured data'})")
                    print(f"  Expected peak frequency: fp = 1/{tp_manual:.3f} = {fp_expected:.6f} Hz")
                    print(f"DEBUG: args.gamma = {args.gamma}, type = {type(args.gamma)}")
                    for idx, gamma_val in enumerate(args.gamma):
                        print(f"DEBUG: Processing gamma_val = {gamma_val}, type = {type(gamma_val)}")
                        if isinstance(gamma_val, list):
                            continue
                        gamma_val = float(gamma_val)
                        
                        # JONSWAP constrained to gamma >= GAMMA_MIN for physical validity
                        if gamma_val < GAMMA_MIN:
                            print(f"  ERROR: gamma={gamma_val:.3f} < {GAMMA_MIN} - outside valid range (skipping)")
                            continue
                        
                        try:
                            js_manual = jonswap_spectrum_custom(freq, tp_manual, hm0_manual, gamma=gamma_val)
                            # Check for NaN or inf values
                            if np.any(np.isnan(js_manual)) or np.any(np.isinf(js_manual)):
                                print(f"  WARNING: γ={gamma_val:.3f} produced NaN/inf values")
                                continue
                            error_vs_fitted = np.sum((js_overall - js_manual) ** 2)
                            
                            # Find the peak frequency of this spectrum
                            peak_idx = np.argmax(js_manual)
                            peak_freq = freq[peak_idx]
                            peak_height = js_manual[peak_idx]
                            
                            color = gamma_colors[idx % len(gamma_colors)]
                            linestyle = gamma_linestyles[idx % len(gamma_linestyles)]
                            plt.plot(
                                freq,
                                js_manual,
                                label=f"Overall Average (manual γ={gamma_val:.3f}, err={error_vs_fitted:.2e})",
                                linewidth=2,
                                linestyle=linestyle,
                                color=color,
                                alpha=0.5
                            )
                            print(f"  γ={gamma_val:.3f}: Peak at f={peak_freq:.6f} Hz (expect {fp_expected:.6f}), height={peak_height:.4f} m²/Hz, error={error_vs_fitted:.6e}")
                            print(f"  Manual γ={gamma_val:.3f}: Error vs fitted = {error_vs_fitted:.6e}")
                        except Exception as e:
                            print(f"  ERROR with γ={gamma_val:.3f}: {e}")
                            import traceback
                            traceback.print_exc()
            
            plt.xlabel("Frequency [Hz]", fontsize=12)
            plt.ylabel("Spectral Density [m²/Hz]", fontsize=12)
            plt.title(f"NDBC Buoy {buoy_number} - Year-Averaged Measured vs Fitted JONSWAP Spectra", fontsize=14)
            plt.legend(fontsize=10, loc='upper right', ncol=2)
            plt.grid(True, alpha=0.3)
            plt.tight_layout()
            plt.show()
            plt.close()
        else:
            print("No valid years found to plot")


#~~~~~~~~~~~~~~ Plot monthly wave power statistics if requested ~~~~~~~~~~~~~~

def select_records(flag_values):
    """Raw swden records matching the current --year / --date / --time selection.

    A month list or year passed to the flag itself overrides the global year choice.
    Returns (DataFrame, label) on success or (None, reason).
    """
    months = parse_month_list(flag_values, min_count=1)
    year_arg = None
    if months is None and len(flag_values) == 1:
        try:
            candidate = int(flag_values[0])
            if candidate > 12:
                year_arg = candidate
        except ValueError:
            pass

    years = [year_arg] if year_arg else list(selected_years)
    frames = [ndbc_data[str(y)] for y in years if str(y) in ndbc_data]
    if not frames:
        return None, f"no data found for {years}"

    raw = pd.concat(frames, axis=1, sort=False)

    def keep(ts):
        if months and ts.month not in months:
            return False
        if not extract_targets:
            return True
        for t in extract_targets:
            if t['kind'] == 'range':
                if t['timestamp'] <= ts <= t['end']:
                    return True
            elif t['kind'] == 'daily':
                if ts.date() == t['timestamp'].date():
                    return True
            elif abs((ts - t['timestamp']).total_seconds()) <= 1800:
                return True
        return False

    cols = sorted([c for c in raw.columns if keep(c)])
    if len(cols) < 2:
        return None, f"only {len(cols)} record(s) in the selection"

    names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
             'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
    if extract_targets:
        labels = [t['label'] for t in extract_targets]
        label = labels[0] if len(labels) == 1 else f"{labels[0]} ... {labels[-1]}"
    elif months:
        label = f"{', '.join(names[m - 1] for m in months)} {years[0]}"
    else:
        label = ', '.join(map(str, years))
    return raw[cols], label


if args.plot_wavestats is not None:
    if spectrum_from_file:
        print("\n--plot_wavestats needs buoy records and is not available with --spectrum_file")
    else:
        window, sel_label = select_records(plot_wavestats_values)
        if window is None:
            print(f"\nCannot plot wave statistics: {sel_label}")
        else:
            idx = pd.DatetimeIndex(window.columns)

            hm0_w = np.asarray(wave.resource.significant_wave_height(window)).ravel()
            tz_w = np.asarray(wave.resource.average_zero_crossing_period(window)).ravel()
            metrics = {
                'Hm0': hm0_w,
                'Te': np.asarray(wave.resource.energy_period(window)).ravel(),
                'Tp': np.asarray(wave.resource.peak_period(window)).ravel(),
            }
            if water_depth:
                metrics['J'] = np.asarray(
                    wave.resource.energy_flux(window, water_depth)).ravel() / 1000.0
            # Wave steepness Sm = Hm0 / ((g/2pi) * Tz^2)
            metrics['Sm'] = hm0_w / ((9.81 / (2 * np.pi)) * (tz_w ** 2))

            data_clean = pd.DataFrame(metrics, index=idx).dropna()

            if len(data_clean) < 2:
                print("\nNot enough valid data for wave statistics")
            else:
                clean_idx = data_clean.index
                span = clean_idx[-1] - clean_idx[0]

                # Resolution follows the span of the selection
                month_labels = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
                                'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
                if span <= pd.Timedelta(days=2):
                    group_key, xlabel, res = clean_idx.hour, "Hour of Day (UTC)", "Hourly"
                elif span <= pd.Timedelta(days=62):
                    group_key, xlabel, res = clean_idx.normalize(), "Date", "Daily"
                else:
                    group_key, xlabel, res = clean_idx.month, "Month", "Monthly"

                print(f"\nGenerating {res.lower()} wave statistics for {sel_label} "
                      f"({len(data_clean)} records)...")

                data_group = data_clean.groupby(group_key)
                medians = data_group.median()
                quartiles = data_group.describe()

                QoIs = list(data_clean.columns)
                fig, axs = plt.subplots(len(QoIs), 1, figsize=(10, 10), sharex=True)
                if len(QoIs) == 1:
                    axs = [axs]

                units = {'Hm0': 'm', 'Te': 's', 'Tp': 's', 'J': 'kW/m', 'Sm': '-'}
                for i, QoI in enumerate(QoIs):
                    axs[i].plot(medians.index, medians[QoI], marker="o", linewidth=2,
                                markersize=6, color="steelblue")
                    axs[i].fill_between(
                        medians.index,
                        quartiles[QoI, "25%"],
                        quartiles[QoI, "75%"],
                        alpha=0.3,
                        color="steelblue",
                        label="25%-75% IQR"
                    )
                    axs[i].grid(True, alpha=0.3)
                    axs[i].set_ylabel(f"{QoI} [{units.get(QoI, '-')}]", fontsize=11)

                    mx, mn = medians[QoI].max(), medians[QoI].min()
                    mx_at, mn_at = medians[QoI].idxmax(), medians[QoI].idxmin()
                    if res == "Monthly":
                        mx_at, mn_at = month_labels[mx_at - 1], month_labels[mn_at - 1]
                    elif res == "Daily":
                        mx_at, mn_at = f"{mx_at:%Y-%m-%d}", f"{mn_at:%Y-%m-%d}"
                    else:
                        mx_at, mn_at = f"{mx_at:02d}:00", f"{mn_at:02d}:00"
                    print(f"\n{QoI}:")
                    print(f"  Max: {mx:.4f} at {mx_at}")
                    print(f"  Min: {mn:.4f} at {mn_at}")

                    if i == 0:
                        axs[i].set_title(
                            f"{source_title} - {res} Wave Statistics ({sel_label})",
                            fontsize=13)
                    if i == len(QoIs) - 1:
                        axs[i].set_xlabel(xlabel, fontsize=11)

                if res == "Monthly":
                    axs[-1].set_xticks(range(1, 13))
                    axs[-1].set_xticklabels(month_labels)
                elif res == "Hourly":
                    axs[-1].set_xticks(range(0, 24, 2))
                else:
                    fig.autofmt_xdate()

                plt.tight_layout()
                plt.show()
                plt.close()

                print(f"\n{res} wave statistics plot displayed")


#~~~~~~~~~~~~~~ Plot wind data if requested ~~~~~~~~~~~~~~

if args.plot_wind is not None:
    if len(wind_data) == 0:
        print("\nNo wind data available to plot")
    else:
        # A month or year given to the flag overrides the global year selection
        wind_months = parse_month_list(plot_wind_values, min_count=1)
        wind_year = None
        if wind_months is None and len(plot_wind_values) == 1:
            try:
                candidate = int(plot_wind_values[0])
                if candidate > 12:
                    wind_year = candidate
            except ValueError:
                pass

        wind_years = [wind_year] if wind_year else list(selected_years)
        frames = [wind_data[str(y)].T for y in wind_years if str(y) in wind_data]

        if not frames:
            print(f"\nNo wind data found for {wind_years}")
        else:
            wind_ts = pd.concat(frames, axis=0).sort_index()

            # Mask NDBC missing-value sentinels
            for col, limit in (('WSPD', 90), ('GST', 90), ('WDIR', 400)):
                if col in wind_ts.columns:
                    wind_ts[col] = wind_ts[col].where(wind_ts[col] < limit)

            if wind_months:
                wind_ts = wind_ts[wind_ts.index.month.isin(wind_months)]

            _names = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
                      'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
            wind_label = (f"{', '.join(_names[m - 1] for m in wind_months)} {wind_years[0]}"
                          if wind_months else ', '.join(map(str, wind_years)))

            series_specs = [
                ('WSPD', 'Wind Speed [m/s]', 'steelblue'),
                ('GST', 'Wind Gusts [m/s]', 'darkorange'),
                ('WDIR', 'Wind Direction [deg]', 'seagreen'),
            ]
            series_specs = [s for s in series_specs if s[0] in wind_ts.columns]

            if extract_single or extract_daily:
                # Daily view: raw 10-minute interval time series for the requested day
                target_day = (single_measurement_date if extract_single else daily_start).date()
                day_ts = wind_ts[wind_ts.index.date == target_day]

                if len(day_ts) == 0:
                    print(f"\nNo wind data found for {target_day}")
                else:
                    print(f"\nGenerating daily wind plot for {target_day} ({len(day_ts)} 10-minute records)...")
                    fig, axs = plt.subplots(len(series_specs), 1, figsize=(12, 8), sharex=True)
                    if len(series_specs) == 1:
                        axs = [axs]

                    for i, (col, label, color) in enumerate(series_specs):
                        axs[i].plot(day_ts.index, day_ts[col], marker='o', markersize=3,
                                    linewidth=1.2, color=color)
                        axs[i].set_ylabel(label, fontsize=11)
                        axs[i].grid(True, which='major', alpha=0.3)
                        axs[i].grid(True, which='minor', axis='x', alpha=0.15, linestyle=':')
                        if col == 'WDIR':
                            axs[i].set_ylim(0, 360)
                            axs[i].set_yticks([0, 90, 180, 270, 360])
                        if extract_single:
                            axs[i].axvline(single_measurement_date, color='red',
                                           linestyle='--', linewidth=1, alpha=0.7)

                    axs[0].set_title(
                        f"NDBC Buoy {buoy_number} - Wind at 10-Minute Intervals ({target_day})",
                        fontsize=13)
                    axs[-1].set_xlabel("Hour of Day (UTC)", fontsize=11)

                    # Tick every 2 hours (minor ticks hourly) across the full 24-hour day
                    day_start = pd.Timestamp(target_day)
                    axs[-1].set_xlim(day_start, day_start + pd.Timedelta(days=1))
                    axs[-1].xaxis.set_major_locator(mdates.HourLocator(interval=2))
                    axs[-1].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
                    axs[-1].xaxis.set_minor_locator(mdates.HourLocator(interval=1))
                    plt.setp(axs[-1].get_xticklabels(), rotation=0, ha='center')

                    plt.tight_layout()
                    plt.show()
                    plt.close()
                    print("Daily wind plot displayed")
            else:
                # Median + IQR at a resolution that follows the selection
                idx = wind_ts.index
                span = idx[-1] - idx[0]
                month_labels = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
                                'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
                if span <= pd.Timedelta(days=62):
                    group_key, xlabel, res = idx.normalize(), "Date", "Daily"
                else:
                    group_key, xlabel, res = idx.month, "Month", "Monthly"

                print(f"\nGenerating {res.lower()} wind statistics for {wind_label} "
                      f"({len(wind_ts)} records)...")

                grouped = wind_ts.groupby(group_key)
                medians = grouped.median()
                quartiles = grouped.describe()

                fig, axs = plt.subplots(len(series_specs), 1, figsize=(12, 10), sharex=True)
                if len(series_specs) == 1:
                    axs = [axs]

                for i, (col, label, color) in enumerate(series_specs):
                    axs[i].plot(medians.index, medians[col], marker="o", linewidth=2,
                                markersize=5, color=color, label="Median")
                    axs[i].fill_between(
                        medians.index,
                        quartiles[col, "25%"],
                        quartiles[col, "75%"],
                        alpha=0.3,
                        color=color,
                        label="25%-75% IQR"
                    )
                    axs[i].set_ylabel(label, fontsize=11)
                    axs[i].grid(True, alpha=0.3)
                    if col == 'WDIR':
                        axs[i].set_ylim(0, 360)
                        axs[i].set_yticks([0, 90, 180, 270, 360])
                    if i == 0:
                        axs[i].legend(fontsize=9, loc='upper right')

                    mx, mn = medians[col].max(), medians[col].min()
                    mx_at, mn_at = medians[col].idxmax(), medians[col].idxmin()
                    if res == "Monthly":
                        mx_at, mn_at = month_labels[mx_at - 1], month_labels[mn_at - 1]
                    else:
                        mx_at, mn_at = f"{mx_at:%Y-%m-%d}", f"{mn_at:%Y-%m-%d}"
                    print(f"\n{col}:")
                    print(f"  Max median: {mx:.2f} at {mx_at}")
                    print(f"  Min median: {mn:.2f} at {mn_at}")
                    print(f"  Overall mean/std: {wind_ts[col].mean():.2f} / {wind_ts[col].std():.2f}")

                axs[0].set_title(
                    f"{source_title} - {res} Wind Distribution ({wind_label})", fontsize=13)
                axs[-1].set_xlabel(xlabel, fontsize=11)
                if res == "Monthly":
                    axs[-1].set_xticks(range(1, 13))
                    axs[-1].set_xticklabels(month_labels)
                    axs[-1].set_xlim(0.4, 12.6)
                else:
                    fig.autofmt_xdate()
                plt.tight_layout()
                plt.show()
                plt.close()
                print(f"\n{res} wind statistics plot displayed")


#~~~~~~~~~~~~~~ Plot directional wave spectrum if requested ~~~~~~~~~~~~~~

if args.plot_wavedirection is not None:
    if selected_years:
        # Use the first selected year for directional spectrum
        dir_year = selected_years[0]
        plot_type = args.plot_wavedirection  # elevation, energy, or spread
        print(f"\nFetching directional wave data for year {dir_year}...")
        
        try:
            # Check if directional data is available
            dir_available = ndbc.available_data("swdir", buoy_number)
            if len(dir_available) == 0:
                print(f"No directional wave data available for buoy {buoy_number}")
            else:
                # Request directional data for the selected year
                dir_data_all = ndbc.request_directional_data(buoy_number, dir_year)
                
                if len(dir_data_all) == 0:
                    print(f"No directional data found for year {dir_year}")
                else:
                    # Drop NaN values before selecting or averaging
                    dir_data_clean = dir_data_all.dropna(dim='date', how='all')

                    # --date / --time narrow the average to those records; the swdir
                    # archive is yearly, so the whole year is fetched either way.
                    period_label = f"Year-Averaged {dir_year}"
                    if extract_targets and len(dir_data_clean.date) > 0:
                        dir_dates = pd.DatetimeIndex(dir_data_clean.date.values)
                        labels = [t['label'] for t in extract_targets]
                        if extract_single:
                            mask = np.zeros(len(dir_dates), dtype=bool)
                            for target in extract_targets:
                                if target['kind'] == 'range':
                                    mask |= ((dir_dates >= target['timestamp']) &
                                             (dir_dates <= target['end']))
                                    continue
                                ts = target['timestamp']
                                nearest = dir_dates[np.argmin(np.abs(dir_dates - ts))]
                                diff = abs((nearest - ts).total_seconds())
                                if diff < 3600:
                                    mask |= (dir_dates == nearest)
                                    print(f"  Directional record at {nearest:%Y-%m-%d %H:%M} "
                                          f"({diff/60:.0f} min from requested {ts:%Y-%m-%d %H:%M})")
                                else:
                                    print(f"  WARNING: no directional record within 1 h of "
                                          f"{ts:%Y-%m-%d %H:%M}")
                        else:
                            target_days = {t['timestamp'].normalize() for t in extract_targets}
                            mask = dir_dates.normalize().isin(target_days)

                        if mask.any():
                            dir_data_clean = dir_data_clean.isel(date=np.flatnonzero(mask))
                            period_label = labels[0] if len(labels) == 1 else \
                                f"{len(labels)} selections ({labels[0]} ... {labels[-1]})"
                            print(f"Averaging {int(mask.sum())} directional records for {period_label}...")
                        else:
                            print(f"WARNING: no directional data for the requested date(s); "
                                  f"falling back to the {dir_year} average.")
                    else:
                        print(f"Averaging directional spectrum across year {dir_year}...")

                    if len(dir_data_clean.date) == 0:
                        print(f"No valid directional data for year {dir_year}")
                    else:
                        # Average the spectral components across all times
                        dir_data_mean = dir_data_clean.mean(dim='date')
                        
                        # Create directional spectrum from averaged data (2 degree resolution)
                        directions = np.arange(0, 360, 2.0)
                        raw_spectrum = ndbc.create_directional_spectrum(dir_data_mean, directions)
                        
                        if plot_type == "elevation":
                            # Standard elevation spectrum (2D wave spectrum S(f,θ)) [m²/Hz/deg]
                            directional_spectrum = raw_spectrum
                            plot_title = f"{period_label} Elevation Spectrum"
                            colorbar_label = "m²/Hz/deg"
                        elif plot_type == "energy":
                            # Energy spectrum - multiply by frequency to get energy distribution [J/Hz/deg]
                            directional_spectrum = raw_spectrum.copy()
                            freq_values = raw_spectrum.frequency.values
                            # Convert to energy by multiplying by frequency
                            directional_spectrum.values = raw_spectrum.values * freq_values[:, np.newaxis]
                            plot_title = f"{period_label} Energy Spectrum"
                            colorbar_label = "J/Hz/deg"
                        else:  # spread
                            # Spreading function - normalize each frequency to sum to 1 [1/Hz/deg]
                            directional_spectrum = raw_spectrum.copy()
                            spectrum_values = raw_spectrum.values
                            for freq_idx in range(spectrum_values.shape[0]):
                                freq_sum = spectrum_values[freq_idx, :].sum()
                                if freq_sum > 0:
                                    directional_spectrum.values[freq_idx, :] = spectrum_values[freq_idx, :] / freq_sum
                            plot_title = f"{period_label} Spreading Function"
                            colorbar_label = "1/Hz/deg"
                        
                        # Plot directional spectrum
                        print(f"Plotting {plot_type} spectrum for {period_label}...")
                        wave.graphics.plot_directional_spectrum(directional_spectrum)
                        
                        # Set nautical convention: 0° north (up), 90° east (right)
                        ax = plt.gca()
                        ax.set_theta_zero_location('N')
                        ax.set_theta_direction(-1)  # Clockwise
                        
                        # Add colorbar label with units
                        cbar = ax.collections[0].colorbar if hasattr(ax.collections[0], 'colorbar') and ax.collections[0].colorbar else None
                        if cbar is None:
                            # Try to find colorbar in figure
                            for cbar_obj in ax.figure.axes:
                                if hasattr(cbar_obj, 'yaxis') and cbar_obj.yaxis.get_label_text() == '':
                                    cbar_obj.set_ylabel(colorbar_label)
                        else:
                            cbar.set_label(colorbar_label)
                        
                        # Apply nofill if requested
                        if args.nofill:
                            # Remove filled contours, keep only contour lines with color mapping
                            ax.clear()
                            # Re-plot with contour lines only (no fill), but with colors
                            theta = np.deg2rad(directions)
                            freq = np.arange(directional_spectrum.values.shape[0]) * 0.01  # Approximate frequency bins
                            theta_mesh, freq_mesh = np.meshgrid(theta, freq)
                            # Create contour lines with colormap - no fill
                            contours = ax.contour(theta_mesh, freq_mesh, directional_spectrum.values, levels=12, cmap='viridis', linewidths=1.5)
                            ax.clabel(contours, inline=False, fontsize=0)  # Hide labels but keep contours
                            ax.set_theta_zero_location('N')
                            ax.set_theta_direction(-1)
                            ax.set_title(plot_title)
                        else:
                            plt.title(plot_title)
                        
                        plt.show()
                        plt.close()
                        print(f"Directional spectrum plot displayed")
        except Exception as e:
            print(f"Error plotting directional spectrum: {e}")
    else:
        print("No years found for directional spectrum")


#~~~~~~~~~~~~~~ Plot environmental contour heatmap if requested ~~~~~~~~~~~~~~

if args.plot_heatmap is not None:
    if spectrum_from_file:
        print("\n--plot_heatmap needs buoy records and is not available with --spectrum_file")
    else:
        window, sel_label = select_records(plot_heatmap_values)
        if window is None:
            print(f"\nCannot plot heatmap: {sel_label}")
        else:
            df_metrics = pd.DataFrame({
                'Hm0': np.asarray(wave.resource.significant_wave_height(window)).ravel(),
                'Te': np.asarray(wave.resource.energy_period(window)).ravel(),
            }).dropna()

            if len(df_metrics) < 2:
                print("\nNot enough valid data for the heatmap")
            else:
                print(f"\nGenerating sea state heatmap for {sel_label} "
                      f"({len(df_metrics)} records)...")

                # Finer bins when the selection is small, so a day is not one blob
                if len(df_metrics) < 200:
                    Hm0_bin_size, Te_bin_size = 0.25, 0.5
                else:
                    Hm0_bin_size, Te_bin_size = 0.5, 1.0

                # Bin over the observed range so empty rows/columns are not drawn
                def _edges(series, step):
                    lo = max(0.0, np.floor(float(series.min()) / step) * step)
                    hi = np.ceil(float(series.max()) / step) * step
                    return np.arange(lo, hi + step, step)

                Hm0_bins = _edges(df_metrics['Hm0'], Hm0_bin_size)
                Te_bins = _edges(df_metrics['Te'], Te_bin_size)
                hist, Hm0_edges, Te_edges = np.histogram2d(
                    df_metrics['Hm0'].values,
                    df_metrics['Te'].values,
                    bins=[Hm0_bins, Te_bins]
                )

                fig, ax = plt.subplots(figsize=(12, 8))
                im = ax.pcolormesh(
                    Te_edges,
                    Hm0_edges,
                    hist,
                    shading='auto',
                    cmap='YlOrRd'
                )
                ax.set_xlabel("Energy Period, Te [s]", fontsize=12)
                ax.set_ylabel("Significant Wave Height, Hm0 [m]", fontsize=12)
                ax.set_title(
                    f"{source_title} - Sea State Frequency Distribution ({sel_label})",
                    fontsize=14
                )
                ax.set_xlim(Te_edges[0], Te_edges[-1])
                ax.set_ylim(Hm0_edges[0], Hm0_edges[-1])
                cbar = plt.colorbar(im, ax=ax)
                cbar.set_label("Count (Occurrences)", fontsize=11)
                plt.tight_layout()
                plt.show()
                plt.close()
                print("Sea state heatmap displayed")
