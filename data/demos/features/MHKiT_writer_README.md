# MHKiT Wave YAML Writer

A Python tool for analyzing real ocean wave data and writing it into SEA-Stack YAML input files.

It pulls spectral wave density (`swden`) records from NDBC buoys via MHKiT, computes sea-state statistics, and writes the result into device `*.hydro.yaml` file 

It supports:
- Regular waves (height + period from measured data)
- Irregular waves with a parametric spectrum (Pierson-Moskowitz or JONSWAP with a fitting gamma feature)
- Irregular waves from a measured spectrum, written as a surface elevation (eta) file
- Custom spectra read from a local text file

With capabilities such as
- Direct data access to the various NDBC buoys
- Selecting data by year, month, date, or an individual 30-minute measurement
- Comparing measured spectra against PM and fitted-JONSWAP estimates
- Override capabilities of wave height, period, peak enhancement, water depth, and simulation parameters
- Plotting-only for wave characteristic analysis such as:
  - environmental contours
  - joint distributions
  - monthly wave statistics
  - directional spectra
  - wind data
 

---------

## Basic command structure:

run the python file in terminal
- located in data/demos/run_seastack/features
  - either cd data/demos/run_seastack/features or direct to data/demos/run_seastack/features/MHKiT_writer.py

```bash
python MHKiT_writer.py <path_to_hydro_yaml/case.hydro.yaml> --buoy <number> 
```

The **positional path** is the hydro YAML to update. It must already exist; this script edits an existing `hydrodynamics.waves` block, it does not create one.

---

## How it works

```
   NDBC buoy  ──┐
                ├──►  select years / months / date / measurement ──► data analysis plotting
   spectrum file┘                    │
                                     ▼
                        Hm0 , Te , Tp , water depth
                                     │
              ┌──────────────────────┼──────────────────────┐
              ▼                      ▼                      ▼
        regular waves        parametric spectrum       custom spectrum
        height + period         pm / jonswap          eta file + nfrequencies
              │                      │                      │
              └──────────────────────┼──────────────────────┘
                                     ▼
                          <case>.hydro.yaml  (waves block)
                          <case>.simulation.yaml  (end_time)
```

When plot flags are called (without yaml definitions like `--type`, `--spectrum`, or `--depth`), then the yaml files will not be updated and the code is *plot-only* for data analysis.

---

### `--buoy`
NDBC buoy number to download spectral wave data from, can be found from website: https://www.ndbc.noaa.gov/

Required unless `--spectrum_file` or an existing `--elevation_file` is supplied.

**Note:** not every buoy has a historical `swden` archive. Partner-operated buoys (CDIP/Scripps, e.g. 46258) show live data on the NDBC website but have no spectral archive to download. Some buoys does not have continuous year-long data.

---

### `--year`
Select which year(s) of buoy data to use (default = latest available) for longer time-scale data analysis

- `--year 2024` — a single year
- `--year 2022 2023 2024` — several years averaged
- `--year from:2020` — that year and everything after
- omit for the latest available year

---

### `--date` / `--time`
Narrow the selection to specific days or individual measurements.

- `--date 01-01-2024` → daily average 
- `--date 01-01 01-02 01-03` — list of dates with `--year 2024`
- `--date` and `--time` → single raw measurement at date/time pair
- `--time 05:10 05:40` — list of times 
- `--time 14:00-17:00` — time range
- multiple list values are overlaid on one plot

NDBC samples at `:10` and `:40` past the hour; the closest measurement within an hour is used and reported.

---

## Data analysis features


### `--plot_spectrum`
Plot the measured wave spectrum

- `--plot_spectrum` →  with `--date` and/or `--time` plots corresponding spectrum
- `--plot_spectrum year` → every month of that year overlaid
- `--plot_spectrum month` → individual or list months with `--year`

**Year breakdown example:**
```bash
python MHKiT_writer.py -buoy 46050 --plot_spectrum 2020
```

<img width="1768" height="864" alt="image" src="https://github.com/user-attachments/assets/873ec849-a5eb-4841-af8a-2c6991e6989a" />


**Multiple years example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_spectrum --year from:2010
```

<img width="1763" height="855" alt="image" src="https://github.com/user-attachments/assets/2079c50c-56be-487b-8ce9-45e899f8a214" />


**List of months example:**
```bash
python MHKiT_writer.py -buoy 46050 --plot_spectrum jan feb march apr may --year 2020
```

<img width="1774" height="857" alt="image" src="https://github.com/user-attachments/assets/ccba59e2-8696-4f69-964b-fe955cd09302" />


**Exact date/times example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_spectrum --date 01-01-2020 --time 05:00 06:00 07:00
```

<img width="1763" height="870" alt="image" src="https://github.com/user-attachments/assets/1393d1b2-c360-45cc-a77f-840cc8013dac" />


---

### `--plot_spectrum_pm`
Plot the measured spectrum against a Pierson-Moskowitz estimate at the same Hm0 and Tp, and print the sum of squared residuals fit error.

**Aliases:** `--plot_spectrum_PM`

Accepts the same bare / year / months / `--date` forms as `--plot_spectrum`.

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_spectrum_pm 2020
```

<img width="2066" height="1157" alt="image" src="https://github.com/user-attachments/assets/0ce2577f-1048-4a9c-bc8a-ec4e5c4bf82a" />

---

### `--plot_spectrum_js`
Plot the measured spectrum against a **fitted** JONSWAP spectrum. Gamma and Tp are least-squares fitted to the measurement (Hs held fixed so total energy matches).

**Aliases:** `--plot_spectrum_JS`, `--plot_spectrum_jonswap`, `--plot_spectrum_JONSWAP`

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_spectrum_js --date 01-01 05-01 --time 00:00-12:00 12:00-24:00 --year 2024
```

<img width="883" height="430" alt="image" src="https://github.com/user-attachments/assets/cbca0aac-7e41-4315-ad02-3bb402dbb5df" />

Technically JONSWAP spectrum is only valid for gamma > 0.1, so if the printed gamma pins below, then the sea is broad or double-peaked (swell + wind sea) and a single-peak JONSWAP cannot represent it. 

---

### `--gamma`
Overlay manual gamma values alongside the fitted JONSWAP, with the error against both the fit and the measurement.

**Used with:** `--plot_spectrum_js` or to overwrite yaml 

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_spectrum_js --date 01-03-2022 --gamma 1.0 1.1 1.2
```

<img width="887" height="437" alt="image" src="https://github.com/user-attachments/assets/e3cbb3e6-a6a8-49e9-b6cc-39e535bd23fd" />



---

### `--plot_wavestats`
Median + interquartile range of Hm0, Te, Tp, energy flux J and steepness Sm, with the max/min month printed for each.

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_wavestats --year 2020
```

<img width="731" height="732" alt="image" src="https://github.com/user-attachments/assets/cb82c366-ace2-47bd-848c-dfe047dc2075" />

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_wavestats jan --year 2020
```

<img width="733" height="737" alt="image" src="https://github.com/user-attachments/assets/8821399d-d2a2-4cbd-aafd-fd09365a0a96" />


---

### `--plot_heatmap`
Sea-state frequency distribution — a 2D histogram of Hm0 against Te over the selected time.

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_heatmap --year 2020
```

<img width="842" height="584" alt="image" src="https://github.com/user-attachments/assets/adebf612-620e-49d0-ab17-dbecd41c0eaa" />

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_heatmap jan --year 2024
```

<img width="834" height="581" alt="image" src="https://github.com/user-attachments/assets/cc38922b-effd-4d1f-bae9-efc837baf81e" />


---

### `--plot_wavedirection`
Directional wave spectrum on a polar plot, nautical convention (0° north, clockwise).

Options:
- elevation (default)
- energy
- spread

Add `--nofill` for contour lines only.

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_wavedirection --year 2020
```

<img width="428" height="347" alt="image" src="https://github.com/user-attachments/assets/4bd2f173-f513-4230-9c8b-98f30fe45144" />

**Example:**
```bash
python MHKiT_writer.py  --buoy 46050 --plot_wavedirection spread --date 01-01-2020 01-02-2020 01-02-2020 --nofill
```

<img width="440" height="344" alt="image" src="https://github.com/user-attachments/assets/3758d232-1fa2-48f8-a98f-29a29dda2710" />

**Example:**
```bash
python MHKiT_writer.py  --buoy 46050 --plot_wavedirection spread --year 2024 --nofill
```

<img width="418" height="344" alt="image" src="https://github.com/user-attachments/assets/9eb1c9c6-a197-446a-b27d-294c83e88fe6" />


---

### `--plot_wind`
Wind speed, direction and gusts from the `stdmet` record.

- with `--date` → 10-minute interval series for that day
- without → monthly box-and-whisker with mean ± 1 std

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_wind --date 01-01-2020
```

<img width="881" height="584" alt="image" src="https://github.com/user-attachments/assets/7738ab10-fd33-4877-a9e7-7139ea65f18f" />

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_wind jan --year 2024
```

<img width="879" height="733" alt="image" src="https://github.com/user-attachments/assets/7fac223c-d6fd-4cf0-9e2f-df4034bbe517" />

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_wind jan 2023
```

<img width="886" height="740" alt="image" src="https://github.com/user-attachments/assets/7bcb2a96-afa9-4a9c-ae98-a06f0271aa4b" />


---

## Writing to Yaml input files

### Path to yaml file name
This path tells the script where the yaml file of the device is. It will only edit the waves information in the hydro.yaml file, and sometimes the simulation.yaml if necessary.

```bash
python MHKiT_writer.py <path_to_hydro_yaml/case.hydro.yaml>
```

### Override flags
To force a value to write to the yaml, instead of using the measured one.

- `--height` — significant wave height [m]
- `--period` — wave period [s]
- `--depth` — water depth [m] (otherwise taken from buoy metadata)
- `--seed` — random seed for eta generation (default 42)
- `--ramp_time` — ramp duration [s], cosine ramp (default 60)
- `--gamma` — peak enhancement factor, overrides for plotting and yaml writing


**Override height, period, and depth:**
```bash
--height 3.0 --period 9.0 --depth 100
```


### `--type`
Wave type to define YAML hydro (default = `regular`)

- regular
- irregular

**Write year averaged regular waves:**
```bash
python MHKiT_writer.py ../5sa/regular_waves/5sa_regular.hydro.yaml --buoy 46050 --year 2024
```

---

### `--spectrum`
Spectrum type for irregular waves

Options:
- `pm` / `pierson_moskowitz` — parametric PM, uses Tp
- `js` / `jonswap` — parametric JONSWAP, gamma fitted to the measured spectrum associated with data
- `custom` — build a surface elevation record from the measured spectrum, or spectrum file, and write an eta file

**Used with:** `--type irregular`

**Jonswap fit and write:**
```bash
python MHKiT_writer.py <../path_to_hydro_yaml/case.hydro.yaml> --type irregular --spectrum jonswap --date 01-01-2020 --gamma 1.2
```
This pulls spectrum data for the day 01-01-2020, fits a jonswap function with height, period, and gamma, overrides the gamma with 1.2, then writes this all to the input hydro.yaml file to generate this type of irregular wave.

---

### `--spectrum_file`
Read the spectrum from a local text file instead of downloading it. **`--buoy` is not needed.**

Implies `--type irregular --spectrum custom`.

Two layouts are accepted — blank lines and `#` / `%` comments are ignored. Units are Hz and m²/Hz.

**Two rows** (frequencies, then spectral densities):
```
.0200  .0325  .0375  .0425  .0475 ...
0.00   0.00   0.00   0.00   0.00  ...
```

**Two columns** (frequency, density):
```
# f      S
0.0200   0.00
0.0325   0.00
0.0375   0.00
```

Path may be a bare name (resolved next to the YAML), a relative path, or absolute.

**Example to write an elevation time-series from custom spectrum file:**
```bash
python MHKiT_writer.py <../path_to_hydro_yaml/case.hydro.yaml> --spectrum_file custom_spectrum.txt
```


**Fit a JONSWAP curve to a custom spectrum without writing to yaml:**
```bash
python MHKiT_writer.py ../5sa/custom_waves/5sa_custom.hydro.yaml --spectrum_file test_spectrum.txt --plot_spectrum_js
```

<img width="883" height="430" alt="image" src="https://github.com/user-attachments/assets/49af3680-0f43-496e-9bc8-ab982066d55b" />


---

### `--elevation_duration` / `--elevation_dt`
Length and time step of the generated eta record (default = 600 s; dt from the companion `simulation.time_step`)

**Works with:** any `--type` and `--spectrum`, but defines time length and intervals for `--spectrum custom` generated eta files

**Important:** duration *is* frequency resolution. The eta record's frequency grid is fixed by the time vector as

```
df = 1 / (n_samples * dt)
```

so a longer record resolves the spectrum more finely but produces more wave components and a bigger file. At `dt = 0.01 s`, 600 s gives 600 components and a 1.2 MB eta file; 10800 s gives 10790 components and 22 MB.

**Example:**
```bash
python MHKiT_writer.py <path_to_hydro_yaml/case.hydro.yaml> --type irregular --spectrum jonswap --elevation_duration 1200 --elevation_dt 0.01
```

---

### `--elevation_file`
Name of the eta file to write, **or** an existing eta file to consume as-is.

If the file already exists, nothing is downloaded or generated — the record is used exactly as it is and only the YAMLs are updated. Default name is `<yaml stem>_eta.txt`.

To use existing elevation time-series file:
**Example:**
```bash
--elevation_file downloaded_eta.txt
```

To use write an elevation time-series file from buoy data:
**Example:**
```bash
--buoy 46050 --date 2024-10-12 --type irregular --spectrum custom --elevation_duration 100 --elevation_file newer_eta.txt
```
<img width="884" height="432" alt="image" src="https://github.com/user-attachments/assets/49ec12e5-f2ec-4250-a1bd-07000fcf5397" />

Interpolates the buoy spectral data and generates a time-series elevation txt file

<img width="884" height="291" alt="image" src="https://github.com/user-attachments/assets/488ee008-4e10-427e-8ae7-6d050d6315b0" />

And changes the yaml file to read from the new .txt eta file generated


---

### `--eta_method`
How SEA-Stack consumes the eta file (default = `dft`)

- `dft` — extracts discrete wave components, giving a spatial free surface, so the GUI wireframe renders
- `irf` — `EtaTableWaveField`, a point time series; the body moves but no wave surface is drawn


---

## Directional wave partitions

NDBC transmits the directional distribution as four moments per frequency (`swdir` α₁, `swdir2` α₂, `swr1` r₁, `swr2` r₂) alongside `swden`. These are reconstructed into a full 2D spectrum S(f,θ), split into distinct wave systems, and each system is fitted to the JONSWAP + cos-2s form that SEA-Stack accepts as `waves.partitions`.

The reconstruction uses the **maximum entropy method** (Lygre & Krogstad) rather than the truncated Fourier series. With only two directional harmonics, the Fourier form rings negative opposite the peak whenever r₁ + r₂ > 0.5 — inventing energy from directions that carry none. MEM is non-negative by construction and resolves twin directional peaks.

Partitions are then found by a **steepest-ascent watershed** over the (frequency, direction) plane, wrapping in θ. Every cell climbs to a local maximum; cells sharing a maximum form one wave system. This finds systems in 2D, so a swell and a wind sea arriving from different directions at overlapping frequencies are separated — which a frequency-only split cannot do.

Per partition:

| YAML key | Derived from |
|---|---|
| `Hs` | 4√(energy integrated over the partition's cells) |
| `Tp` | 1 / frequency of the partition's peak cell |
| `gamma` | least-squares JONSWAP fit to the partition's frequency spectrum |
| `direction` | local maximum of the directional marginal, parabolic sub-bin refined |
| `s` | least-squares cos-2s fit to the measured D(θ), weighted by energy |

**Direction convention:** NDBC reports the direction waves come *from*, clockwise from true North. SEA-Stack `direction` is the direction waves travel *toward*, counter-clockwise from +X. The conversion is `(270 − α) mod 360`, and both values are printed.



### `--plot_wavedirection_cos`
Compare the measured directional spectrum against the cos-2s reconstruction SEA-Stack would generate. **Never writes the YAML**, even alongside `--partitions` or the custom-spectrum flags.

- `--plot_wavedirection_cos` → uses the recommended number of partitions
- `--plot_wavedirection_cos 4` → forces 4 partitions

Produces three polar panels — measured (MEM), reconstructed (N × JONSWAP × cos-2s), and their difference — plus the per-partition directional distributions with the `n_theta` sampling marked. Prints the RMS error and where the largest residual sits.

**Example:**
```bash
python MHKiT_writer.py --buoy 46050 --plot_wavedirection_cos --date 01-03-2022
```

<!-- image: three polar panels + directional distribution row -->

---

### `--partitions`
Split the measured directional spectrum into wave systems and write them as `waves.partitions`. Implies `--type irregular`. Plots the same comparison as `--plot_wavedirection_cos`.

- `--partitions` → uses the recommended count
- `--partitions 3` → forces 3 partitions

The recommendation counts systems holding at least 5% of the total energy, capped at 6. The full energy ladder is printed so you can see whether your chosen number agrees with the data:

```
Watershed found 14 raw system(s); 2 hold >=5% of the energy -> recommending 2 partition(s)
  energy share of the strongest: 63.3%, 31.9%, 1.7%, 1.0%, 0.7%, 0.4%
```

**Example:**
```bash
python MHKiT_writer.py ../5sa/bimodal/5sa_bimodal.hydro.yaml --buoy 46050 --partitions --date 01-03-2022
```

<!-- image: console output showing the partition table -->

Produces:

```yaml
  waves:
    type: irregular
    discretization:
      n_omega: 64
      n_theta: 21
    seed: 42
    partitions:
      - spectrum: jonswap
        Hs: 5.647
        Tp: 12.121
        gamma: 1.048
        direction: 82.4
        spreading:
          type: cos2s
          s: 23.91
      - spectrum: jonswap
        Hs: 3.863
        Tp: 11.429
        gamma: 1.024
        direction: 359.4
        spreading:
          type: cos2s
          s: 11.04
```

---

### `--n_omega` / `--n_theta`
Discretization written to `waves.discretization` (defaults 64 and 21).

**Used with:** `--partitions`

`s` controls how concentrated each lobe is — higher `s` is a narrower beam — while `n_theta` controls how finely SEA-Stack samples it. Total wave components are `n_omega × n_theta × partitions`, all evaluated per surface vertex per frame, so this is the main cost knob.

**Example:**
```bash
python MHKiT_writer.py <path_to_hydro_yaml/case.hydro.yaml> --buoy 46050 --partitions 2 --n_omega 64 --n_theta 31
```

---

### Notes on directional partitioning

- **Use `--date` or `--time`, not a year.** Averaging thousands of records smooths every local maximum away — a full year of buoy 46050 collapses to a single system, while one day resolves 9–14. The script warns when more partitions are requested than the spectrum contains, since merging can combine systems but never split one.
- **More partitions is not automatically better.** Beyond the systems that hold real energy, extra partitions fragment a single physical wave train and each one still costs `n_omega × n_theta` components.
- **cos-2s cannot match MEM exactly.** Both are normalised so ∫D dθ = 1, but the measured distribution is zero outside its partition while cos-2s has tails around the whole circle. A peak match of 90–97% is the structural limit, not a fitting failure.
- **Swell is inner, wind sea is outer.** Radius is frequency, so low-frequency swell sits near the centre with a tight lobe (high `s`), and locally generated wind sea sits toward the rim with a broad lobe (low `s`) aligned with the local wind. Cross-check against `--plot_wind` for the same date.
- Needs buoy directional data (`swdir` / `swr1` / `swr2`); not available with `--spectrum_file`.


---

## Small notes 

- **`nfrequencies` is computed** SEA-Stack's DFT eta import only evaluates the orthogonal Fourier frequencies `k/(N·dt)`, and strides — silently dropping energy — if `nfrequencies` is below the in-band bin count. The script sizes it to cover the whole default 0.001–1.0 Hz eta band.
- **Hm0 in the file header vs the statistics block** can differ by a percent or two because the header uses a plain trapezoid integration whereas the statistics use MHKiT's `significant_wave_height`. The eta record targets the former.
- **SSL / VPN.** If NDBC downloads fail with a certificate error, the `REQUESTS_CA_BUNDLE` and `SSL_CERT_FILE` lines near the top of the script point at a local root CA and may need updating for your machine.
- **Plot flags do not write YAML.** Add `--spectrum`, `--type irregular` if you want a plot *and* an update in the same run.
