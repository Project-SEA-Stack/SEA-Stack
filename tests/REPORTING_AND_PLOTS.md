# Test reporting and plot standards (SEA-Stack)

Aggregated Markdown/PDF reports and shared plot styling for regression, verification, and comparison suites. For a short summary and output paths, see [`README.md`](README.md).

## Report pipelines

Three test suites generate aggregated reports, each under its own test output tree:

- **Regression**: `tests/regression/utilities/generate_regression_report.py` (post-ctest; not a CTest target) → `build/bin/<Config>/results/tests/regression/report/regression_test_report.md` (+ PDF when `scripts/windows/run_regression_tests.ps1` finds `pandoc` on PATH, or when you pass `--pdf` manually)
- **Verification**: `generate_verification_report.py` → `build/bin/<Config>/results/tests/verification/report/verification_report.md` (+ PDF). PS1 runners write under `<BuildType>`; discovery is still most reliable for **Release**.
- **Comparison**: `tests/comparison/utilities/generate_comparison_report.py` → `build/bin/<Config>/results/tests/comparison/report/comparison_test_report.md` (+ PDF); same **Release** caveat.

All report generators use shared infrastructure from `tests/utilities/report_generator.py` for:

- Status file discovery and reading
- Plot file discovery
- Markdown generation helpers
- PDF conversion (via pandoc + LaTeX)
- Status aggregation

All reports derive pass/fail from `.status.json` files (never hardcoded). All support `--exit-on-failure`
for CI integration. PDF generation requires pandoc and a LaTeX distribution (MiKTeX, MacTeX, or TeX Live).

### PDF auto-generation

- **Regression:** `run_regression_tests.ps1` adds `--pdf` when `pandoc` is on PATH (`-NoPdf` to skip).
- **Verification / comparison (PowerShell):** `run_verification_tests.ps1` and `run_comparison_tests.ps1` do the same after ctest.
- **Verification / comparison (raw ctest):** CMake adds `--pdf` to the `*_report_generation` tests only if `pandoc` was found at **configure** time; reconfigure after installing pandoc if needed.

If pandoc/LaTeX is missing or conversion fails, Markdown-only output is still written.

Configure-time vs runtime pandoc behavior for verification/comparison is also described in [TEST_SUITES_REFERENCE.md](TEST_SUITES_REFERENCE.md) (raw `ctest` vs PowerShell).

## Plot standard

All test plots across regression and comparison suites follow a consistent visual standard defined in `tests/utilities/plot_helpers.py`. This ensures plots from different suites clearly look like part of the same SEA-Stack test/reporting system.

### Visual consistency

**Figure layout:**

- Standard figure size: 12x9 inches
- Absolute-positioned panels for metadata (test info, system info, data statistics, error metrics, simulation summary)
- Main comparison plot and error plot positioned using shared LAYOUT configuration
- 300 DPI output for all plots

**Series styling:**

- Primary trace (SEA-Stack/reference): `#007bff` (blue), solid line, 2.5pt width
- Secondary trace (baseline/comparison): `#dc3545` (red), dashed line, 2.5pt width
- Error trace: `#dc3545` (red), solid line, 2.0pt width
- Skip boundary markers: gray, dotted line, 0.5 alpha

**Axis styling:**

- X/Y labels: 11pt font, `#495057` color, medium weight
- Titles: 13pt font, `#212529` color, bold
- Grid: subtle gray (`#6c757d`), 0.2 alpha
- Modern spine styling with consistent colors

**Metadata panels:**

- Test Information: Model/executable, file paths, generation timestamp
- System Information: Platform, Python version, SEA-Stack version, Chrono version
- Data Statistics: Mean, std, range, correlation for both series
- Error Metrics: L₂ norm, L∞ norm, max error, mean error
- Simulation Summary: Duration, timestep, point count for both series

### Shared infrastructure

The plot standard is implemented through shared helpers in `tests/utilities/plot_helpers.py`:

- **Constants**: `SERIES_STYLES`, `AXIS_STYLE`, `FIGURE_DPI`, `LAYOUT`
- **Figure creation**: `create_standard_figure()` — builds figure with pre-positioned axes
- **Panel builders**: `build_info_panel()`, `build_system_panel()`, `build_error_metrics_panel()`, `build_data_stats_panel()`, `build_simulation_summary_panel()`
- **Styling**: `apply_modern_style()` — applies consistent grid, spines, and colors
- **Utilities**: `compute_error_metrics()`, `extract_units_from_label()`, `clip_to_common_time()`

### Suite-specific wrappers

While the visual presentation is standardized, suite-specific plotting wrappers (`create_comparison_plot` for regression, `plot_comparison` for comparison) preserve their distinct semantics:

- **Regression**: Compares against frozen SEA-Stack regression references (`ss_ref_*.txt`); **verification** compares against external codes (e.g. WEC-Sim, multicode datasets).
- **Comparison**: Compares two internal SEA-Stack methods against each other

Both wrappers call the same shared helpers, ensuring visual consistency while maintaining their unique data interpretation and labeling.

### Custom domain-specific plots

Some tests (e.g., RM3 hydraulic PTO internals) generate custom multi-panel figures for domain-specific analysis. These plots:

- Use `apply_modern_style()` for consistent axis styling
- Use `FIGURE_DPI` for output resolution
- Keep their custom layout (not forced into the standard LAYOUT structure)

This allows specialized visualizations while maintaining overall visual coherence.
