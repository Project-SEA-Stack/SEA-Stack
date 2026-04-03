# SEA-Stack scripts

| Location | Use |
|----------|-----|
| [`unix/`](unix/) | Bash drivers for **macOS and Linux** (build, CTest wrappers, demo smoke). |
| [`windows/`](windows/) | **PowerShell** equivalents for Windows. |
| Repo root of `scripts/` | Cross-platform **Python** helpers (`seastack_build_prepare.py`, `validate_campaign_schema.py`). |

From the **sea-stack** repository root:

- **Unix:** `./scripts/unix/build.sh`, `./scripts/unix/ctest_suite.sh <suite>`, etc.
- **Windows:** `.\scripts\windows\build.ps1`, `.\scripts\windows\run_unit_tests.ps1`, etc.

**Running tests:** Prefer `ctest_suite.sh` (Unix) or `run_regression_tests.ps1` /
`run_unit_tests.ps1` / … (Windows) over raw `ctest` or `build.ps1 -Test` alone —
the wrappers align the build configuration, parallelism, and (where
applicable) Python report steps. **Optional:** install **pandoc** and a LaTeX
distribution on `PATH`; then those scripts request PDF reports as well as
Markdown. Without pandoc, reports remain **`.md`-only** (figures still listed
in the Markdown). Use `--no-pdf` / `-NoPdf` to skip PDF.

`build-config.json` and CMake behavior are shared between `unix/build.sh` and `windows/build.ps1`.
