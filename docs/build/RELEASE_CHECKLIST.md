# SEA-Stack release checklist

Short, repeatable steps for cutting a prerelease or release (e.g. `v1.0.0-beta.N`).
Keep Windows and macOS in sync before tagging.

## 1. Branch and version bump

1. Create / check out `release/vX.Y.Z[-suffix]` from `main`.
2. Bump every product-facing version string:
   - [`CMakeLists.txt`](../../CMakeLists.txt) — `SEASTACK_VERSION_SUFFIX` (e.g. `"-rc.1"`, or `""` for a release)
   - [`README.md`](../../README.md) — badge + “current release” prose
   - [`TECHNICAL_OVERVIEW.md`](../../TECHNICAL_OVERVIEW.md) — version header and body mentions
   - [`scripts/windows/stage_release_reports.ps1`](../../scripts/windows/stage_release_reports.ps1) — usage examples (`-Version v…`)
3. Commit: `Bump version to vX.Y.Z[-suffix]`.

Keep `SEASTACK_VERSION_SUFFIX` in sync with the release branch name
(`release/v<PROJECT_VERSION><SUFFIX>`).

If release candidates are tagged on a branch whose suffix is already `""`,
their ZIPs carry the final version name (e.g. `SEAStack-1.0.0-win64.zip`).
Keep those packages internal; publish only the ZIPs built from the final tag.

## 2. Clean package (Windows)

From a Visual Studio / `vcvars64` environment:

```powershell
.\scripts\windows\build.ps1 -Clean -Package -MoorDyn -VSG -Demos
```

Expect: `build/SEAStack-<version>-win64.zip`. Confirm layout in
[PACKAGE_LAYOUT.md](PACKAGE_LAYOUT.md), then check the ZIP contents (demo
allowlist, `bin/run_seastack.exe` as the only executable, nothing forbidden):

```powershell
python scripts/check_package_contents.py build/SEAStack-<version>-win64.zip --platform win64
```

## 3. Clean package (macOS and Linux)

```bash
./scripts/unix/build.sh --clean --package --moordyn --vsg --demos
```

Expect: `build/SEAStack-<version>-darwin-*.zip` or
`build/SEAStack-<version>-linux-x64.zip`. Same content check as Windows:

```bash
python3 scripts/check_package_contents.py build/SEAStack-<version>-*.zip --platform darwin     # or linux-x64
```

On Linux, record the distro and version (`/etc/os-release`) and the glibc
version (`ldd --version`) with the release evidence; the glibc version is the
minimum the package runs on.

## 4. Full test suites (Windows)

After the packaged Release build exists (do **not** `-Clean` again before staging
reports):

```powershell
.\scripts\windows\run_unit_tests.ps1
.\scripts\windows\run_chrono_free_tests.ps1
.\scripts\windows\run_regression_tests.ps1
.\scripts\windows\run_verification_tests.ps1
.\scripts\windows\run_comparison_tests.ps1
.\scripts\windows\run_benchmarks.ps1
```

Repeat the equivalent runners on macOS via `scripts/unix/ctest_suite.sh`
(`unit`, `chrono-free`, `regression`, `verification`, `comparison`,
`benchmark`). If a fresh checkout reports `permission denied`, the scripts lost
their executable bit — run `chmod +x scripts/unix/*.sh` (or invoke with
`bash scripts/unix/...`).

The six suite runners do **not** cover the `external` label. Run it separately
on both platforms so the external-PTO Chrono regression is exercised:

```bash
ctest --test-dir build -C Release -L external --output-on-failure
```

**PDF reports:** suite scripts request PDF via pandoc when available. PDF needs a
LaTeX engine on PATH (`xelatex` / `pdflatex` / `lualatex`, e.g. MiKTeX on
Windows, MacTeX on macOS). Without it, suites still pass and write Markdown
only — but release staging expects the PDFs.

## 5. Stage release report assets

```powershell
.\scripts\windows\stage_release_reports.ps1 -Version vX.Y.Z-suffix
```

Copies regression / verification / comparison **PDF** reports into
`./release-assets/` (gitignored) with GitHub Release names, e.g.
`SEA-Stack-v1.0.0-regression-report.pdf`. Fails if any PDF is missing.

## 6. Staged-install smoke

From the install prefix produced by packaging (`build/install`), run one or two
representative demos with the staged `run_seastack` (and `PATH` set so bundled
DLLs / shared libraries resolve). Confirm the case starts cleanly and writes
outputs under the case directory.

Include at least one external-PTO demo, since it exercises the child-process
launch path (needs Python 3 on `PATH`; `python3` alone is fine on macOS/Linux):

```bash
./bin/run_seastack --nogui demos/rm3/external_pto
```

## 7. Downstream-consumer check

Installs a Chrono-free Debug SDK to a temporary prefix, builds
[`tests/consumer`](../../tests/consumer) against it with
`find_package(SEAStack)`, and runs it. On MSVC it also checks that the consumer
inherits `/arch:AVX2` and `_ENABLE_EXTENDED_ALIGNED_STORAGE` from
`SEAStack::Core` and runs under the CRT debug heap, which catches Eigen
alignment mismatches between the libraries and downstream code.

```powershell
.\scripts\windows\run_consumer_check.ps1    # VS x64 developer shell
```

```bash
./scripts/unix/run_consumer_check.sh        # --prefix-path for Eigen/HDF5 if not on the default path
```

## 8. Tagging and publishing

Conventions used in this project:

- Work on a **`release/v…` branch** (not directly on `main`), then open a PR and
  merge it into `main` when the cut is ready.
- Tag the release commit with an annotated tag matching the version string, e.g.
  `v1.0.0` (same stem as the branch / `SEASTACK_VERSION_FULL`).
- Beta / RC tags are published as **GitHub pre-releases**; attach the Windows and
  macOS ZIPs plus any staged reports from `release-assets/`.
- Prefer creating the GitHub Release in the web UI (or `gh release create`) after
  the tag exists — include a short summary of what changed since the previous
  tag.

## Related docs

| Doc | Role |
|-----|------|
| [BUILD_WINDOWS.md](BUILD_WINDOWS.md) / [BUILD_MACOS.md](BUILD_MACOS.md) | Machine setup and `build.ps1` / `build.sh` flags |
| [PACKAGE_LAYOUT.md](PACKAGE_LAYOUT.md) | What is inside the runtime ZIP |
| [QUICKSTART_RELEASE.txt](QUICKSTART_RELEASE.txt) | End-user notes installed as `QUICKSTART.txt` |
