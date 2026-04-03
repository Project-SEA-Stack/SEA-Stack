<#
    Run SEA-Stack regression tests and display the summary report.

    Thin wrapper around ctest + generate_regression_report.py. Runs the regression suite
    with maximum parallelism by default (all available workers), then runs the
    report script after ctest finishes (regression report is not a CTest target).

    Does NOT rebuild -- run build.ps1 first if needed.

    Usage:
        .\scripts\windows\run_regression_tests.ps1                    # all workers, full suite
        .\scripts\windows\run_regression_tests.ps1 -j 6                # limit parallelism
        .\scripts\windows\run_regression_tests.ps1 -Filter sphere      # model filter
        .\scripts\windows\run_regression_tests.ps1 -Recompare          # regenerate comparison plots
        .\scripts\windows\run_regression_tests.ps1 -Long               # enable long regression tests (SEASTACK_LONG_TESTS)
        .\scripts\windows\run_regression_tests.ps1 -Help               # usage info
#>

param(
    [string]$BuildDir   = "build",
    [ValidateSet("Release", "Debug", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType  = "Release",
    [int]$j             = 0,  # 0 = auto-detect (all processors), otherwise use specified count
    [string]$Filter,           # Filter by model (sphere, oswec, f3of, rm3)
    [switch]$Recompare,        # Re-run comparison tests to regenerate plots
    [switch]$Long,             # Set SEASTACK_LONG_TESTS=1 for this run only
    [switch]$NoPdf,            # Skip PDF even if pandoc is on PATH (e.g. LaTeX missing)
    [switch]$Quiet,            # ctest -Q only (minimal live output); report stays --quiet unless -Verbose
    [switch]$Verbose,          # ctest -V + full Python report output (debug)
    [switch]$Help
)

function Write-Step  { param([string]$msg) Write-Host "`n>> $msg" -ForegroundColor Cyan }
function Write-OK    { param([string]$msg) Write-Host "   [OK] $msg" -ForegroundColor Green }
function Write-Warn  { param([string]$msg) Write-Host "   [WARN] $msg" -ForegroundColor Yellow }
function Write-Fail  { param([string]$msg) Write-Host "   [FAIL] $msg" -ForegroundColor Red }

$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
. (Join-Path $repoRoot 'scripts\windows\SeaStackCtest.ps1')

if ($Help) {
    Write-Host "`nSEA-Stack Regression Test Runner" -ForegroundColor Cyan
    Write-Host "=================================" -ForegroundColor Cyan
    Write-Host "`nRuns the regression test suite and displays a summary report.`n"
    Write-Host "OPTIONS:" -ForegroundColor Yellow
    Write-Host "  -BuildDir <dir>    Build directory (default: build)"
    Write-Host "  -BuildType <type>  Release|Debug|RelWithDebInfo|MinSizeRel (default: Release)"
    Write-Host "  -j <count>         Number of parallel workers (default: all available processors)"
    Write-Host "  -Filter <model>    Filter by family (CTest prefix test_regression_<model>_)"
    Write-Host "  -Recompare         Re-run comparison tests to regenerate plots"
    Write-Host "  -Long              Enable long regression tests (SEASTACK_LONG_TESTS=1 for this run)"
    Write-Host "  -NoPdf             Do not pass --pdf (default: --pdf if pandoc is on PATH)"
    Write-Host "  -Quiet             Minimal ctest output (-Q)"
    Write-Host "  -Verbose           Extra ctest detail (-V) and full Python report output"
    Write-Host "  (default)          Live ctest lines; parallel 'Start N:' lines are hidden"
    Write-Host ""
    Write-Host "EXAMPLES:" -ForegroundColor Yellow
    Write-Host "  .\scripts\windows\run_regression_tests.ps1"
    Write-Host "  .\scripts\windows\run_regression_tests.ps1 -j 6"
    Write-Host "  .\scripts\windows\run_regression_tests.ps1 -Filter sphere"
    Write-Host "  .\scripts\windows\run_regression_tests.ps1 -Recompare"
    Write-Host "  .\scripts\windows\run_regression_tests.ps1 -Long"
    Write-Host ""
    exit 0
}

$buildPath = Join-Path $repoRoot $BuildDir
$reportScript = Join-Path $repoRoot "tests\regression\utilities\generate_regression_report.py"

if (-not (Test-Path $buildPath)) {
    Write-Warn "Build directory not found: $buildPath"
    Write-Host "   Run build.ps1 first." -ForegroundColor Yellow
    exit 1
}

# Determine worker count
if ($j -eq 0) {
    $workers = [Environment]::ProcessorCount
    Write-Host "`nAuto-detected $workers workers" -ForegroundColor Gray
} else {
    $workers = $j
    Write-Host "`nUsing $workers workers" -ForegroundColor Gray
}

# ── Run regression tests ───────────────────────────────────────────────────────

$ctestArgs = @(
    "-C", $BuildType,
    "-L", "regression",
    "--test-dir", $BuildDir,
    "-j", $workers,
    "--output-on-failure"
)

if ($Filter) {
    $ctestArgs += @("-R", "^test_regression_${Filter}_")
    Write-Step "Running regression tests (filter: $Filter, $workers workers)"
} else {
    Write-Step "Running regression tests ($workers workers)"
}

$priorLongTests = $env:SEASTACK_LONG_TESTS
Push-Location $repoRoot
try {
    if ($Long) {
        $env:SEASTACK_LONG_TESTS = '1'
        Write-Host "`nSEASTACK_LONG_TESTS=1 (long regression tests enabled for this run)" -ForegroundColor Gray
    }

    $suppressStart = (-not $Quiet -and -not $Verbose)
    $ctestExit = Invoke-SeaStackCtest -CtestArguments $ctestArgs -WorkingDirectory $repoRoot `
        -CtestQuiet:$Quiet -CtestVerbose:$Verbose -SuppressStartLines:$suppressStart
    if ($ctestExit -ne 0) {
        Write-Warn "Some regression tests failed (exit code $ctestExit)"
    } else {
        Write-OK "All regression tests passed"
    }

    # ── Generate report with visible stdout ─────────────────────────────────

    Write-Step "Regression report"

    $reportArgs = @(
        $reportScript,
        "--build-dir", $BuildDir,
        "--config", $BuildType
    )
    if ($Recompare) {
        $reportArgs += "--recompare"
    }
    if (-not $Verbose) {
        $reportArgs += "--quiet"
    }
    $pandocCmd = Get-Command pandoc -ErrorAction SilentlyContinue
    if (-not $NoPdf -and $pandocCmd) {
        Write-Host "   pandoc found -- requesting PDF" -ForegroundColor Gray
        $reportArgs += "--pdf"
    } elseif (-not $NoPdf) {
        Write-Host "   pandoc not on PATH -- Markdown only" -ForegroundColor Gray
    }

    python @reportArgs

    $reportDir = Join-Path $buildPath "bin\$BuildType\results\tests\regression\report"
    $markdownReport = Join-Path $reportDir "regression_test_report.md"
    $pdfReport = Join-Path $reportDir "regression_test_report.pdf"

    if (Test-Path $markdownReport) {
        Write-OK ("Markdown: " + (Resolve-Path $markdownReport).Path)
    }
    if (Test-Path $pdfReport) {
        Write-OK ("PDF:      " + (Resolve-Path $pdfReport).Path)
    }

    # Exit with CTest's exit code so CI can detect failures
    exit $ctestExit

} finally {
    if ($Long) {
        if ([string]::IsNullOrEmpty($priorLongTests)) {
            Remove-Item Env:\SEASTACK_LONG_TESTS -ErrorAction SilentlyContinue
        } else {
            $env:SEASTACK_LONG_TESTS = $priorLongTests
        }
    }
    Pop-Location
}
