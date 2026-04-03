<#
    Run SEA-Stack comparison tests and generate the summary report.

    Runs ctest with label comparison but excludes the CMake report target
    (comparison_report_generation), then runs generate_comparison_report.py
    with a runtime pandoc check (like run_regression_tests.ps1).

    Raw `ctest -L comparison` still runs the report target; this script does not.

    -BuildType matches ctest -C and report output under bin\<BuildType>\...
    Release is the supported mode for reports; non-Release may be incomplete.

    Does NOT rebuild -- run build.ps1 first if needed.

    Usage:
        .\scripts\windows\run_comparison_tests.ps1
        .\scripts\windows\run_comparison_tests.ps1 -j 6
        .\scripts\windows\run_comparison_tests.ps1 -NoPdf
        .\scripts\windows\run_comparison_tests.ps1 -Help
#>

param(
    [string]$BuildDir   = "build",
    [ValidateSet("Release", "Debug", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType  = "Release",
    [int]$j             = 0,
    [switch]$NoPdf,
    [switch]$Quiet,
    [switch]$Verbose,
    [switch]$Help
)

function Write-Step { param([string]$msg) Write-Host "`n>> $msg" -ForegroundColor Cyan }
function Write-OK   { param([string]$msg) Write-Host "   [OK] $msg" -ForegroundColor Green }
function Write-Warn { param([string]$msg) Write-Host "   [WARN] $msg" -ForegroundColor Yellow }

$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
. (Join-Path $repoRoot 'scripts\windows\SeaStackCtest.ps1')

if ($Help) {
    Write-Host "`nSEA-Stack Comparison Test Runner" -ForegroundColor Cyan
    Write-Host "=================================" -ForegroundColor Cyan
    Write-Host "`nRuns comparison tests then generates comparison_test_report (Markdown + optional PDF).`n"
    Write-Host "Skips CTest target comparison_report_generation; runs Python after ctest (runtime pandoc)."
    Write-Host "For CMake-time report behavior, use raw: ctest -L comparison`n"
    Write-Host "OPTIONS:" -ForegroundColor Yellow
    Write-Host "  -BuildDir <dir>    Build directory (default: build)"
    Write-Host "  -BuildType <type>  Release|Debug|RelWithDebInfo|MinSizeRel (default: Release)"
    Write-Host "  -j <count>         Parallel workers (0 = all processors)"
    Write-Host "  -NoPdf             Do not pass --pdf (default: --pdf if pandoc is on PATH)"
    Write-Host "  -Quiet             Minimal ctest output (-Q)"
    Write-Host "  -Verbose           Extra ctest detail (-V) and full Python report output"
    Write-Host "  (default)          Live ctest lines; parallel 'Start N:' lines are hidden"
    Write-Host ""
    Write-Host "EXAMPLES:" -ForegroundColor Yellow
    Write-Host "  .\scripts\windows\run_comparison_tests.ps1"
    Write-Host "  .\scripts\windows\run_comparison_tests.ps1 -j 6"
    Write-Host ""
    exit 0
}

$buildPath = Join-Path $repoRoot $BuildDir
$reportScript = Join-Path $repoRoot "tests\comparison\utilities\generate_comparison_report.py"
$outputDir = Join-Path $buildPath "bin\$BuildType\results\tests\comparison\report"

if (-not (Test-Path $buildPath)) {
    Write-Warn "Build directory not found: $buildPath"
    Write-Host "   Run build.ps1 first." -ForegroundColor Yellow
    exit 1
}

if ($j -eq 0) {
    $workers = [Environment]::ProcessorCount
    Write-Host "`nAuto-detected $workers workers" -ForegroundColor Gray
} else {
    $workers = $j
    Write-Host "`nUsing $workers workers" -ForegroundColor Gray
}

$ctestArgs = @(
    "-C", $BuildType,
    "-L", "comparison",
    "-E", "^comparison_report_generation$",
    "--test-dir", $BuildDir,
    "-j", $workers,
    "--output-on-failure"
)

Write-Step "Running comparison tests ($workers workers; report via Python after ctest)"

Push-Location $repoRoot
try {
    $suppressStart = (-not $Quiet -and -not $Verbose)
    $ctestExit = Invoke-SeaStackCtest -CtestArguments $ctestArgs -WorkingDirectory $repoRoot `
        -CtestQuiet:$Quiet -CtestVerbose:$Verbose -SuppressStartLines:$suppressStart
    if ($ctestExit -ne 0) {
        Write-Warn "Some comparison tests failed (exit code $ctestExit)"
    } else {
        Write-OK "All comparison tests passed"
    }

    Write-Step "Comparison report"

    $reportArgs = @(
        $reportScript,
        "--build-dir", $BuildDir,
        "--output-dir", $outputDir
    )
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

    $mdReport = Join-Path $outputDir "comparison_test_report.md"
    $pdfReport = Join-Path $outputDir "comparison_test_report.pdf"

    if (Test-Path $mdReport) {
        Write-OK ("Markdown: " + (Resolve-Path $mdReport).Path)
    }
    if (Test-Path $pdfReport) {
        Write-OK ("PDF:      " + (Resolve-Path $pdfReport).Path)
    }

    exit $ctestExit
} finally {
    Pop-Location
}
