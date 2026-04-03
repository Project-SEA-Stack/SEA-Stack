<#
    Run SEA-Stack benchmarks and display the comparison report.

    Runs ctest with label benchmark but excludes benchmark_report_generation
    (same idea as run_verification_tests.ps1), then runs generate_benchmark_report.py
    so the comparison table and paths are visible on stdout. Sequential (-j 1) by default.

    Does NOT rebuild -- run build.ps1 first if needed.

    Usage:
        .\scripts\windows\run_benchmarks.ps1                         # defaults (sequential, accurate timing)
        .\scripts\windows\run_benchmarks.ps1 -Baseline v0.1.0       # compare against a specific baseline
        .\scripts\windows\run_benchmarks.ps1 -BuildDir build_alt     # non-default build directory
#>

param(
    [string]$BuildDir   = "build",
    [ValidateSet("Release", "Debug", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType  = "Release",
    [string]$Baseline,
    [switch]$Quiet,
    [switch]$Verbose,
    [switch]$Help
)

function Write-Step  { param([string]$msg) Write-Host "`n>> $msg" -ForegroundColor Cyan }
function Write-OK    { param([string]$msg) Write-Host "   [OK] $msg" -ForegroundColor Green }
function Write-Warn  { param([string]$msg) Write-Host "   [WARN] $msg" -ForegroundColor Yellow }

$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
. (Join-Path $repoRoot 'scripts\windows\SeaStackCtest.ps1')

if ($Help) {
    Write-Host "`nSEA-Stack Benchmark Runner" -ForegroundColor Cyan
    Write-Host "==========================" -ForegroundColor Cyan
    Write-Host "`nRuns benchmark executables via ctest, then generates the benchmark report in Python.`n"
    Write-Host "Skips CTest target benchmark_report_generation; raw ctest -L benchmark still runs it.`n"
    Write-Host "OPTIONS:" -ForegroundColor Yellow
    Write-Host "  -BuildDir <dir>    Build directory (default: build)"
    Write-Host "  -BuildType <type>  Release|Debug|RelWithDebInfo|MinSizeRel (default: Release)"
    Write-Host "  -Baseline <tag>    Compare against a specific promoted baseline tag"
    Write-Host "                     (default: latest promoted file in data/benchmarks/history/)"
    Write-Host "  -Quiet             Minimal ctest output (-Q)"
    Write-Host "  -Verbose           Extra ctest detail (-V)"
    Write-Host "  (default)          Live ctest lines; parallel 'Start N:' lines are hidden"
    Write-Host ""
    Write-Host "EXAMPLES:" -ForegroundColor Yellow
    Write-Host "  .\scripts\windows\run_benchmarks.ps1"
    Write-Host "  .\scripts\windows\run_benchmarks.ps1 -Baseline v0.1.0-baseline"
    Write-Host "  .\scripts\windows\run_benchmarks.ps1 -BuildDir build -BuildType RelWithDebInfo"
    Write-Host ""
    exit 0
}

$buildPath  = Join-Path $repoRoot $BuildDir
$historyDir = Join-Path $repoRoot "data\benchmarks\history"
$reportScript = Join-Path $repoRoot "tests\benchmark\utilities\generate_benchmark_report.py"

if (-not (Test-Path $buildPath)) {
    Write-Warn "Build directory not found: $buildPath"
    Write-Host "   Run build.ps1 first." -ForegroundColor Yellow
    exit 1
}

# ── Run benchmarks ──────────────────────────────────────────────────────────

Write-Step "Running benchmarks (sequential, -j 1; report via Python after ctest)"

$ctestArgs = @(
    "-C", $BuildType,
    "-L", "benchmark",
    "-E", "^benchmark_report_generation$",
    "--test-dir", $BuildDir,
    "-j", "1",
    "--output-on-failure"
)

Push-Location $repoRoot
try {
    $suppressStart = (-not $Quiet -and -not $Verbose)
    $ctestExit = Invoke-SeaStackCtest -CtestArguments $ctestArgs -WorkingDirectory $repoRoot `
        -CtestQuiet:$Quiet -CtestVerbose:$Verbose -SuppressStartLines:$suppressStart
    if ($ctestExit -ne 0) {
        Write-Warn "Some benchmark tests failed (exit code $ctestExit)"
    } else {
        Write-OK "All benchmark tests passed"
    }

    # ── Generate report with visible stdout ─────────────────────────────────

    Write-Step "Benchmark report"

    $outputDir = Join-Path $buildPath "bin\$BuildType\results\tests\benchmark\report"
    $reportArgs = @(
        $reportScript,
        "--build-dir", $BuildDir,
        "--output-dir", $outputDir,
        "--history-dir", $historyDir
    )
    if ($Baseline) {
        $reportArgs += @("--baseline", $Baseline)
    }

    python @reportArgs
    $reportExit = $LASTEXITCODE
    # Exit: ctest failures take priority; if tests passed, report script exit code is used.
    if ($ctestExit -ne 0) { exit $ctestExit } else { exit $reportExit }

} finally {
    Pop-Location
}
