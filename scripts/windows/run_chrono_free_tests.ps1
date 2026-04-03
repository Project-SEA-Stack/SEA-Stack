<#
    Run SEA-Stack Chrono-free tests (CTest label: chrono-free).

    Subset of tests that do not require Chrono DLLs at runtime. Parallel by default.

    -BuildType aligns with ctest -C. Release is the validated mode.

    Usage:
        .\scripts\windows\run_chrono_free_tests.ps1
        .\scripts\windows\run_chrono_free_tests.ps1 -j 6
        .\scripts\windows\run_chrono_free_tests.ps1 -Help
#>

param(
    [string]$BuildDir   = "build",
    [ValidateSet("Release", "Debug", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType  = "Release",
    [int]$j             = 0,
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
    Write-Host "`nSEA-Stack Chrono-Free Test Runner" -ForegroundColor Cyan
    Write-Host "=================================" -ForegroundColor Cyan
    Write-Host "`nRuns CTest tests with label `chrono-free` (no Chrono DLLs needed).`n"
    Write-Host "OPTIONS:" -ForegroundColor Yellow
    Write-Host "  -BuildDir <dir>    Build directory (default: build)"
    Write-Host "  -BuildType <type>  Release|Debug|RelWithDebInfo|MinSizeRel (default: Release)"
    Write-Host "  -j <count>         Parallel workers (0 = all processors)"
    Write-Host "  -Quiet             Minimal ctest output (-Q)"
    Write-Host "  -Verbose           Extra ctest detail (-V)"
    Write-Host "  (default)          Live ctest lines; parallel 'Start N:' lines are hidden"
    Write-Host ""
    Write-Host "EXAMPLES:" -ForegroundColor Yellow
    Write-Host "  .\scripts\windows\run_chrono_free_tests.ps1"
    Write-Host "  .\scripts\windows\run_chrono_free_tests.ps1 -j 6"
    Write-Host ""
    exit 0
}

$buildPath = Join-Path $repoRoot $BuildDir
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

Write-Step "Running chrono-free tests ($workers workers)"

$ctestArgs = @(
    "-C", $BuildType,
    "-L", "chrono-free",
    "--test-dir", $BuildDir,
    "-j", $workers,
    "--output-on-failure"
)

Push-Location $repoRoot
try {
    $suppressStart = (-not $Quiet -and -not $Verbose)
    $exitCode = Invoke-SeaStackCtest -CtestArguments $ctestArgs -WorkingDirectory $repoRoot `
        -CtestQuiet:$Quiet -CtestVerbose:$Verbose -SuppressStartLines:$suppressStart
    if ($exitCode -ne 0) {
        Write-Warn "Some chrono-free tests failed (exit code $exitCode)"
    } else {
        Write-OK "All chrono-free tests passed"
    }
    exit $exitCode
} finally {
    Pop-Location
}
