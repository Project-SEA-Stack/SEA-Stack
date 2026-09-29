<#
.SYNOPSIS
  Downstream-consumer check: install a Chrono-free Debug SEA-Stack SDK, build
  tests/consumer against it with find_package(SEAStack), and run it under the
  MSVC debug heap.

.DESCRIPTION
  Guards the exported build contract of SEAStack::Core (MSVC: /arch:AVX2 and
  _ENABLE_EXTENDED_ALIGNED_STORAGE, i.e. the same Eigen alignment as the
  libraries). A mismatch shows up as a CRT heap assertion or a flag check failure.

  Run from a Visual Studio x64 developer shell (cl and ninja on PATH).
  Work files go to a new directory under %TEMP% unless -WorkDir is given.

.PARAMETER PrefixPath
  Extra CMAKE_PREFIX_PATH for Eigen3/HDF5 (e.g. C:/vcpkg/installed/x64-windows).
  Default: derived from HDF5Dir in build-config.json when it ends in share/hdf5.

.EXAMPLE
  .\scripts\windows\run_consumer_check.ps1
  .\scripts\windows\run_consumer_check.ps1 -PrefixPath C:/vcpkg/installed/x64-windows
#>
param(
    [string]$WorkDir = "",
    [string]$PrefixPath = "",
    [string]$ConfigPath = ""
)
# Native tools write progress to stderr; failures are detected via exit codes below.
$ErrorActionPreference = 'Continue'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
if (-not $ConfigPath) { $ConfigPath = Join-Path $repoRoot "build-config.json" }

function Fail([string]$msg) { Write-Host "[FAIL] $msg" -ForegroundColor Red; exit 1 }

foreach ($tool in 'cl', 'ninja', 'cmake') {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        Fail "$tool not on PATH. Run from a Visual Studio x64 developer shell."
    }
}

$hdf5Dir = ""
if (Test-Path $ConfigPath) {
    $cfg = Get-Content $ConfigPath -Raw | ConvertFrom-Json
    if ($cfg.PSObject.Properties.Name -contains 'HDF5Dir' -and $cfg.HDF5Dir) { $hdf5Dir = $cfg.HDF5Dir }
}
if (-not $PrefixPath -and $hdf5Dir -match '[\\/]share[\\/]hdf5[\\/]?$') {
    $PrefixPath = (Split-Path (Split-Path $hdf5Dir -Parent) -Parent) -replace '\\', '/'
}

if (-not $WorkDir) {
    $WorkDir = Join-Path $env:TEMP ("seastack-consumer-check-" + (Get-Date -Format 'yyyyMMdd-HHmmss'))
}
if (Test-Path $WorkDir) { Fail "WorkDir already exists: $WorkDir" }
New-Item -ItemType Directory -Path $WorkDir | Out-Null
$sdkBuild = Join-Path $WorkDir "sdk-build"
$sdk = Join-Path $WorkDir "sdk"
$consumerBuild = Join-Path $WorkDir "consumer-build"
$h5 = Join-Path $repoRoot "data\demos\run_seastack\rm3\assets\hydroData\rm3.h5"

Write-Host "Repo:        $repoRoot"
Write-Host "WorkDir:     $WorkDir"
Write-Host "PrefixPath:  $PrefixPath"
Write-Host "HDF5_DIR:    $hdf5Dir"

$common = @()
if ($hdf5Dir) { $common += "-DHDF5_DIR=$hdf5Dir" }

Write-Host "`n>> Configure + install Chrono-free Debug SDK"
$sdkArgs = @('-S', $repoRoot, '-B', $sdkBuild, '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Debug',
    "-DCMAKE_INSTALL_PREFIX=$sdk", '-DSEASTACK_ENABLE_CHRONO=OFF', '-DSEASTACK_ENABLE_APPS=OFF',
    '-DSEASTACK_ENABLE_TESTS=OFF', '-DSEASTACK_ENABLE_HYDRO_IO=ON') + $common
if ($PrefixPath) { $sdkArgs += "-DCMAKE_PREFIX_PATH=$PrefixPath" }
& cmake @sdkArgs | Select-Object -Last 1
if ($LASTEXITCODE -ne 0) { Fail "SDK configure" }
& cmake --build $sdkBuild | Select-Object -Last 1
if ($LASTEXITCODE -ne 0) { Fail "SDK build" }
& cmake --install $sdkBuild | Out-Null
if ($LASTEXITCODE -ne 0) { Fail "SDK install" }

Write-Host "`n>> Configure + build consumer (Debug) against the installed SDK"
$consumerPrefix = if ($PrefixPath) { "$sdk;$PrefixPath" } else { $sdk }
$consumerArgs = @('-S', (Join-Path $repoRoot "tests\consumer"), '-B', $consumerBuild, '-G', 'Ninja',
    '-DCMAKE_BUILD_TYPE=Debug', "-DCMAKE_PREFIX_PATH=$consumerPrefix") + $common
& cmake @consumerArgs | Select-Object -Last 1
if ($LASTEXITCODE -ne 0) { Fail "consumer configure" }
& cmake --build $consumerBuild | Select-Object -Last 1
if ($LASTEXITCODE -ne 0) { Fail "consumer build" }

$ninjaFile = Get-Content (Join-Path $consumerBuild "build.ninja") -Raw
if ($ninjaFile -notmatch '/arch:AVX2') { Fail "consumer compile flags lack /arch:AVX2 (SEAStack::Core INTERFACE)" }
if ($ninjaFile -notmatch '_ENABLE_EXTENDED_ALIGNED_STORAGE') { Fail "consumer defines lack _ENABLE_EXTENDED_ALIGNED_STORAGE" }
Write-Host "   [OK] consumer inherits /arch:AVX2 and _ENABLE_EXTENDED_ALIGNED_STORAGE"

Write-Host "`n>> Run consumer under the CRT debug heap"
if ($PrefixPath) {
    $p = $PrefixPath -replace '/', '\'
    $env:PATH = "$p\debug\bin;$p\bin;$env:PATH"
}
$exe = Join-Path $consumerBuild "consumer_check.exe"
$output = & $exe $h5 2>&1 | ForEach-Object { "$_" }
$code = $LASTEXITCODE
$output | ForEach-Object { Write-Host "   $_" }
if ($code -ne 0 -or -not ($output -match 'consumer OK')) { Fail ("consumer run exit code {0} (0x{0:X8})" -f $code) }

Write-Host "`n[OK] Downstream-consumer check passed (work files: $WorkDir)" -ForegroundColor Green
exit 0
