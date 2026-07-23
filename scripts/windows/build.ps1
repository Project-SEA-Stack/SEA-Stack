<#
    SEA-Stack Build Script

    Unix / macOS: use scripts/unix/build.sh (same build-config.json schema and CMake options).

    Configures, builds, and optionally tests SEA-Stack.  For Chrono builds,
    set ChronoDir in build-config.json; CMake discovers the package (either
    ChronoConfig.cmake or chrono-config.cmake).  Use -NoChrono for a
    Chrono-free build (no config file required).

    Default workflow: CMake configure is rerunnable; cmake --build performs
    incremental compiles.  Use -Clean when changing generator, toolchain, or
    major options.  Use -NoConfigure to build only against an existing cache.

    Usage:
        .\scripts\windows\build.ps1
        .\scripts\windows\build.ps1 -Doctor
        .\scripts\windows\build.ps1 -NoChrono
        .\scripts\windows\build.ps1 -Generator Ninja
        .\scripts\windows\build.ps1 -NoConfigure -Target test_linear_pto
        .\scripts\windows\build.ps1 -Clean -Verbose

    Configuration:
        Copy build-config.example.json to build-config.json and edit paths.
        Optional: "Generator" in JSON (used when CLI -Generator is omitted).
#>

param(
    [ValidateSet("Release", "Debug", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType = "Release",

    [switch]$Clean,
    [switch]$Verbose,
    [switch]$ConfigureOnly,
    [switch]$Reconfigure,
    [switch]$NoConfigure,
    [switch]$Test,
    [string]$TestLabel,
    [switch]$Package,
    [string]$Target,

    [string]$Generator,
    [ValidateSet("x64", "Win32", "ARM64")]
    [string]$Arch = "x64",

    [switch]$NoChrono,
    [switch]$NoHydroIO,
    [switch]$MoorDyn,
    [switch]$External,
    [switch]$VSG,
    [switch]$Vehicle,
    [switch]$Demos,
    [switch]$NoTests,
    [switch]$NoApps,

    [string]$ConfigPath = ".\build-config.json",
    [switch]$Doctor,
    [switch]$Help
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$SCRIPT_VERSION = "1.0"
$SEASTACK_CMAKE_MIN = [version]'3.21.0'

# =============================================================================
# Output helpers
# =============================================================================

$script:SeaStackStepIndex = 0
$script:SeaStackStepTotal = 0

function Write-SeaStackStep {
    param([string]$Message)
    $script:SeaStackStepIndex++
    if ($script:SeaStackStepTotal -gt 0) {
        Write-Host "`n[$($script:SeaStackStepIndex)/$($script:SeaStackStepTotal)] $Message" -ForegroundColor Cyan
    } else {
        Write-Host "`n>> $Message" -ForegroundColor Cyan
    }
}

function Write-OK     { param([string]$msg) Write-Host ('   [OK] ' + $msg) -ForegroundColor Green }
function Write-Warn   { param([string]$msg) Write-Host ('   [WARN] ' + $msg) -ForegroundColor Yellow }
function Write-Fail   { param([string]$msg) Write-Host ('   [FAIL] ' + $msg) -ForegroundColor Red }
function Write-Detail { param([string]$msg) Write-Host ('   ' + $msg) -ForegroundColor Gray }

function Get-SeaStackDiagLogPath {
    param([string]$RepoRoot)
    Join-Path $RepoRoot "build\build-diagnostics.log"
}

function Initialize-SeaStackDiagLog {
    param(
        [string]$Path,
        [string]$Title
    )
    $dir = Split-Path -Parent $Path
    if (-not (Test-Path $dir)) {
        New-Item -ItemType Directory -Path $dir -Force | Out-Null
    }
    $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    @(
        "========================================",
        $Title,
        "Time: $ts",
        "Script: $SCRIPT_VERSION",
        "PSVersion: $($PSVersionTable.PSVersion)",
        "OS: $([System.Environment]::OSVersion.VersionString)",
        "========================================",
        ""
    ) | Add-Content -LiteralPath $Path -Encoding UTF8
}

function Add-SeaStackDiagLog {
    param(
        [string]$Path,
        [object]$Line
    )
    if ($null -eq $Line) { return }
    if ($Line -is [array]) {
        $Line | ForEach-Object { Add-SeaStackDiagLog -Path $Path -Line $_ }
        return
    }
    # -Clean removes build/; later steps still log — ensure parent dir exists
    $dir = Split-Path -Parent $Path
    if (-not (Test-Path -LiteralPath $dir)) {
        New-Item -ItemType Directory -Path $dir -Force | Out-Null
    }
    Add-Content -LiteralPath $Path -Encoding UTF8 -Value $Line
}

# =============================================================================
# External tools (exit codes, logging)
# =============================================================================

function Invoke-SeaStackTool {
    param(
        [Parameter(Mandatory)][string]$StepName,
        [Parameter(Mandatory)][string]$Executable,
        [AllowEmptyCollection()][string[]]$Arguments = @(),
        [Parameter(Mandatory)][string]$DiagLogPath,
        [bool]$ShowOutputOnHost,
        [switch]$SoftFail,
        [string]$FailHint = "Re-run with -Verbose. See diagnostics log for full output."
    )

    $argDisplay = ($Arguments | ForEach-Object {
        if ($_ -match '\s') { "`"$_`"" } else { $_ }
    }) -join ' '
    $cmdLine = "$Executable $argDisplay"
    Add-SeaStackDiagLog -Path $DiagLogPath -Line "---- $StepName ----"
    Add-SeaStackDiagLog -Path $DiagLogPath -Line $cmdLine

    $prevEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        if (-not (Get-Command $Executable -ErrorAction SilentlyContinue)) {
            Write-Fail "${StepName}: executable not found: $Executable"
            Add-SeaStackDiagLog -Path $DiagLogPath -Line "ERROR: $Executable not on PATH"
            return $false
        }

        $captured = & $Executable @Arguments 2>&1
        $code = $LASTEXITCODE
        foreach ($line in $captured) {
            Add-SeaStackDiagLog -Path $DiagLogPath -Line "$line"
            if ($ShowOutputOnHost) {
                Write-Host ([string]$line)
            }
        }
        if (-not $ShowOutputOnHost -and $code -ne 0) {
            Write-Host ($captured -join "`n")
        }
    } finally {
        $ErrorActionPreference = $prevEap
    }

    Add-SeaStackDiagLog -Path $DiagLogPath -Line "Exit code: $code"
    if ($code -ne 0) {
        if ($SoftFail) {
            Write-Warn "${StepName} finished with exit $code"
        } else {
            Write-Fail "${StepName} failed (exit $code)"
            Write-Host ('   ' + $FailHint) -ForegroundColor Yellow
            Write-Host ('   Log: ' + $DiagLogPath) -ForegroundColor Yellow
        }
        return $false
    }
    return $true
}

# =============================================================================
# CMake / generator helpers
# =============================================================================

function Get-CMakeCacheGenerator {
    param([string]$BuildDir)
    $cacheFile = Join-Path $BuildDir "CMakeCache.txt"
    if (-not (Test-Path -LiteralPath $cacheFile)) {
        return $null
    }
    foreach ($line in Get-Content -LiteralPath $cacheFile) {
        if ($line -match '^CMAKE_GENERATOR:INTERNAL=(.+)$') {
            return $Matches[1].Trim()
        }
    }
    return $null
}

# Read a single CMake cache entry (PATH, STRING, FILEPATH, or UNINITIALIZED).
function Get-CMakeCacheVariable {
    param(
        [string]$BuildDir,
        [string]$Name
    )
    $cacheFile = Join-Path $BuildDir "CMakeCache.txt"
    if (-not (Test-Path -LiteralPath $cacheFile)) {
        return $null
    }
    $escaped = [regex]::Escape($Name)
    foreach ($line in Get-Content -LiteralPath $cacheFile) {
        if ($line -match "^${escaped}:(?:PATH|STRING|FILEPATH|BOOL|UNINITIALIZED)=(.*)$") {
            return $Matches[1].Trim()
        }
    }
    return $null
}

# Infer VSG DLL directory from ChronoConfig.cmake vsg_DIR when CMake did not set VSG_DLL_DIR.
function Write-SeaStackChronoParsersPythonDllCheck {
    param(
        [Parameter(Mandatory)][string]$BinDir,
        [Parameter(Mandatory)][string]$DiagLogPath
    )
    $parsers = Join-Path $BinDir 'Chrono_parsers.dll'
    if (-not (Test-Path -LiteralPath $parsers)) {
        return
    }
    $dumpbinCmd = Get-Command dumpbin -ErrorAction SilentlyContinue
    if (-not $dumpbinCmd) {
        Write-Detail 'dumpbin not on PATH; skipping Chrono_parsers Python import check'
        Add-SeaStackDiagLog -Path $DiagLogPath -Line 'Chrono_parsers Python check skipped (no dumpbin)'
        return
    }
    $raw = & dumpbin /nologo /dependents $parsers 2>&1
    $imp = $null
    foreach ($line in $raw) {
        if ([string]$line -match '(?i)\b(python\d{2,}\.dll)\b') {
            $imp = $Matches[1].ToLowerInvariant()
            break
        }
    }
    if (-not $imp) {
        return
    }
    Add-SeaStackDiagLog -Path $DiagLogPath -Line "Chrono_parsers.dll imports: $imp"
    $targetPath = Join-Path $BinDir $imp
    if (-not (Test-Path -LiteralPath $targetPath)) {
        Write-Warn "Chrono_parsers.dll imports $imp but that file is not in the same folder as the app ($BinDir). Loader may fail (0xC0000135)."
    }
}

function Resolve-SeaStackVsgDllDirFromChronoVsgDir {
    param([string]$VsgCmakeDir)
    if ([string]::IsNullOrWhiteSpace($VsgCmakeDir)) {
        return $null
    }
    $vsgParent = Split-Path -Parent $VsgCmakeDir
    $parentLeaf = Split-Path -Leaf $VsgParent
    # vcpkg: .../installed/<triplet>/share/vsg -> DLLs in <triplet>/bin
    if ((Split-Path -Leaf $VsgCmakeDir) -eq 'vsg' -and $parentLeaf -eq 'share') {
        $vsgRoot = Split-Path -Parent $VsgParent
    } else {
        # Typical CMake install: .../lib/cmake/vsg -> prefix/bin
        $vsgRoot = Split-Path (Split-Path $vsgParent)
    }
    if ([string]::IsNullOrWhiteSpace($vsgRoot)) {
        return $null
    }
    return (Join-Path $vsgRoot "bin")
}

function Expand-SeaStackGeneratorAlias {
    param([string]$Name)
    if ([string]::IsNullOrWhiteSpace($Name)) {
        return $Name
    }
    $n = $Name.Trim()
    switch -Regex ($n) {
        '^(?i)ninja$' { return 'Ninja' }
        '^(?i)vs2022|vs-2022$' { return 'Visual Studio 17 2022' }
        '^(?i)vs2019|vs-2019$' { return 'Visual Studio 16 2019' }
        '^(?i)vs2026|vs-2026$' { return 'Visual Studio 18 2026' }
        default { return $n }
    }
}

function Find-SeaStackAutoGenerator {
    param([string]$DiagLogPath)

    if (Get-Command ninja -ErrorAction SilentlyContinue) {
        Add-SeaStackDiagLog -Path $DiagLogPath -Line "Auto-detect: Ninja found on PATH"
        return @{ Name = 'Ninja'; MultiConfig = $false }
    }

    $pf86 = ${env:ProgramFiles(x86)}
    if ([string]::IsNullOrEmpty($pf86)) {
        $pf86 = "C:\Program Files (x86)"
    }
    $vswhere = Join-Path $pf86 "Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path -LiteralPath $vswhere)) {
        Add-SeaStackDiagLog -Path $DiagLogPath -Line "Auto-detect: vswhere.exe not found"
        return $null
    }

    $installVer = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion -format value 2>$null
    if (-not $installVer) {
        $installVer = & $vswhere -latest -products * -property installationVersion -format value 2>$null
    }
    if (-not $installVer) {
        Add-SeaStackDiagLog -Path $DiagLogPath -Line "Auto-detect: no Visual Studio installation reported by vswhere"
        return $null
    }

    $major = [int]($installVer.Split('.')[0])
    Add-SeaStackDiagLog -Path $DiagLogPath -Line "Auto-detect: VS installationVersion=$installVer (major=$major)"

    # Generator strings must match the installed CMake; verify with cmake --help when adding new majors.
    if ($major -ge 18) {
        return @{ Name = 'Visual Studio 18 2026'; MultiConfig = $true }
    }
    if ($major -ge 17) {
        return @{ Name = 'Visual Studio 17 2022'; MultiConfig = $true }
    }
    if ($major -ge 16) {
        return @{ Name = 'Visual Studio 16 2019'; MultiConfig = $true }
    }
    return $null
}

function Resolve-SeaStackGenerator {
    param(
        [string]$CliGenerator,
        [string]$ConfigFileGenerator,
        [string]$BuildDir,
        [bool]$CleanRequested,
        [string]$DiagLogPath
    )

    $cached = if (-not $CleanRequested) { Get-CMakeCacheGenerator -BuildDir $BuildDir } else { $null }
    $cli = Expand-SeaStackGeneratorAlias -Name $CliGenerator
    $fromConfig = Expand-SeaStackGeneratorAlias -Name $ConfigFileGenerator

    Add-SeaStackDiagLog -Path $DiagLogPath -Line "Generator resolution: CLI='$cli' Config='$fromConfig' Cached='$cached'"

    if ($cli) {
        if ($cached -and ($cached -ne $cli)) {
            Write-Fail "Generator mismatch: build tree was configured with ``$cached`` but -Generator requests ``$cli``."
            Write-Host "   Run with -Clean to reconfigure, or omit -Generator to keep using the cached generator." -ForegroundColor Yellow
            Add-SeaStackDiagLog -Path $DiagLogPath -Line "ERROR: generator mismatch cached vs CLI"
            return $null
        }
        return @{ Name = $cli; MultiConfig = ($cli -match 'Visual Studio') }
    }

    if (-not $CleanRequested -and $cached) {
        return @{ Name = $cached; MultiConfig = ($cached -match 'Visual Studio') }
    }

    if ($fromConfig) {
        return @{ Name = $fromConfig; MultiConfig = ($fromConfig -match 'Visual Studio') }
    }

    $auto = Find-SeaStackAutoGenerator -DiagLogPath $DiagLogPath
    if (-not $auto) {
        Write-Fail "Could not auto-detect a CMake generator."
        Write-Host "   Install Visual Studio (Desktop development with C++) or Ninja, or pass -Generator explicitly." -ForegroundColor Yellow
        Add-SeaStackDiagLog -Path $DiagLogPath -Line "ERROR: auto-detect failed"
        return $null
    }
    if (-not $auto.ContainsKey('MultiConfig')) {
        $auto.MultiConfig = ($auto.Name -match 'Visual Studio')
    }
    return $auto
}

# Optional Chrono package file for pre-configure heuristics only (Eigen/HDF5/VSG parse, DLL copy).
function Get-ChronoPackageConfigPathOptional {
    param([string]$ChronoDir)
    if ([string]::IsNullOrWhiteSpace($ChronoDir) -or -not (Test-Path -LiteralPath $ChronoDir)) {
        return $null
    }
    $lower = Join-Path $ChronoDir "chrono-config.cmake"
    $pascal = Join-Path $ChronoDir "ChronoConfig.cmake"
    if (Test-Path -LiteralPath $lower) { return $lower }
    if (Test-Path -LiteralPath $pascal) { return $pascal }
    return $null
}

# =============================================================================
# Prerequisites
# =============================================================================

function Write-SeaStackPythonPrerequisiteStatus {
    # StrictMode off in this helper only: avoids VariableIsUndefined on some hosts
    # when combining Get-Command results with string concat (caller stays strict).
    Set-StrictMode -Off
    $pyRes = Get-Command python -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $pyRes) {
        Write-Warn 'Python not on PATH - benchmark report / some tests may fail'
    } else {
        $pySrc = $pyRes.Source
        Write-OK ('Python: ' + $pySrc)
    }
}

function Test-SeaStackPrerequisites {
    param(
        [version]$MinCMake,
        [string]$ResolvedGeneratorName,
        [bool]$NeedsPython,
        [bool]$DoctorMode,
        [string]$DiagLogPath
    )

    if ($PSVersionTable.PSVersion.Major -lt 5) {
        Write-Warn "PowerShell 7+ is recommended; this is $($PSVersionTable.PSVersion)"
    }

    $cmakeCmd = Get-Command cmake -ErrorAction SilentlyContinue
    if (-not $cmakeCmd) {
        Write-Fail "CMake not found on PATH"
        Add-SeaStackDiagLog -Path $DiagLogPath -Line "Prerequisite fail: cmake missing"
        return $false
    }

    $verLine = cmake --version 2>&1 | Select-Object -First 1
    Add-SeaStackDiagLog -Path $DiagLogPath -Line "CMake: $verLine"
    if ($verLine -match '(\d+\.\d+\.\d+)') {
        $cv = [version]$Matches[1]
        if ($cv -lt $MinCMake) {
            Write-Fail "CMake $MinCMake or newer required (found $cv)"
            return $false
        }
        Write-OK "CMake $cv"
    } else {
        Write-Warn "Could not parse CMake version from: $verLine"
    }

    if ($ResolvedGeneratorName -match '(?i)^Ninja$') {
        if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
            Write-Fail "Generator Ninja selected but ninja.exe not found on PATH"
            return $false
        }
        if (-not (Get-Command cl -ErrorAction SilentlyContinue)) {
            Write-Warn 'MSVC (cl.exe) not on PATH - use a Visual Studio Developer shell or vcvars for Ninja+MSVC builds'
        }
    }

    $gitCmd = Get-Command git -ErrorAction SilentlyContinue
    if (-not $gitCmd) {
        Write-Warn "Git not on PATH (needed for submodules such as MoorDyn)"
    } else {
        Write-OK "Git available"
    }

    if ($NeedsPython) {
        Write-SeaStackPythonPrerequisiteStatus
    }

    return $true
}

# =============================================================================
# Help
# =============================================================================

if ($Help) {
    Write-Host "`nSEA-Stack Build System v$SCRIPT_VERSION" -ForegroundColor Cyan
    Write-Host "======================================" -ForegroundColor Cyan
    Write-Host ("`nBuilds SEA-Stack. Chrono is discovered by CMake (Chrono_DIR)." + "`n")

    Write-Host "USAGE:" -ForegroundColor Yellow
    Write-Host '  .\scripts\windows\build.ps1 [options]'
    Write-Host ''

    Write-Host "OPTIONS:" -ForegroundColor Yellow
    Write-Host '  -BuildType [type]  Release|Debug|RelWithDebInfo|MinSizeRel (default: Release)'
    Write-Host "  -Clean             Remove build directory before configuring"
    Write-Host "  -Verbose           Stream cmake/build output to console (also in diagnostics log)"
    Write-Host "  -ConfigureOnly     Configure only, skip build"
    Write-Host '  -Reconfigure       Force CMake configure (useful when not using -NoConfigure; no-op vs default today)'
    Write-Host "  -NoConfigure       Skip configure; run cmake --build only (requires existing CMakeCache.txt)"
    Write-Host "  -Test              Run ctest after a successful build"
    Write-Host '  -TestLabel [label] CTest label filter (unit, chrono-free, regression, ...)'
    Write-Host "  -Package           cmake --install + CPack ZIP"
    Write-Host '  -Target [name]     Single build target'
    Write-Host '  -Generator [name] CMake -G (aliases: Ninja, VS2022, VS2019, VS2026) or full generator name'
    Write-Host '  -Arch [arch]       With Visual Studio generators: x64 (default), Win32, ARM64'
    Write-Host "  -Doctor            Print diagnostics and exit"
    Write-Host ""
    Write-Host "  -NoChrono          Chrono-free build (no config file needed)"
    Write-Host "  -NoHydroIO         Disable HDF5-based hydro IO"
    Write-Host "  -MoorDyn           Enable MoorDyn"
    Write-Host "  -External          Enable out-of-process external force modules (Python PTO IPC)"
    Write-Host "                     (implied by -Package so RM3/OSWEC external_pto demos work in the ZIP)"
    Write-Host "  -VSG               Enable VSG visualization"
    Write-Host "  -Vehicle           Enable Chrono::Vehicle support (Chrono must be built with CH_ENABLE_MODULE_VEHICLE)"
    Write-Host "  -Demos             Build C++ demo executables"
    Write-Host "  -NoTests           Skip tests"
    Write-Host "  -NoApps            Skip application executables"
    Write-Host ""
    Write-Host '  -ConfigPath [path] JSON config (default: build-config.json)'
    Write-Host ''

    Write-Host "CONFIG FILE:" -ForegroundColor Yellow
    Write-Host '  Optional "Generator": "Ninja" | "Visual Studio 17 2022" | ...' -ForegroundColor Gray
    Write-Host ""
    exit 0
}

if ($NoConfigure -and $ConfigureOnly) {
    Write-Fail "-NoConfigure and -ConfigureOnly cannot be used together"
    exit 1
}
if ($NoConfigure -and $Reconfigure) {
    Write-Fail "-NoConfigure and -Reconfigure cannot be used together"
    exit 1
}

# =============================================================================
# Repo root
# =============================================================================

$repoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$buildDir = Join-Path $repoRoot "build"
$diagLogPath = Get-SeaStackDiagLogPath -RepoRoot $repoRoot
Initialize-SeaStackDiagLog -Path $diagLogPath -Title "SEA-Stack build diagnostics"

Add-SeaStackDiagLog -Path $diagLogPath -Line "Repo root: $repoRoot"
Add-SeaStackDiagLog -Path $diagLogPath -Line "Flags: Clean=$Clean Verbose=$Verbose ConfigureOnly=$ConfigureOnly Reconfigure=$Reconfigure NoConfigure=$NoConfigure Test=$Test Package=$Package Doctor=$Doctor"
Add-SeaStackDiagLog -Path $diagLogPath -Line "BuildType=$BuildType Target=$Target Generator=$Generator NoChrono=$NoChrono"

Push-Location $repoRoot

try {

$useChrono = -not $NoChrono
$ChronoDir = $null
$PythonRoot = $null
$eigen3Include = $null
$hdf5Dir = $null
$chronoContent = $null
$configGenerator = $null

$resolvedConfig = $ConfigPath
if (-not [System.IO.Path]::IsPathRooted($ConfigPath)) {
    $resolvedConfig = Join-Path $repoRoot $ConfigPath
}

$config = $null
if (Test-Path -LiteralPath $resolvedConfig) {
    try {
        $config = Get-Content -LiteralPath $resolvedConfig -Raw | ConvertFrom-Json
    } catch {
        Write-Warn ('Could not parse ' + $resolvedConfig + ' -- ' + $_.Exception.Message)
        Add-SeaStackDiagLog -Path $diagLogPath -Line "JSON parse error: $($_.Exception.Message)"
    }
}

if ($null -ne $config -and $config.PSObject.Properties.Name -contains 'Generator') {
    $configGenerator = [string]$config.Generator
}

# --- Step count for progress ([N/M] in console; Doctor has its own count) ---
if ($Doctor) {
    $script:SeaStackStepTotal = 4
    if ($useChrono -and $config -and ($config.PSObject.Properties.Name -contains 'ChronoDir') -and $config.ChronoDir) {
        $script:SeaStackStepTotal = 5
    }
} else {
    # Match every Write-SeaStackStep in order (see script body below).
    $n = 1
    if ($useChrono) { $n++ }
    if ($Clean) { $n++ }
    $n++
    $n++
    if (-not $ConfigureOnly) { $n++ }
    $n++
    if ($Test) { $n++ }
    if ($Test -and $TestLabel -eq "benchmark") { $n++ }
    if ($Package) { $n++ }
    if ($Package -and $useChrono -and (-not $NoApps)) { $n++ }
    $script:SeaStackStepTotal = $n
}

Write-Host "`nSEA-Stack Build v$SCRIPT_VERSION" -ForegroundColor Cyan
Write-Host "========================" -ForegroundColor Cyan
Write-Detail ('Diagnostics log: ' + $diagLogPath)

# =============================================================================
# Doctor
# =============================================================================

if ($Doctor) {
    Write-SeaStackStep "Doctor: configuration"
    Write-Detail ('Config file: ' + $resolvedConfig + ' (exists: ' + (Test-Path -LiteralPath $resolvedConfig) + ')')
    if ($config) {
        Write-Detail ('ChronoDir: ' + [string]$config.ChronoDir)
        Write-Detail "Generator (JSON): $configGenerator"
    }
    Write-SeaStackStep "Doctor: generator"
    $gInfo = Resolve-SeaStackGenerator -CliGenerator $Generator -ConfigFileGenerator $configGenerator -BuildDir $buildDir -CleanRequested:$false -DiagLogPath $diagLogPath
    if ($gInfo) {
        Write-OK "Resolved generator: $($gInfo.Name) (multi-config: $($gInfo.MultiConfig))"
    } else {
        Write-Warn "Could not resolve a generator (see messages above)"
    }

    Write-SeaStackStep "Doctor: prerequisites"
    $needPy = $true
    if ($gInfo) {
        $null = Test-SeaStackPrerequisites -MinCMake $SEASTACK_CMAKE_MIN -ResolvedGeneratorName $gInfo.Name -NeedsPython:$needPy -DoctorMode $true -DiagLogPath $diagLogPath
    } else {
        $null = Test-SeaStackPrerequisites -MinCMake $SEASTACK_CMAKE_MIN -ResolvedGeneratorName "" -NeedsPython:$needPy -DoctorMode $true -DiagLogPath $diagLogPath
    }

    Write-SeaStackStep "Doctor: build directory"
    Write-Detail ('Build dir: ' + $buildDir + ' (exists: ' + (Test-Path -LiteralPath $buildDir) + ')')
    $cg = Get-CMakeCacheGenerator -BuildDir $buildDir
    if ($cg) { Write-Detail "Cached CMAKE_GENERATOR: $cg" }

    if ($useChrono -and $config -and ($config.PSObject.Properties.Name -contains 'ChronoDir') -and $config.ChronoDir) {
        Write-SeaStackStep "Doctor: Chrono_DIR"
        $cd = [string]$config.ChronoDir
        Write-Detail ('Path exists: ' + (Test-Path -LiteralPath $cd))
        $pkg = Get-ChronoPackageConfigPathOptional -ChronoDir $cd
        if ($pkg) {
            Write-OK "Optional pre-config package file for heuristics: $(Split-Path $pkg -Leaf)"
        } else {
            Write-Detail "No chrono-config.cmake or ChronoConfig.cmake in directory (CMake may still find the package if layout differs)"
        }
    }

    Write-Host ("`nDoctor complete. See log: " + $diagLogPath) -ForegroundColor Green
    exit 0
}

# =============================================================================
# Load configuration (Chrono / Chrono-free)
# =============================================================================

Write-SeaStackStep "Loading configuration"

if ($useChrono) {
    if (-not $config) {
        Write-Fail ('Config file not found: ' + $resolvedConfig)
        Write-Host "`nCopy build-config.example.json to build-config.json and edit paths." -ForegroundColor Yellow
        exit 1
    }

    $ChronoDir = [string]$config.ChronoDir
    if ($config.PSObject.Properties.Name -contains 'PythonRoot') {
        $PythonRoot = $config.PythonRoot
    }

    if ([string]::IsNullOrWhiteSpace($ChronoDir)) {
        Write-Fail "ChronoDir not set in config file"
        exit 1
    }
    if (-not (Test-Path -LiteralPath $ChronoDir)) {
        Write-Fail ('Chrono_DIR path does not exist: ' + $ChronoDir)
        exit 1
    }
    Write-OK ('Chrono_DIR: ' + $ChronoDir)

    if ([string]::IsNullOrWhiteSpace($PythonRoot)) {
        Write-Warn 'PythonRoot not set in build-config.json: CMake may not find the Python runtime DLL for packaging. Set it to the same conda/env Python that Chrono''s Parsers module was linked against (libpython), or run_seastack may fail to start (0xC0000135) on machines without that DLL on PATH.'
    }

    Write-SeaStackStep "Chrono pre-configure hints (optional)"
    $chronoPkg = Get-ChronoPackageConfigPathOptional -ChronoDir $ChronoDir
    if ($chronoPkg) {
        Write-OK "Found package config file for heuristics: $(Split-Path $chronoPkg -Leaf)"
        $chronoContent = Get-Content -LiteralPath $chronoPkg -Raw
    } else {
        Write-Warn 'No chrono-config.cmake or ChronoConfig.cmake under ChronoDir; skipping pre-configure parse (CMake will still run find_package)'
    }

    if ($chronoContent) {
        $hasVSG = $chronoContent -match 'Chrono_VSG_AVAILABLE\s+ON'
        $hasHDF5 = $chronoContent -match 'CHRONO_HDF5_AVAILABLE\s+(ON|TRUE|1)'
        $useVSG = $VSG -and $hasVSG
        if ($hasVSG) {
            if ($useVSG) { $vsgMsg = 'ON' } else { $vsgMsg = 'available (use -VSG)' }
        } else {
            $vsgMsg = 'not available'
        }
        Write-Detail ('VSG: ' + $vsgMsg)

        if ($chronoContent -match 'Eigen3_DIR\s+"(\S+)"' -and $Matches[1] -notmatch 'NOTFOUND') {
            $eigen3Root = Split-Path (Split-Path $Matches[1])
            $candidate = Join-Path $eigen3Root "include/eigen3"
            if (Test-Path -LiteralPath (Join-Path $candidate "Eigen")) {
                $eigen3Include = $candidate
                Write-OK ('Eigen3: ' + $eigen3Include)
            }
        }
        if (-not $eigen3Include -and $chronoContent -match 'EIGEN3_INCLUDE_DIR\s+"(\S+)"' -and $Matches[1] -notmatch 'NOTFOUND') {
            $candidate = $Matches[1]
            if (Test-Path -LiteralPath (Join-Path $candidate "Eigen")) {
                $eigen3Include = $candidate
                Write-OK ('Eigen3: ' + $eigen3Include + ' (via EIGEN3_INCLUDE_DIR)')
            }
        }

        if ($chronoContent -match 'HDF5_DIR\s+"(\S+)"' -and $Matches[1] -notmatch 'NOTFOUND') {
            $candidate = $Matches[1]
            if (Test-Path -LiteralPath (Join-Path $candidate "hdf5-config.cmake")) {
                $hdf5Dir = $candidate
                Write-OK ('HDF5: ' + $hdf5Dir)
            }
        }
    } else {
        if ($VSG) {
            Write-Warn 'VSG: cannot verify from Chrono package file; enabling -VSG will be validated by CMake configure'
        }
        $useVSG = $VSG
    }
} else {
    Write-Detail "Chrono-free: core libraries only"
    if ($config -and ($config.PSObject.Properties.Name -contains 'PythonRoot')) {
        $PythonRoot = $config.PythonRoot
    }
    $useVSG = $false
}

if (-not $useChrono) {
    $useVSG = $false
}

if (-not $hdf5Dir -and $config -and ($config.PSObject.Properties.Name -contains 'HDF5Dir') -and $config.HDF5Dir -and (Test-Path -LiteralPath $config.HDF5Dir)) {
    $hdf5Dir = [string]$config.HDF5Dir
    Write-OK ('HDF5: ' + $hdf5Dir + ' (from config file)')
}

if (-not $NoHydroIO -and -not $hdf5Dir) {
    Write-Warn "HDF5 not found; HydroIO will be disabled."
    if ($useChrono) {
        Write-Detail "If Chrono was built without HDF5, set HDF5Dir in build-config.json."
    } else {
        Write-Detail "For Chrono-free builds, set HDF5Dir explicitly. See README.md."
    }
    $NoHydroIO = $true
}

# =============================================================================
# MoorDyn submodule
# =============================================================================

if ($MoorDyn -and -not (Test-Path -LiteralPath ".\extern\MoorDyn\CMakeLists.txt")) {
    Write-Fail "MoorDyn submodule not found at extern/MoorDyn"
    Write-Host "   Run: git submodule update --init extern/MoorDyn" -ForegroundColor Yellow
    exit 1
}

# =============================================================================
# Clean
# =============================================================================

if ($Clean) {
    Write-SeaStackStep "Cleaning"
    $removed = @()
    if (Test-Path -LiteralPath $buildDir) {
        Remove-Item -Recurse -Force -LiteralPath $buildDir -ErrorAction Stop
        $removed += "build"
    }
    Get-ChildItem -Path $repoRoot -Directory -Filter "build_*" -ErrorAction SilentlyContinue | ForEach-Object {
        Remove-Item -Recurse -Force -LiteralPath $_.FullName -ErrorAction Stop
        $removed += $_.Name
    }
    if ($removed.Count -gt 0) {
        Write-OK ('Removed: ' + [string]::Join(', ', $removed))
    } else {
        Write-Detail "No build directories found to remove"
    }
}

# =============================================================================
# Generator + prerequisites
# =============================================================================

Write-SeaStackStep "Checking tools and generator"

$genInfo = Resolve-SeaStackGenerator -CliGenerator $Generator -ConfigFileGenerator $configGenerator -BuildDir $buildDir -CleanRequested:$Clean -DiagLogPath $diagLogPath
if (-not $genInfo) {
    exit 1
}
Write-OK "Generator: $($genInfo.Name)"

$needPython = $Test -and ($TestLabel -eq "benchmark")
if (-not (Test-SeaStackPrerequisites -MinCMake $SEASTACK_CMAKE_MIN -ResolvedGeneratorName $genInfo.Name -NeedsPython:$needPython -DoctorMode $false -DiagLogPath $diagLogPath)) {
    exit 1
}

# =============================================================================
# Configure
# =============================================================================

if (-not (Test-Path -LiteralPath $buildDir)) {
    New-Item -ItemType Directory -Path $buildDir -Force | Out-Null
}

$shouldConfigure = (-not $NoConfigure) -or $Reconfigure
if ($ConfigureOnly) {
    $shouldConfigure = $true
}
if ($NoConfigure) {
    $shouldConfigure = $false
    if (-not (Test-Path -LiteralPath (Join-Path $buildDir "CMakeCache.txt"))) {
        Write-Fail ('-NoConfigure requires an existing CMake cache: ' + (Join-Path $buildDir 'CMakeCache.txt'))
        exit 1
    }
}

if ($shouldConfigure) {
    Write-SeaStackStep "Configuring CMake"

    $cmakeArgs = @(
        "-S", $repoRoot,
        "-B", $buildDir,
        "-G", $genInfo.Name,
        "-DCMAKE_BUILD_TYPE=$BuildType",
        "-DSEASTACK_ENABLE_CHRONO=$(if ($useChrono) { 'ON' } else { 'OFF' })",
        "-DSEASTACK_ENABLE_HYDRO_IO=$(if ($NoHydroIO) { 'OFF' } else { 'ON' })",
        "-DSEASTACK_ENABLE_MOORING=$(if ($MoorDyn) { 'ON' } else { 'OFF' })",
        # Release ZIP ships RM3/OSWEC external_pto demos; packaging implies External ON.
        "-DSEASTACK_ENABLE_EXTERNAL=$(if ($External -or $Package) { 'ON' } else { 'OFF' })",
        "-DSEASTACK_ENABLE_VSG=$(if ($useVSG) { 'ON' } else { 'OFF' })",
        "-DSEASTACK_ENABLE_VEHICLE=$(if ($Vehicle -and $useChrono) { 'ON' } else { 'OFF' })",
        "-DSEASTACK_ENABLE_TESTS=$(if ($NoTests) { 'OFF' } else { 'ON' })",
        "-DSEASTACK_ENABLE_DEMOS=$(if ($Demos) { 'ON' } else { 'OFF' })",
        "-DSEASTACK_ENABLE_APPS=$(if ($NoApps) { 'OFF' } else { 'ON' })"
    )

    if ($genInfo.MultiConfig) {
        $cmakeArgs += @("-A", $Arch)
    }

    if ($useChrono) {
        $chronoForward = ($ChronoDir -replace '\\', '/')
        $cmakeArgs += "-DChrono_DIR=$chronoForward"
    }
    if ($eigen3Include) {
        $cmakeArgs += "-DEIGEN3_INCLUDE_DIR=$(($eigen3Include -replace '\\', '/'))"
    }
    if ($hdf5Dir) {
        $cmakeArgs += "-DHDF5_DIR=$(($hdf5Dir -replace '\\', '/'))"
    }
    if ($PythonRoot -and (Test-Path -LiteralPath $PythonRoot)) {
        $cmakeArgs += "-DPython3_ROOT_DIR=$(($PythonRoot -replace '\\', '/'))"
    }
    # ChronoConfig find_package(vsgImGui) needs imgui/implot (e.g. vcpkg installed/x64-windows).
    if ($config -and $config.PSObject.Properties.Name -contains 'CMakePrefixPath') {
        $cpp = [string]$config.CMakePrefixPath
        if (-not [string]::IsNullOrWhiteSpace($cpp)) {
            $cpp = $cpp.Trim()
            if (Test-Path -LiteralPath $cpp) {
                $cmakeArgs += "-DCMAKE_PREFIX_PATH=$(($cpp -replace '\\', '/'))"
                Write-OK ('CMAKE_PREFIX_PATH: ' + $cpp)
            } else {
                Write-Warn "CMakePrefixPath in build-config.json does not exist: $cpp"
            }
        }
    }

    Write-Host ""
    Write-Detail "Build type : $BuildType"
    Write-Detail "Generator  : $($genInfo.Name)"
    Write-Detail "Chrono     : $(if ($useChrono) { 'ON' } else { 'OFF' })"
    Write-Detail "HydroIO    : $(if ($NoHydroIO) { 'OFF' } else { 'ON' })"
    Write-Detail "Mooring    : $(if ($MoorDyn) { 'ON' } else { 'OFF' })"
    Write-Detail "VSG        : $(if ($useVSG) { 'ON' } else { 'OFF' })"
    Write-Detail "Tests      : $(if ($NoTests) { 'OFF' } else { 'ON' })"
    Write-Detail "Demos      : $(if ($Demos) { 'ON' } else { 'OFF' })"
    Write-Detail "Apps       : $(if ($NoApps) { 'OFF' } else { 'ON' })"

    Add-SeaStackDiagLog -Path $diagLogPath -Line "CMake configure arguments:"
    Add-SeaStackDiagLog -Path $diagLogPath -Line ($cmakeArgs -join ' ')

    if (-not (Invoke-SeaStackTool -StepName "CMake configure" -Executable "cmake" -Arguments $cmakeArgs -DiagLogPath $diagLogPath -ShowOutputOnHost:$Verbose)) {
        exit 1
    }

    Write-OK "Configuration complete"

    if ($ConfigureOnly) {
        Write-Host ("`nConfiguration written to: " + $buildDir) -ForegroundColor Green
        Write-Host "Open the generated solution or run without -ConfigureOnly to build." -ForegroundColor Gray
        exit 0
    }
} else {
    Write-SeaStackStep "Skipping configure (-NoConfigure)"
}

# =============================================================================
# Build
# =============================================================================

Write-SeaStackStep "Building"

$t0 = Get-Date

$buildArgs = @("--build", $buildDir, "--config", $BuildType)
if ($Target) {
    $buildArgs += @("--target", $Target)
}

Add-SeaStackDiagLog -Path $diagLogPath -Line "CMake build arguments: $($buildArgs -join ' ')"

if (-not (Invoke-SeaStackTool -StepName "CMake build" -Executable "cmake" -Arguments $buildArgs -DiagLogPath $diagLogPath -ShowOutputOnHost:$Verbose)) {
    exit 1
}

$buildSecs = [math]::Round(((Get-Date) - $t0).TotalSeconds, 1)
Write-OK "Built in $buildSecs seconds"

# =============================================================================
# Copy extra DLLs (heuristic; requires chronoContent from optional parse)
# =============================================================================

$binPath = Join-Path $buildDir "bin\$BuildType"

if ($useChrono -and $chronoContent) {
    if ($chronoContent -match 'HDF5_DIR\s+"(\S+)"') {
        $hdf5Root = Split-Path (Split-Path $Matches[1])
        $hdf5BinDir = Join-Path $hdf5Root "bin"
        if (Test-Path -LiteralPath $hdf5BinDir) {
            $hdf5Dlls = Get-ChildItem -Path $hdf5BinDir -Include "hdf5*.dll", "szip*.dll", "zlib*.dll" -Recurse -ErrorAction SilentlyContinue
            $copiedCount = 0
            foreach ($dll in $hdf5Dlls) {
                $destDll = Join-Path $binPath $dll.Name
                if (-not (Test-Path -LiteralPath $destDll)) {
                    Copy-Item -LiteralPath $dll.FullName -Destination $destDll -Force
                    $copiedCount++
                }
            }
            if ($copiedCount -gt 0) {
                Write-OK "Copied $copiedCount HDF5/compression DLLs"
            }
        }
    }

    $vsgDllDir = $null
    $vsgFromCache = Get-CMakeCacheVariable -BuildDir $buildDir -Name "VSG_DLL_DIR"
    if (-not [string]::IsNullOrWhiteSpace($vsgFromCache) -and (Test-Path -LiteralPath $vsgFromCache)) {
        $vsgDllDir = $vsgFromCache
        Write-Detail "VSG DLL dir (from CMake cache): $vsgDllDir"
    }
    if (-not $vsgDllDir -and $chronoContent -match 'vsg_DIR\s+"(\S+)"') {
        $vsgDllDir = Resolve-SeaStackVsgDllDirFromChronoVsgDir -VsgCmakeDir $Matches[1]
        if ($vsgDllDir) {
            Write-Detail "VSG DLL dir (from Chrono vsg_DIR layout): $vsgDllDir"
        }
    }
    if ($vsgDllDir -and (Test-Path -LiteralPath $vsgDllDir)) {
        $vsgDlls = Get-ChildItem -Path $vsgDllDir -Filter "*.dll" -ErrorAction SilentlyContinue
        $copiedCount = 0
        foreach ($dll in $vsgDlls) {
            $destDll = Join-Path $binPath $dll.Name
            if (-not (Test-Path -LiteralPath $destDll)) {
                Copy-Item -LiteralPath $dll.FullName -Destination $destDll -Force
                $copiedCount++
            }
        }
        if ($copiedCount -gt 0) {
            Write-OK "Copied $copiedCount VSG DLLs"
        }
    }

    if ($MoorDyn) {
        $moordynDll = Join-Path $binPath "moordyn.dll"
        if (Test-Path -LiteralPath $moordynDll) {
            Write-OK "MoorDyn DLL: moordyn.dll"
        } else {
            $found = Get-ChildItem -Path $buildDir -Filter "moordyn.dll" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
            if ($found) {
                Copy-Item -LiteralPath $found.FullName -Destination $moordynDll -Force
                Write-OK "MoorDyn DLL: copied from $($found.Directory.Name)"
            } else {
                Write-Warn "MoorDyn enabled but moordyn.dll not found in build tree"
            }
        }
    }
}

# Python runtime for Chrono_parsers is staged by CMake (install + POST_BUILD on run_seastack); see CMakeLists.txt.

# =============================================================================
# Verify outputs
# =============================================================================

Write-SeaStackStep "Verifying outputs"

$binDir = Join-Path $buildDir "bin\$BuildType"
if (Test-Path -LiteralPath $binDir) {
    $exes = Get-ChildItem -Path $binDir -Filter "*.exe" -ErrorAction SilentlyContinue
    $dlls = Get-ChildItem -Path $binDir -Filter "*.dll" -ErrorAction SilentlyContinue

    Write-Detail ('Output: ' + $binDir)
    Write-Detail "Executables: $($exes.Count)  DLLs: $($dlls.Count)"

    if ($useChrono) {
        $app = Join-Path $binDir "run_seastack.exe"
        if (Test-Path -LiteralPath $app) {
            $size = [math]::Round((Get-Item -LiteralPath $app).Length / 1MB, 1)
            Write-OK ('run_seastack.exe (' + $size + ' MB)')
        }
        Write-SeaStackChronoParsersPythonDllCheck -BinDir $binDir -DiagLogPath $diagLogPath
    }

    # standalone_controller is an SDK-only install target (not shipped in the runtime ZIP).
} else {
    Write-Warn ('Output directory not found: ' + $binDir)
}

# =============================================================================
# Test
# =============================================================================

if ($Test) {
    Write-SeaStackStep "Running tests"

    $ctestArgs = @("-C", $BuildType, "--test-dir", $buildDir, "--output-on-failure")
    if ($TestLabel) {
        $ctestArgs += @("-L", $TestLabel)
        Write-Detail "Filter: label = $TestLabel"
        if ($TestLabel -eq "benchmark") {
            $ctestArgs += @("-j", "1")
            $ctestArgs += @("-E", "^benchmark_report_generation$")
            Write-Detail "Benchmark mode: sequential (-j 1)"
        }
    }

    Add-SeaStackDiagLog -Path $diagLogPath -Line "ctest $($ctestArgs -join ' ')"

    if (-not (Invoke-SeaStackTool -StepName "CTest" -Executable "ctest" -Arguments $ctestArgs -DiagLogPath $diagLogPath -ShowOutputOnHost:$true -SoftFail)) {
        Write-Warn "Some tests failed. Re-run failed tests with:"
        Write-Detail ('ctest -C ' + $BuildType + ' --test-dir ' + $buildDir + ' --rerun-failed --output-on-failure')
    } else {
        Write-OK "All tests passed"
    }

    if ($TestLabel -eq "benchmark") {
        $historyDir = Join-Path $repoRoot "data\benchmarks\history"
        $reportScript = Join-Path $repoRoot "tests\benchmark\utilities\generate_benchmark_report.py"
        $benchReportOut = Join-Path $buildDir "bin\$BuildType\results\tests\benchmark\report"

        Write-Host ""
        Write-SeaStackStep "Benchmark report"
        $benchArgs = @($reportScript, "--build-dir", $buildDir, "--output-dir", $benchReportOut, "--history-dir", $historyDir)
        if (-not (Invoke-SeaStackTool -StepName "Benchmark report (python)" -Executable "python" -Arguments $benchArgs -DiagLogPath $diagLogPath -ShowOutputOnHost:$true -SoftFail)) {
            Write-Warn "Benchmark report script failed (is python on PATH?)"
        }
    }
}

# =============================================================================
# Package
# =============================================================================

$packageZipFullPath = $null
if ($Package) {
    Write-SeaStackStep "Packaging"

    $prefix = Join-Path $buildDir "install"
    $installArgs = @("--install", $buildDir, "--config", $BuildType, "--prefix", $prefix)
    Add-SeaStackDiagLog -Path $diagLogPath -Line "cmake $($installArgs -join ' ')"
    if (-not (Invoke-SeaStackTool -StepName "CMake install" -Executable "cmake" -Arguments $installArgs -DiagLogPath $diagLogPath -ShowOutputOnHost:$Verbose)) {
        exit 1
    }

    $verifyScript = Join-Path $repoRoot "scripts\verify_release_install_prefix.py"
    $verifyArgs = @($verifyScript, $prefix)
    if (-not (Invoke-SeaStackTool -StepName "Verify install prefix" -Executable "python" -Arguments $verifyArgs -DiagLogPath $diagLogPath -ShowOutputOnHost:$Verbose)) {
        Write-Fail "Release install prefix verification failed (see scripts/verify_release_install_prefix.py)"
        exit 1
    }

    if ($useChrono -and (-not $NoApps)) {
        $stagedExe = Join-Path $prefix\bin "run_seastack.exe"
        if (Test-Path -LiteralPath $stagedExe) {
            Write-SeaStackStep "Package smoke: run_seastack --help"
            $binDirForSmoke = Join-Path $prefix "bin"
            $oldSmokePath = $env:PATH
            try {
                # Loader uses the staged bin\ folder first (must contain pythonNN.dll peers installed by CMake).
                $env:PATH = ($binDirForSmoke + ';' + $oldSmokePath)
                Write-SeaStackChronoParsersPythonDllCheck -BinDir $binDirForSmoke -DiagLogPath $diagLogPath
                $p = Start-Process -FilePath $stagedExe -ArgumentList @('--help') -WorkingDirectory $prefix -Wait -PassThru -NoNewWindow
            } finally {
                $env:PATH = $oldSmokePath
            }
            if ($null -eq $p.ExitCode -or $p.ExitCode -ne 0) {
                $ec = $p.ExitCode
                Write-Fail "Staged run_seastack.exe --help failed (exit $ec). Reconfigure with PythonRoot matching the Python used for Chrono Parsers (or -DPython3_ROOT_DIR); CMake stages the pythonNN.dll Chrono_parsers imports. Use -Clean if CMake cached the wrong Python."
                exit 1
            }
            Write-OK "Staged run_seastack.exe starts (--help)"

            if ($useVSG) {
                Write-SeaStackStep "Package smoke: run_seastack --doctor (VSG required)"
                try {
                    $env:PATH = ($binDirForSmoke + ';' + $oldSmokePath)
                    $doctorProc = Start-Process -FilePath $stagedExe -ArgumentList @('--doctor') `
                        -WorkingDirectory $prefix -Wait -PassThru -NoNewWindow
                    if ($null -eq $doctorProc.ExitCode -or $doctorProc.ExitCode -ne 0) {
                        Write-Fail "Staged run_seastack.exe --doctor failed (exit $($doctorProc.ExitCode))."
                        exit 1
                    }
                    $doctorDiag = Get-ChildItem -Path $prefix -Filter "seastack_doctor_*.txt" -File |
                        Sort-Object LastWriteTime -Descending |
                        Select-Object -First 1
                    if (-not $doctorDiag) {
                        Write-Fail "run_seastack --doctor did not write a diagnostics file under $prefix"
                        exit 1
                    }
                    $doctorText = Get-Content -LiteralPath $doctorDiag.FullName -Raw
                    Add-SeaStackDiagLog -Path $diagLogPath -Line "run_seastack --doctor diagnostics ($($doctorDiag.Name)):`n$doctorText"
                    if ($doctorText -notmatch '\[PASS\]\s+Visualization:.*VSG available') {
                        Write-Fail "Release build used -VSG but run_seastack --doctor did not report VSG available (compile-time). Reconfigure with -Clean -VSG."
                        exit 1
                    }
                    Write-OK "Staged run_seastack.exe reports VSG available (--doctor)"
                } finally {
                    $env:PATH = $oldSmokePath
                }
            }
        }
    }

    if ($useVSG) {
        $vsgCache = Get-CMakeCacheVariable -BuildDir $buildDir -Name "SEASTACK_ENABLE_VSG"
        if ($vsgCache -ne "ON") {
            Write-Fail "Release packaging with -VSG requires SEASTACK_ENABLE_VSG=ON in CMake cache (found: '$vsgCache'). Reconfigure with -Clean -VSG."
            exit 1
        }
        Write-Detail "CMake cache: SEASTACK_ENABLE_VSG=ON"
    }

    $stalePackages = @(Get-ChildItem -Path $buildDir -Filter "SEAStack-*.zip" -File -ErrorAction SilentlyContinue)
    foreach ($stalePackage in $stalePackages) {
        Remove-Item -LiteralPath $stalePackage.FullName -Force
    }
    if ($stalePackages.Count -gt 0) {
        Write-Detail "Removed $($stalePackages.Count) stale package ZIP(s)"
    }

    Push-Location $buildDir
    try {
        if (-not (Invoke-SeaStackTool -StepName "CPack" -Executable "cpack" -Arguments @("-C", $BuildType) -DiagLogPath $diagLogPath -ShowOutputOnHost:$Verbose)) {
            exit 1
        }
    } finally {
        Pop-Location
    }

    $pkg = Get-ChildItem -Path $buildDir -Filter "SEAStack-*.zip" -File -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($pkg) {
        Write-OK ($pkg.Name + ' (' + [string][math]::Round($pkg.Length / 1MB, 1) + ' MB)')
        $packageZipFullPath = $pkg.FullName
    }
}

# =============================================================================
# Done
# =============================================================================

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "BUILD SUCCESSFUL" -ForegroundColor Green
Write-Host "========================================`n" -ForegroundColor Green

Write-Host "Main output" -ForegroundColor Yellow
Write-Host ('  ' + $binDir) -ForegroundColor Gray
Write-Host ""

if ($packageZipFullPath) {
    Write-Host "Package" -ForegroundColor Yellow
    Write-Host ('  ' + $packageZipFullPath) -ForegroundColor Gray
    Write-Host ""
}

if (-not $NoTests) {
    Write-Host 'Recommended test commands (from repo root)' -ForegroundColor Yellow
    if ($useChrono) {
        Write-Host '  .\scripts\windows\run_unit_tests.ps1' -ForegroundColor Gray
        Write-Host '  .\scripts\windows\run_chrono_free_tests.ps1' -ForegroundColor Gray
        Write-Host '  .\scripts\windows\run_regression_tests.ps1' -ForegroundColor Gray
        Write-Host '  .\scripts\windows\run_verification_tests.ps1' -ForegroundColor Gray
        Write-Host '  .\scripts\windows\run_comparison_tests.ps1' -ForegroundColor Gray
        Write-Host '  .\scripts\windows\run_benchmarks.ps1' -ForegroundColor Gray
    } else {
        Write-Host '  .\scripts\windows\run_chrono_free_tests.ps1' -ForegroundColor Gray
        Write-Host '  .\scripts\windows\run_unit_tests.ps1' -ForegroundColor Gray
    }
    Write-Host ""

    if (-not $useChrono) {
        Write-Host 'Notes' -ForegroundColor Yellow
        Write-Host '  Chrono-free build: full regression, verification, comparison, and benchmarks need a Chrono build.' -ForegroundColor Gray
        Write-Host ''
    }
}
if ($useChrono) {
    Write-Host 'Report root' -ForegroundColor Yellow
    Write-Host ('  build/bin/' + $BuildType + '/results/tests/') -ForegroundColor Gray
    Write-Host '  Reports live under each suite report folder (regression, verification, comparison, benchmark).' -ForegroundColor Gray
    Write-Host ''

    Write-Host 'Notes' -ForegroundColor Yellow
    Write-Host '  PDFs when pandoc is on PATH; use -NoPdf on test runners to skip. Long regression: .\scripts\windows\run_regression_tests.ps1 -Long' -ForegroundColor Gray
    Write-Host ''
}

Write-Host 'Advanced' -ForegroundColor Yellow
Write-Host '  Bare ctest:' -ForegroundColor Gray
if ($useChrono) {
    Write-Host ('    ctest -C ' + $BuildType + ' -L [label] --test-dir build -j 8 --output-on-failure') -ForegroundColor Gray
} else {
    Write-Host ('    ctest -C ' + $BuildType + ' -L chrono-free --test-dir build -j 8 --output-on-failure') -ForegroundColor Gray
}
Write-Host ""

} finally {
    Pop-Location
}
