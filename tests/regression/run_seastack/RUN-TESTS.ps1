Param([string]$Python = "python", [switch]$NoVenv)

# Resolve script directory and install root regardless of current working directory
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
# When installed to tests/, install root is one directory up
$installRoot = Resolve-Path (Join-Path $scriptDir "..")
$tests = Join-Path $installRoot "tests"
$demos = Join-Path $installRoot "demos"
$bin   = Join-Path $installRoot "bin"
$exe   = Join-Path $bin  "run_seastack.exe"

& $Python -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" 2>$null
if ($LASTEXITCODE -ne 0) {
    $ver = & $Python --version 2>&1
    Write-Error "Python 3.10 or newer is required to run the packaged tests. Interpreter: $Python ($ver)"
    exit 1
}

if (!(Test-Path $tests)) {
    Write-Error "Tests not found at $tests"
    exit 1
}
if (!(Test-Path $demos)) {
    Write-Error "Demos not found at $demos"
    exit 1
}
if (!(Test-Path $exe)) {
    Write-Error "Executable not found at $exe"
    exit 2
}

# Capture current PATH and prepend bin for this session
$oldPath = $env:PATH
$env:PATH = "$bin;$env:PATH"

# Set data directory for the installed package
# The executable has hardcoded build-time paths, so we override via environment
$dataDir = Join-Path $installRoot "data"
$env:SEASTACK_DATA_DIR = $dataDir

Push-Location $tests

$reqs = @()
if (Test-Path ".\requirements.txt") {
    $reqs = Get-Content ".\requirements.txt" | Where-Object { $_ -and -not $_.StartsWith('#') }
}

if (-not $NoVenv) {
    # Friendly summary prompt
    $relTests = (Resolve-Path $tests).Path
    Write-Host ""; Write-Host "SEA-Stack Test Environment" -ForegroundColor Cyan
    Write-Host "  Tests folder : $relTests" -ForegroundColor Gray
    Write-Host "  Python       : $Python" -ForegroundColor Gray
    if ($reqs.Count -gt 0) {
        $reqList = ($reqs -join ", ")
        Write-Host "  Will install : $reqList" -ForegroundColor Yellow
        Write-Host "  Source       : PyPI via pip" -ForegroundColor Yellow
    }
    $doSetup = Read-Host "Create local venv at '.venv' and install requirements? [Y]es / [N]o (use current Python)"
    if ($doSetup -match '^(?i)y') {
        if (!(Test-Path ".\.venv")) {
            & $Python -m venv .venv
            if ($LASTEXITCODE) { Pop-Location; $env:PATH = $oldPath; exit $LASTEXITCODE }
        }
        & .\.venv\Scripts\python.exe -m pip install --upgrade pip
        if ($reqs.Count -gt 0) {
            & .\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
        }
        $runner = ".\.venv\Scripts\python.exe"
    } else {
        $runner = $Python
    }
} else {
    $runner = $Python
}

# Pass explicit exe path and demos directory
$env:SS_RUN_EXE = $exe
$env:SS_DEMOS_DIR = $demos
& $runner .\run_tests.py --all --exe "$exe" --demos-dir "$demos"
$code = $LASTEXITCODE

Pop-Location
$env:PATH = $oldPath
exit $code
