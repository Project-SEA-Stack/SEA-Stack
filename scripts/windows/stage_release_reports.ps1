<#
    Copy test-suite PDF reports into GitHub Release asset names.

    Run after the regression / verification / comparison suite scripts have
    produced PDFs under build/bin/<BuildType>/results/... (requires pandoc + a
    LaTeX engine such as xelatex / pdflatex / lualatex). Exits non-zero if any
    expected PDF is missing.

    Usage:
        .\scripts\windows\stage_release_reports.ps1 -Version v1.0.0-beta.3
        .\scripts\windows\stage_release_reports.ps1 -Version v1.0.0-beta.3 -OutputDir .\release-assets
#>

param(
    [Parameter(Mandatory = $true)]
    [string]$Version,

    [ValidateSet("Release", "Debug", "RelWithDebInfo", "MinSizeRel")]
    [string]$BuildType = "Release",

    [string]$BuildDir = ".\build",
    [string]$OutputDir = ".\release-assets"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $repoRoot
try {
    $resultsRoot = Join-Path $BuildDir "bin\$BuildType\results\tests"

    $sources = @(
        @{
            Suite  = "regression"
            Source = Join-Path $resultsRoot "regression\report\regression_test_report.pdf"
            Target = "SEA-Stack-$Version-regression-report.pdf"
        },
        @{
            Suite  = "verification"
            Source = Join-Path $resultsRoot "verification\report\verification_report.pdf"
            Target = "SEA-Stack-$Version-verification-report.pdf"
        },
        @{
            Suite  = "comparison"
            Source = Join-Path $resultsRoot "comparison\report\comparison_test_report.pdf"
            Target = "SEA-Stack-$Version-comparison-report.pdf"
        }
    )

    New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

    $missing = @()
    $staged = 0
    foreach ($item in $sources) {
        if (Test-Path $item.Source) {
            $dest = Join-Path $OutputDir $item.Target
            Copy-Item -Path $item.Source -Destination $dest -Force
            Write-Host "OK  $dest" -ForegroundColor Green
            $staged++
        } else {
            $missing += "$($item.Suite): expected PDF $($item.Source)"
        }
    }

    if ($missing.Count -gt 0) {
        Write-Host "`nMissing PDF report(s). Run the suite scripts with pandoc + LaTeX on PATH:" -ForegroundColor Yellow
        Write-Host "  .\scripts\windows\run_regression_tests.ps1"
        Write-Host "  .\scripts\windows\run_verification_tests.ps1"
        Write-Host "  .\scripts\windows\run_comparison_tests.ps1"
        Write-Host "Then re-run this script. (Markdown under results/.../report/ is not staged.)"
        foreach ($line in $missing) {
            Write-Host "  - $line" -ForegroundColor Red
        }
        exit 1
    }

    Write-Host "`nStaged $staged PDF report(s) in $(Resolve-Path $OutputDir)" -ForegroundColor Cyan
}
finally {
    Pop-Location
}
