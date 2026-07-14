<#
    Copy test-suite PDF reports into GitHub Release asset names.

    Run after the regression / verification / comparison suite scripts have
    produced PDFs (they must exist under build/bin/<BuildType>/results/...).

    Usage:
        .\scripts\windows\stage_release_reports.ps1 -Version v1.0.0-beta.2
        .\scripts\windows\stage_release_reports.ps1 -Version v1.0.0-beta.2 -OutputDir .\release-assets
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
            Suite = "regression"
            Source = Join-Path $resultsRoot "regression\report\regression_test_report.pdf"
            Target = "SEA-Stack-$Version-regression-report.pdf"
        },
        @{
            Suite = "verification"
            Source = Join-Path $resultsRoot "verification\report\verification_report.pdf"
            Target = "SEA-Stack-$Version-verification-report.pdf"
        },
        @{
            Suite = "comparison"
            Source = Join-Path $resultsRoot "comparison\report\comparison_test_report.pdf"
            Target = "SEA-Stack-$Version-comparison-report.pdf"
        }
    )

    New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

    $missing = @()
    foreach ($item in $sources) {
        if (-not (Test-Path $item.Source)) {
            $missing += "$($item.Suite): $($item.Source)"
            continue
        }
        $dest = Join-Path $OutputDir $item.Target
        Copy-Item -Path $item.Source -Destination $dest -Force
        Write-Host "OK  $dest" -ForegroundColor Green
    }

    if ($missing.Count -gt 0) {
        Write-Host "`nMissing PDF(s). Run the suite scripts first:" -ForegroundColor Yellow
        Write-Host "  .\scripts\windows\run_regression_tests.ps1"
        Write-Host "  .\scripts\windows\run_verification_tests.ps1"
        Write-Host "  .\scripts\windows\run_comparison_tests.ps1"
        foreach ($line in $missing) {
            Write-Host "  - $line" -ForegroundColor Red
        }
        exit 1
    }

    Write-Host "`nStaged $($sources.Count) report(s) in $(Resolve-Path $OutputDir)" -ForegroundColor Cyan
}
finally {
    Pop-Location
}
