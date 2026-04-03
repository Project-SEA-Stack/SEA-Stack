<#
    Invoke ctest with optional UX tweaks for SEA-Stack PowerShell runners.

    - Default (no -CtestQuiet, no -CtestVerbose): stream output live but drop parallel
      "Start N:" lines so completed test lines (Passed/Failed, timings) remain.
    - -CtestQuiet: passes ctest -Q (Start filtering not used; quiet mode changes output).
    - -CtestVerbose: passes ctest -V; no Start filtering (full debug-style stream).

    Names avoid -Verbose/-Quiet so this advanced function does not clash with common parameters.

    Exit code is the ctest process exit code.
#>
function Invoke-SeaStackCtest {
    param(
        [Parameter(Mandatory)][string[]]$CtestArguments,
        [Parameter(Mandatory)][string]$WorkingDirectory,
        [switch]$CtestQuiet,
        [switch]$CtestVerbose,
        [switch]$SuppressStartLines
    )

    $argList = [System.Collections.Generic.List[string]]::new()
    foreach ($a in $CtestArguments) {
        $argList.Add($a)
    }
    if ($CtestVerbose) {
        $argList.Add('-V')
    } elseif ($CtestQuiet) {
        $argList.Add('-Q')
    }
    $argv = $argList.ToArray()

    $useStartFilter = $SuppressStartLines -and -not $CtestQuiet -and -not $CtestVerbose

    if (-not $useStartFilter) {
        Push-Location -LiteralPath $WorkingDirectory
        try {
            & ctest @argv
            return $LASTEXITCODE
        } finally {
            Pop-Location
        }
    }

    $ctestExe = (Get-Command ctest -ErrorAction Stop).Source
    # Build a single argument string for CreateProcess (quote args that need it).
    $argString = ($argv | ForEach-Object {
            $t = "$_"
            if ($t -match '[\s"]') {
                '"' + ($t -replace '"', '\"') + '"'
            } else {
                $t
            }
        }) -join ' '

    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $ctestExe
    $psi.Arguments = $argString
    $psi.WorkingDirectory = $WorkingDirectory
    $psi.UseShellExecute = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.CreateNoWindow = $true

    $proc = New-Object System.Diagnostics.Process
    $proc.StartInfo = $psi
    [void]$proc.Start()

    # CMake parallel runs: "    Start 12: test_name" — hide these only.
    $startLineRx = '^\s+Start\s+\d+:'

    while ($null -ne ($line = $proc.StandardOutput.ReadLine())) {
        if ($line -notmatch $startLineRx) {
            Write-Host $line
        }
    }

    $errTail = $proc.StandardError.ReadToEnd()
    if ($errTail) {
        Write-Host $errTail
    }

    [void]$proc.WaitForExit()
    return $proc.ExitCode
}
