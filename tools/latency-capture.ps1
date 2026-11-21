# Live capture of the latency test (latency-testing branch, src/autons/latency_test.cpp).
#
# 1. Plug the laptop into the brain (USB), or into the controller to go over the radio.
# 2. Run this script first, then start the test (LEFT in driver control).
# 3. It shows the test's progress and saves logs/latency_NNN_live.csv when the robot says it is done.
#    Then run: python tools/latency_test.py
#
# If it reports missing rows, the same run is on the brain's SD card: hold UP and press DOWN in driver control while
# tools/pull-logs.ps1 is listening, and analyse that copy instead.
#
#   -FromCapture <file>   convert a saved terminal capture instead of listening (for testing the script)
param(
    [string]$OutputDirectory = (Join-Path (Get-Location) "logs"),
    [int]$TimeoutSeconds = 300,
    [string]$FromCapture = ""
)

$ErrorActionPreference = "Stop"

$phaseNames = @{
    "1" = "settle"; "2" = "wiggle slow"; "3" = "wiggle medium"; "4" = "wiggle fast"; "5" = "drive slow";
    "6" = "drive fast"; "7" = "settle"; "8" = "forward burst"; "9" = "brake"; "10" = "reverse burst";
    "11" = "brake"; "12" = "spin right"; "13" = "brake"; "14" = "spin left"; "15" = "brake"; "16" = "return";
    "17" = "done"; "-1" = "ABORTED"
}

function Read-CaptureText {
    param([string]$Path)
    $stream = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read,
                                     [System.IO.FileShare]::ReadWrite)
    try {
        $reader = [System.IO.StreamReader]::new($stream)
        try { return $reader.ReadToEnd() } finally { $reader.Dispose() }
    }
    finally { $stream.Dispose() }
}

# The last LT_BEGIN ... LT_END block in the terminal text.
function Get-Run {
    param([string]$Text)
    $run = $null
    foreach ($raw in ($Text -split "`r?`n")) {
        $line = $raw.Trim()
        if ($line.StartsWith("LT_BEGIN,")) {
            $run = [pscustomobject]@{ Name = $line.Substring(9); Header = $null; Rows = New-Object System.Collections.Generic.List[string]; Sent = -1; Ended = $false }
        }
        elseif ($null -eq $run) { continue }
        elseif ($line.StartsWith("LT_HEADER,")) { $run.Header = $line.Substring(10) }
        elseif ($line.StartsWith("LT,")) { $run.Rows.Add($line.Substring(3)) }
        elseif ($line.StartsWith("LT_END,")) {
            $parts = $line.Split(",")
            if ($parts.Count -ge 3) { $run.Sent = [int]$parts[2] }
            $run.Ended = $true
        }
    }
    return $run
}

function Save-Run {
    param($Run)
    if ($null -eq $Run -or $null -eq $Run.Header) { throw "No latency test was seen in the terminal output." }
    New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
    $base = [System.IO.Path]::GetFileNameWithoutExtension($Run.Name)
    if ($base -eq "live-only") { $base = "latency_" + (Get-Date -Format "yyyyMMdd_HHmmss") }
    $path = Join-Path $OutputDirectory ($base + "_live.csv")

    # Keep only complete rows (a dropped character can split one).
    $columns = $Run.Header.Split(",").Count
    $good = @($Run.Rows | Where-Object { $_.Split(",").Count -eq $columns })
    [System.IO.File]::WriteAllText($path, $Run.Header + "`n" + ($good -join "`n") + "`n")

    $times = @($good | ForEach-Object { [double]($_.Split(",")[0]) })
    $gaps = 0
    $worst = 0
    for ($i = 1; $i -lt $times.Count; $i++) {
        $gap = $times[$i] - $times[$i - 1]
        if ($gap -gt 25) { $gaps++ }
        if ($gap -gt $worst) { $worst = $gap }
    }
    Write-Host ("Saved " + $path + ": " + $good.Count + " rows")
    if (-not $Run.Ended) { Write-Host "  The test's end marker never arrived; the run may be cut short." }
    if ($Run.Sent -gt 0 -and $good.Count -lt $Run.Sent) {
        Write-Host ("  Missing " + ($Run.Sent - $good.Count) + " of " + $Run.Sent + " rows; biggest gap " + $worst + " ms.")
        Write-Host "  If that's a lot, pull the SD copy (tools/pull-logs.ps1, then UP+DOWN) and analyse that instead."
    }
    elseif ($gaps -gt 0) {
        Write-Host ("  " + $gaps + " gaps over 25 ms (biggest " + $worst + " ms).")
    }
}

if ($FromCapture -ne "") {
    Save-Run (Get-Run (Read-CaptureText $FromCapture))
    return
}

# pros.exe ships with the VS Code PROS extension but isn't on PATH outside its terminal (same lookup as deploy.ps1).
if (-not (Get-Command pros.exe -ErrorAction SilentlyContinue)) {
    $inst = Join-Path $env:APPDATA 'Code\User\globalStorage\sigbots.pros\install'
    $env:PATH = (Join-Path $inst 'pros-cli-windows') + ';' + (Join-Path $inst 'vex-vexcom-windows') + ';' + $env:PATH
}

$capturePath = Join-Path $env:TEMP ("crimson-latency-" + [guid]::NewGuid().ToString() + ".txt")
$terminal = Start-Process -FilePath "pros.exe" `
    -ArgumentList @("terminal", "--backend", "solo", "--raw", "--no-banner", "--output", $capturePath) `
    -WindowStyle Hidden -PassThru

Write-Host "Listening. Start the test now (LEFT in driver control). Waiting up to $TimeoutSeconds s..."
$run = $null
$lastPhase = ""
try {
    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    do {
        Start-Sleep -Milliseconds 300
        if (-not (Test-Path $capturePath)) { continue }
        try { $run = Get-Run (Read-CaptureText $capturePath) } catch [System.IO.IOException] { continue }
        if ($null -eq $run -or $null -eq $run.Header -or $run.Rows.Count -eq 0) { continue }
        $phaseColumn = [array]::IndexOf($run.Header.Split(","), "test_phase")
        $fields = $run.Rows[$run.Rows.Count - 1].Split(",")
        if ($phaseColumn -ge 0 -and $fields.Count -gt $phaseColumn -and $fields[$phaseColumn] -ne $lastPhase) {
            $lastPhase = $fields[$phaseColumn]
            $label = $phaseNames[$lastPhase]
            if ($null -eq $label) { $label = "phase " + $lastPhase }
            Write-Host ("  " + $label + "   (" + $run.Rows.Count + " rows so far)")
        }
    } while (-not ($run -and $run.Ended) -and (Get-Date) -lt $deadline)
}
finally {
    if (-not $terminal.HasExited) { Stop-Process -Id $terminal.Id }
}

if (Test-Path $capturePath) {
    $run = Get-Run (Read-CaptureText $capturePath)
    Remove-Item -LiteralPath $capturePath -Force
}
Save-Run $run
