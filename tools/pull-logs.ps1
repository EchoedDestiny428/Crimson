param(
    [string]$OutputDirectory = (Join-Path (Get-Location) "logs"),
    [int]$TimeoutSeconds = 120
)

$ErrorActionPreference = "Stop"
$capturePath = Join-Path $env:TEMP ("crimson-log-dump-" + [guid]::NewGuid().ToString() + ".txt")

New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
Write-Host "Make sure the updated Crimson program is already running on the Brain."
Write-Host "The Brain must be connected to this computer over USB."
Write-Host "Hold controller UP and press DOWN once to export the SD-card CSV files."
Write-Host "Waiting up to $TimeoutSeconds seconds for the export..."

$terminal = Start-Process -FilePath "pros.exe" `
    -ArgumentList @("terminal", "--backend", "solo", "--raw", "--no-banner", "--output", $capturePath) `
    -WindowStyle Hidden `
    -PassThru

try {
    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    do {
        Start-Sleep -Milliseconds 250
        if (Test-Path $capturePath) {
            $text = [System.IO.File]::ReadAllText($capturePath)
            if ($text.Contains("LOG_DUMP_COMPLETE")) {
                break
            }
        }
    } while ((Get-Date) -lt $deadline)
}
finally {
    if (!$terminal.HasExited) {
        Stop-Process -Id $terminal.Id
    }
}

if (!(Test-Path $capturePath)) {
    throw "The PROS terminal did not produce an output file."
}

$text = [System.IO.File]::ReadAllText($capturePath)
$matches = [regex]::Matches($text, '(?ms)LOG_DUMP_BEGIN,(?<name>[^\r\n]+)\r?\n(?<data>.*?)LOG_DUMP_END,\k<name>')
if ($matches.Count -eq 0) {
    Remove-Item -LiteralPath $capturePath -Force
    throw "No telemetry CSVs were received. Connect the Brain and explicitly trigger the log dump before the timeout."
}

foreach ($match in $matches) {
    $name = [System.IO.Path]::GetFileName($match.Groups["name"].Value)
    [System.IO.File]::WriteAllText(
        (Join-Path $OutputDirectory $name),
        $match.Groups["data"].Value
    )
    Write-Host ("Saved " + $name)
}

Remove-Item -LiteralPath $capturePath -Force
Write-Host ("Saved " + $matches.Count + " telemetry file(s) to " + $OutputDirectory)
