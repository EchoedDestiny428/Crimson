# Build and upload to the V5 brain (slot from project.pros) with the full PROS log.
# Uses the PROS CLI and toolchain bundled with the VS Code PROS extension.
$inst = Join-Path $env:APPDATA 'Code\User\globalStorage\sigbots.pros\install'
$env:PROS_TOOLCHAIN = Join-Path $inst 'pros-toolchain-windows\usr'
$env:PATH = (Join-Path $inst 'pros-toolchain-windows\usr\bin') + ';' +
            (Join-Path $inst 'pros-cli-windows') + ';' +
            (Join-Path $inst 'vex-vexcom-windows') + ';' + $env:PATH

Set-Location (Split-Path $PSScriptRoot -Parent)
pros lsusb
pros mu @args
