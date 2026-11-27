param([switch]$MotorPowerDisconnected)
if (!$MotorPowerDisconnected) {
    throw 'Disconnect motor/driver battery, then rerun with -MotorPowerDisconnected.'
}
$root = Split-Path -Parent $PSScriptRoot
& (Join-Path $PSScriptRoot 'build.ps1') -Profile robot_v1
if ($LASTEXITCODE -ne 0) { throw 'Build failed; no flash performed.' }
$pkg = Join-Path $env:USERPROFILE '.platformio\packages\tool-openocd'
$bin = Join-Path $root '.pio\build\robot_v1\firmware.bin'
if (!(Test-Path -LiteralPath $bin)) { throw 'Firmware BIN missing.' }
& "$pkg\bin\openocd.exe" -s "$pkg\openocd\scripts" -f interface/stlink.cfg `
    -c 'transport select swd' -f target/stm32f1x.cfg -c 'adapter speed 100' `
    -c 'reset_config none' -c 'cortex_m reset_config sysresetreq' `
    -c 'init' -c 'halt' -c "program $bin verify 0x08000000" `
    -c "verify_image $bin 0x08000000 bin" -c 'reset run' -c 'shutdown'
if ($LASTEXITCODE -ne 0) { throw 'Flash or verify failed.' }
Write-Host 'V1 flashed and verified. Motor remains idle until PB1 START.'
