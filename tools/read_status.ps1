param([ValidateSet('robot_v1','robot_v2')][string]$Profile='robot_v1',
      [ValidateSet('g_robot_status','g_robot_fault_snapshot')][string]$Symbol='g_robot_status')
$root = Split-Path -Parent $PSScriptRoot
$elf = Join-Path $root ".pio\build\$Profile\firmware.elf"
$tool = Join-Path $env:USERPROFILE '.platformio\packages\toolchain-gccarmnoneeabi\bin'
if (!(Test-Path -LiteralPath $elf)) { throw 'Build the profile first.' }
$entry = & "$tool\arm-none-eabi-nm.exe" -S -n $elf | Select-String " [BbDd] $Symbol$" | Select-Object -First 1
if (!$entry) { throw "Symbol not found in this ELF: $Symbol" }
$parts = $entry.ToString() -split '\s+'
$address = $parts[0]
$bytes = [Convert]::ToInt32($parts[1],16)
$pkg = Join-Path $env:USERPROFILE '.platformio\packages\tool-openocd'
& "$pkg\bin\openocd.exe" -s "$pkg\openocd\scripts" -f interface/stlink.cfg `
    -c 'transport select swd' -f target/stm32f1x.cfg -c 'adapter speed 100' `
    -c 'reset_config none' -c 'cortex_m reset_config sysresetreq' `
    -c 'init' -c "mdb 0x$address $bytes" -c 'shutdown'
if ($LASTEXITCODE -ne 0) { throw 'Read failed.' }
Write-Host "Read $Symbol from symbol address 0x$address in $Profile ELF."
