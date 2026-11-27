param([ValidateSet('robot_v1','robot_v2')][string]$Profile='robot_v1')
$root = Split-Path -Parent $PSScriptRoot
$pio = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe'
if (!(Test-Path -LiteralPath $pio)) { throw 'PlatformIO pio.exe not found' }
Push-Location $root
try { & $pio run -e $Profile; if ($LASTEXITCODE -ne 0) { throw "Build failed: $Profile" } }
finally { Pop-Location }
