param(
    [Parameter(Mandatory = $true)][string]$OpenOcd,
    [Parameter(Mandatory = $true)][string]$OutputDirectory
)

$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $OpenOcd -PathType Leaf)) { throw 'OpenOCD executable not found' }
if (-not (Test-Path -LiteralPath $OutputDirectory -PathType Container)) { throw 'Output directory must already exist' }
$resolved = (Resolve-Path -LiteralPath $OutputDirectory).Path
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$first = Join-Path $resolved "stm32_128k_${stamp}_a.bin"
$second = Join-Path $resolved "stm32_128k_${stamp}_b.bin"
$logs = @(
    (Join-Path $resolved "stm32_128k_${stamp}_a_openocd.txt"),
    (Join-Path $resolved "stm32_128k_${stamp}_b_openocd.txt")
)
if (@($first, $second) + $logs | Where-Object { Test-Path -LiteralPath $_ }) {
    throw 'Backup path already exists'
}

for ($read = 0; $read -lt 2; $read++) {
    $file = @($first, $second)[$read]
    $openOcdPath = $file.Replace('\', '/')
    if ($openOcdPath -match '[{}\r\n]') { throw 'Output path contains unsupported Tcl characters' }
    # dump_image reads target Flash only. This script contains no Flash write or erase.
    & $OpenOcd -f interface/stlink.cfg -f target/stm32f1x.cfg `
        -c "init; halt; mdw 0x1FFFF7E0 1; mdw 0x1FFFF7E8 3; dump_image {$openOcdPath} 0x08000000 0x20000; shutdown" `
        *> $logs[$read]
    if ($LASTEXITCODE -ne 0) { throw "OpenOCD read failed: $file (see $($logs[$read]))" }
    if ((Get-Item -LiteralPath $file).Length -ne 131072) { throw "Backup size is not 131072 bytes: $file" }
}
$hashA = (Get-FileHash -LiteralPath $first -Algorithm SHA256).Hash
$hashB = (Get-FileHash -LiteralPath $second -Algorithm SHA256).Hash
if ($hashA -ne $hashB) { throw "Backups differ: $first and $second" }
Write-Output "Two 131072-byte readbacks match; SHA256=$hashA"
Write-Output "Keep both files off the PCB. Readback does not prove writable Flash or retention."
Write-Output $first
Write-Output $second
Write-Output $logs[0]
Write-Output $logs[1]
