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
if ((Test-Path -LiteralPath $first) -or (Test-Path -LiteralPath $second)) { throw 'Backup path already exists' }

foreach ($file in @($first, $second)) {
    $openOcdPath = $file.Replace('\', '/')
    # dump_image reads target Flash only. This script contains no Flash write or erase.
    & $OpenOcd -f interface/stlink.cfg -f target/stm32f1x.cfg `
        -c "init; reset halt; dump_image $openOcdPath 0x08000000 0x20000; resume; shutdown"
    if ($LASTEXITCODE -ne 0) { throw "OpenOCD read failed: $file" }
    if ((Get-Item -LiteralPath $file).Length -ne 131072) { throw "Backup size is not 131072 bytes: $file" }
}
$hashA = (Get-FileHash -LiteralPath $first -Algorithm SHA256).Hash
$hashB = (Get-FileHash -LiteralPath $second -Algorithm SHA256).Hash
if ($hashA -ne $hashB) { throw "Backups differ: $first and $second" }
Write-Output "Two 131072-byte readbacks match; SHA256=$hashA"
Write-Output "Keep both files off the PCB. Readback does not prove writable Flash or retention."
Write-Output $first
Write-Output $second
