$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$tempRoot = (Resolve-Path $env:TEMP).Path
$case = Join-Path $tempRoot ('flash backup mock ' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $case | Out-Null
try {
    $mock = Join-Path $PSScriptRoot 'mock_openocd.ps1'
    $script = Join-Path $root 'tools/backup_flash_128k.ps1'
    $result = & $script -OpenOcd $mock -OutputDirectory $case
    if (($result -join "`n") -notmatch 'Two 131072-byte readbacks match') {
        throw "Backup did not validate two reads: $($result -join '; ')"
    }
    if ((Get-ChildItem -LiteralPath $case -Filter '*.bin').Count -ne 2) { throw 'Expected two backups' }
    if ((Get-ChildItem -LiteralPath $case -Filter '*_openocd.txt').Count -ne 2) { throw 'Expected two metadata logs' }
    foreach ($log in Get-ChildItem -LiteralPath $case -Filter '*_openocd.txt') {
        $content = Get-Content -LiteralPath $log.FullName -Raw
        if ($content -notmatch '0x1ffff7e0: 00000080' -or
                $content -notmatch '0x1ffff7e8:') { throw 'Missing device metadata' }
    }
    $corruptCase = Join-Path $case 'corrupt case'
    New-Item -ItemType Directory -Path $corruptCase | Out-Null
    $env:MOCK_CORRUPT_SECOND = '1'
    try {
        $failed = $false
        try { & $script -OpenOcd $mock -OutputDirectory $corruptCase | Out-Null }
        catch { $failed = $_.Exception.Message -match 'Backups differ' }
        if (-not $failed) { throw 'Mismatched readbacks were accepted' }
    } finally {
        Remove-Item Env:MOCK_CORRUPT_SECOND -ErrorAction SilentlyContinue
    }
    Write-Output 'PASS read-only Flash backup command, paths with spaces, two hashes and metadata logs'
} finally {
    $resolvedCase = (Resolve-Path -LiteralPath $case).Path
    if (-not $resolvedCase.StartsWith($tempRoot + [System.IO.Path]::DirectorySeparatorChar,
            [System.StringComparison]::OrdinalIgnoreCase)) { throw 'Unsafe temporary cleanup path' }
    Remove-Item -LiteralPath $resolvedCase -Recurse -Force
}
