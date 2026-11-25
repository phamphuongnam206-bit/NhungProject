# Test double for the read-only backup script. Never opens a debug adapter.
if ($args.Count -ne 6 -or $args[4] -ne '-c') { exit 2 }
$command = [string]$args[5]
if ($command -match '\b(reset|resume|erase|program|flash write|flash erase)\b') { exit 3 }
$match = [regex]::Match($command, 'dump_image \{([^{}]+)\} 0x08000000 0x20000')
if (-not $match.Success) { exit 4 }
$bytes = New-Object byte[] 131072
for ($i = 0; $i -lt $bytes.Length; $i++) { $bytes[$i] = 255 }
if ($env:MOCK_CORRUPT_SECOND -eq '1' -and
        $match.Groups[1].Value.EndsWith('_b.bin')) { $bytes[0] = 0 }
[System.IO.File]::WriteAllBytes($match.Groups[1].Value, $bytes)
Write-Output '0x1ffff7e0: 00000080'
Write-Output '0x1ffff7e8: 12345678 90abcdef 00112233'
exit 0
