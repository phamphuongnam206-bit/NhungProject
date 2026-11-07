
param(
    [string]$Action,
    [string]$SourceFile
)

# TIM PLATFORMIO
$pio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"

if (-not (Test-Path $pio)) {
    $cmd = Get-Command pio -ErrorAction SilentlyContinue

    if (-not $cmd) {
        Write-Host "Khong tim thay PlatformIO!" -ForegroundColor Red
        exit 1
    }

    $pio = $cmd.Source
}

# ==========================================
# 1. NAP BAT KY FILE CPP DANG MO
# ==========================================
if ($Action -eq "upload") {

    if (
        $SourceFile -notmatch '\.cpp$' -or
        [IO.Path]::GetFileName($SourceFile) -ne $SourceFile
    ) {
        Write-Host "Hay mo file .cpp trong thu muc src!" -ForegroundColor Red
        exit 1
    }

    $sourcePath = Join-Path (Join-Path (Get-Location) "src") $SourceFile

    if (-not (Test-Path -LiteralPath $sourcePath -PathType Leaf)) {
        Write-Host "Khong tim thay src/$SourceFile" -ForegroundColor Red
        exit 1
    }

    Write-Host ""
    Write-Host "===== NAP CODE STM32 =====" -ForegroundColor Cyan
    Write-Host "File dang chay: $SourceFile" -ForegroundColor Green

    # Chi bien dich file dang mo.
    # Su dung cau hinh chung tu env:led.
    $env:PLATFORMIO_BUILD_SRC_FILTER = "-<*> +<$SourceFile>"

    try {
        Write-Host "Dang lam sach Build cu..."
        & $pio run -e led -t clean

        if ($LASTEXITCODE -ne 0) {
            Write-Host "CLEAN THAT BAI!" -ForegroundColor Red
            exit 1
        }

        Write-Host "Dang bien dich va nap..."
        & $pio run -e led -t upload
        $result = $LASTEXITCODE

        if ($result -eq 0) {
            Write-Host "NAP THANH CONG: $SourceFile" -ForegroundColor Green
        }
        else {
            Write-Host "NAP THAT BAI!" -ForegroundColor Red
        }

        exit $result
    }
    finally {
        Remove-Item Env:PLATFORMIO_BUILD_SRC_FILTER -ErrorAction SilentlyContinue
    }
}

# ==========================================
# 2. SERIAL MONITOR - TU QUET COM
# ==========================================
if ($Action -eq "monitor") {

    while ($true) {

        Write-Host ""
        Write-Host "===== DANG QUET CONG COM =====" -ForegroundColor Cyan

        $json = (& $pio device list --json-output | Out-String)

        if ($LASTEXITCODE -ne 0) {
            Write-Host "Khong quet duoc COM!" -ForegroundColor Red
            $answer = Read-Host "Enter de thu lai, Q de thoat"
            if ($answer -eq "q") { exit 0 }
            continue
        }

        try {
            $decoded = ConvertFrom-Json -InputObject $json -ErrorAction Stop
        }
        catch {
            Write-Host "Loi doc danh sach COM!" -ForegroundColor Red
            $answer = Read-Host "Enter de thu lai, Q de thoat"
            if ($answer -eq "q") { exit 0 }
            continue
        }

        $ports = @()

        foreach ($device in @($decoded)) {

            $name = [string]$device.port

            if ($name -match '^COM[0-9]+$') {

                $exists = @(
                    $ports | Where-Object { $_.Port -eq $name }
                ).Count -gt 0

                if (-not $exists) {
                    $ports += [PSCustomObject]@{
                        Port = $name
                        Description = [string]$device.description
                    }
                }
            }
        }

        Write-Host ""
        Write-Host "===== DANH SACH COM HIEN CO =====" -ForegroundColor Cyan

        if ($ports.Count -eq 0) {
            Write-Host "Khong tim thay COM!" -ForegroundColor Yellow
            $answer = Read-Host "Enter de quet lai, Q de thoat"
            if ($answer -eq "q") { exit 0 }
            continue
        }

        foreach ($device in $ports) {
            Write-Host "$($device.Port) - $($device.Description)" -ForegroundColor Green
        }

        $available = (
            @($ports | ForEach-Object { $_.Port })
        ) -join ", "

        Write-Host ""
        $inputNumber = Read-Host "Nhap so COM (Q de thoat)"

        if ($inputNumber -eq "q") {
            exit 0
        }

        $number = 0

        if (
            -not [int]::TryParse($inputNumber, [ref]$number) -or
            $number -lt 1
        ) {
            Write-Host "SAI! Hay nhap so COM." -ForegroundColor Red
            Write-Host "Hien co: $available" -ForegroundColor Yellow
            continue
        }

        $selectedPort = "COM$number"

        $selected = @(
            $ports | Where-Object { $_.Port -eq $selectedPort }
        )

        if ($selected.Count -eq 0) {
            Write-Host "$selectedPort khong ton tai!" -ForegroundColor Red
            Write-Host "Hien co: $available" -ForegroundColor Yellow
            continue
        }

        Write-Host ""
        Write-Host "DA CHON $selectedPort" -ForegroundColor Green
        Write-Host "Dang mo Serial - 115200 baud..."
        Write-Host "Ctrl+C de dung."
        Write-Host ""

        & $pio device monitor -p $selectedPort -b 115200

        $result = $LASTEXITCODE

        if ($result -eq 0 -or $result -eq 130) {
            Write-Host "Da dung Serial Monitor." -ForegroundColor Green
            exit 0
        }

        Write-Host ""
        Write-Host "KHONG MO DUOC $selectedPort!" -ForegroundColor Red
        Write-Host "COM co the dang bi chiem dung hoac mat ket noi."

        $answer = Read-Host "Enter de quet lai, Q de thoat"
        if ($answer -eq "q") { exit 0 }
    }
}

Write-Host "Thao tac khong hop le!" -ForegroundColor Red
exit 1
