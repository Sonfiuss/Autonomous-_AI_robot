# upload.ps1 - Compile & Upload fan_control.ino -> Arduino Uno
# Usage    : powershell -ExecutionPolicy Bypass -File upload.ps1
# With port: powershell -ExecutionPolicy Bypass -File upload.ps1 -Port COM3

param([string]$Port = "")

$SKETCH = Split-Path -Parent $MyInvocation.MyCommand.Path
$FQBN   = "arduino:avr:uno"

# ── Helper: reload PATH so newly installed tools are found ──────────────────
function Reload-Path {
    $env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") +
                ";" +
                [System.Environment]::GetEnvironmentVariable("Path","User")
}

# 1. Install arduino-cli if missing ─────────────────────────────────────────
if (-not (Get-Command arduino-cli -ErrorAction SilentlyContinue)) {
    Write-Host "[1/4] Installing arduino-cli via Chocolatey..." -ForegroundColor Cyan
    choco install arduino-cli -y --no-progress
    Reload-Path
    if (-not (Get-Command arduino-cli -ErrorAction SilentlyContinue)) {
        Write-Host "ERROR: arduino-cli not found after install. Restart terminal and retry." -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host "[1/4] arduino-cli found OK" -ForegroundColor Green
}

# 2. Install AVR core if missing ─────────────────────────────────────────────
$cores = (arduino-cli core list) 2>&1
if ($cores -notmatch "arduino:avr") {
    Write-Host "[2/4] Installing arduino:avr core..." -ForegroundColor Cyan
    arduino-cli core update-index
    arduino-cli core install arduino:avr
} else {
    Write-Host "[2/4] arduino:avr core OK" -ForegroundColor Green
}

# 3. Auto-detect port ─────────────────────────────────────────────────────────
if ($Port -eq "") {
    Write-Host "[3/4] Detecting Arduino Uno port..." -ForegroundColor Cyan
    $boardList = (arduino-cli board list) 2>&1
    $line = $boardList | Where-Object { $_ -match "COM\d+" -and ($_ -match "uno|Unknown") } | Select-Object -First 1
    if (-not $line) {
        # fallback: just grab first COM port listed
        $line = $boardList | Where-Object { $_ -match "COM\d+" } | Select-Object -First 1
    }
    if ($line -match "(COM\d+)") {
        $Port = $Matches[1]
        Write-Host "      Detected: $Port" -ForegroundColor Green
    } else {
        Write-Host "Available ports:" -ForegroundColor Yellow
        $boardList | ForEach-Object { Write-Host "  $_" }
        $Port = Read-Host "[3/4] Enter COM port (e.g. COM3)"
    }
}

# 4. Compile + Upload ─────────────────────────────────────────────────────────
Write-Host "[4/4] Compiling..." -ForegroundColor Cyan
arduino-cli compile --fqbn $FQBN "$SKETCH"
if ($LASTEXITCODE -ne 0) { Write-Host "COMPILE FAILED" -ForegroundColor Red; exit 1 }

Write-Host "[4/4] Uploading to $Port ..." -ForegroundColor Cyan
arduino-cli upload --fqbn $FQBN --port $Port "$SKETCH"
if ($LASTEXITCODE -eq 0) {
    Write-Host "Upload SUCCESS -> $Port" -ForegroundColor Green
} else {
    Write-Host "Upload FAILED" -ForegroundColor Red; exit 1
}
