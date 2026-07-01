# flash_data.ps1 - Upload the data/ folder to the robot's LittleFS partition.
# Preserves /config files (WiFi credentials etc.) by reading the current
# partition first, unpacking /config, then building and flashing a new image.
#
# Usage:
#   .\flash_data.ps1              # auto-detect COM port
#   .\flash_data.ps1 -Port COM15  # specify port

param([string]$Port = "")

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# -- Paths -------------------------------------------------------------------
$sketchDir   = Split-Path $PSScriptRoot -Parent    # BugBot_Firmware/ (one level up from tools/)
$dataDir     = Join-Path $sketchDir "Data"
$configDir   = Join-Path $dataDir   "config"
$imageFile   = Join-Path $sketchDir "littlefs_image.bin"
$backupImage = Join-Path $sketchDir "littlefs_backup.bin"
$tmpUnpack   = "C:\Temp\lfs_unpack_tmp"   # short path, no spaces, mklittlefs needs relative dir
$esptool     = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py\5.2.0\esptool.exe"
$mklittlefs  = "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\mklittlefs\4.0.2-db0513a\mklittlefs.exe"

# LittleFS parameters matching ESP32 defaults
$lfsBlock    = 4096
$lfsPage     = 256

# FFat partition - tinyuf2-partitions-8MB-noota.csv (reused for LittleFS)
$partOffset  = "0x450000"
$partSizeKb  = 3776
$partBytes   = $partSizeKb * 1024

# -- Preflight ---------------------------------------------------------------
foreach ($f in @($dataDir, $esptool, $mklittlefs)) {
    if (-not (Test-Path $f)) { Write-Error "Not found: $f"; exit 1 }
}

# -- Auto-detect COM port ----------------------------------------------------
if ($Port -eq "") {
    $ports = Get-CimInstance Win32_PnPEntity |
             Where-Object { $_.Name -match "COM(\d+)" -and
                            ($_.DeviceID -match "VID_303A|VID_10C4|VID_1A86|VID_0403") } |
             ForEach-Object { if ($_.Name -match "(COM\d+)") { $Matches[1] } }
    if (-not $ports) { Write-Error "No ESP32 USB serial port detected. Plug in the robot or use -Port COMx"; exit 1 }
    $Port = @($ports)[0]
    if (@($ports).Count -gt 1) {
        Write-Host "Multiple ports: $($ports -join ', ') - using $Port" -ForegroundColor Yellow
    }
}
Write-Host "Port: $Port" -ForegroundColor Cyan

# -- Step 1: Read current partition to preserve /config ----------------------
Write-Host "`nReading current LittleFS partition from robot..." -ForegroundColor Cyan
& $esptool --chip esp32s3 --port $Port --baud 921600 read-flash $partOffset $partBytes "$backupImage"
if ($LASTEXITCODE -ne 0) { Write-Error "esptool read-flash failed"; exit 1 }

# -- Step 2: Unpack backup and extract /config --------------------------------
Write-Host "`nExtracting /config from backup..." -ForegroundColor Cyan
if (Test-Path $tmpUnpack) { Remove-Item $tmpUnpack -Recurse -Force }
$null = New-Item -ItemType Directory -Force -Path (Split-Path $tmpUnpack)
$extracted = $false

# mklittlefs prepends "./" to its path arg, so it must be run from the parent dir
Push-Location (Split-Path $tmpUnpack)
$null = & $mklittlefs -u (Split-Path $tmpUnpack -Leaf) -b $lfsBlock -p $lfsPage -s $partBytes "$backupImage"
Pop-Location

$configInTmp = Join-Path $tmpUnpack "config"
if ($LASTEXITCODE -eq 0 -and (Test-Path $configInTmp)) {
    if (Test-Path $configDir) { Remove-Item $configDir -Recurse -Force }
    Copy-Item $configInTmp $configDir -Recurse
    $extracted = $true
    Write-Host "Config files preserved in data/config/" -ForegroundColor Green
} else {
    Write-Host "No /config on robot (first flash or empty FS) - creating fresh defaults." -ForegroundColor Yellow
}
if (Test-Path $tmpUnpack) { Remove-Item $tmpUnpack -Recurse -Force }

# -- Step 3: Build new LittleFS image ----------------------------------------
Write-Host "`nBuilding LittleFS image from data/ ..." -ForegroundColor Cyan
& $mklittlefs -c "$dataDir" -b $lfsBlock -p $lfsPage -s $partBytes "$imageFile"
if ($LASTEXITCODE -ne 0) {
    if ($extracted) { Remove-Item $configDir -Recurse -Force }
    Write-Error "mklittlefs create failed"; exit 1
}

# -- Step 4: Flash -----------------------------------------------------------
Write-Host "`nFlashing to $Port at $partOffset ..." -ForegroundColor Cyan
& $esptool --chip esp32s3 --port $Port --baud 921600 write-flash $partOffset "$imageFile"
if ($LASTEXITCODE -ne 0) {
    if ($extracted) { Remove-Item $configDir -Recurse -Force }
    Write-Error "esptool flash failed"; exit 1
}

# -- Cleanup -----------------------------------------------------------------
if ($extracted) { Remove-Item $configDir -Recurse -Force }
Write-Host "`nDone. Open http://bugbot.local/ in your browser." -ForegroundColor Green
