param(
    [switch]$Flash,
    [string]$RunPort = "COM5"
)

$BuildDir = "C:\BugBotBuild"
$SrcDir   = "C:\BugBot"
$IdfDir   = "$env:USERPROFILE\esp\esp-idf"

if (-not (Test-Path $SrcDir)) {
    Write-Host "Creating junction C:\BugBot -> $PSScriptRoot"
    New-Item -ItemType Junction -Path $SrcDir -Target $PSScriptRoot | Out-Null
}

. "$IdfDir\export.ps1"

Set-Location $SrcDir

Write-Host "=== Building ==="
idf.py -B $BuildDir build
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if (-not $Flash) { exit 0 }

Write-Host "=== Triggering download mode via $RunPort ==="
$portsBefore = [System.IO.Ports.SerialPort]::GetPortNames()

try {
    $s = New-Object System.IO.Ports.SerialPort($RunPort, 1200)
    $s.Open()
    Start-Sleep -Milliseconds 200
    $s.Close()
    $s.Dispose()
    Write-Host "Reset signal sent."
} catch {
    Write-Host "Note: $RunPort not available - device may already be in download mode."
}

Write-Host "Waiting for download port..."
$dlPort = $null
$deadline = [Diagnostics.Stopwatch]::StartNew()
while ($deadline.Elapsed.TotalSeconds -lt 10) {
    $portsNow = [System.IO.Ports.SerialPort]::GetPortNames()
    $dlPort = $portsNow | Where-Object { $portsBefore -notcontains $_ } | Select-Object -First 1
    if ($dlPort) { break }
    Start-Sleep -Milliseconds 300
}

if (-not $dlPort) {
    if ([System.IO.Ports.SerialPort]::GetPortNames() -contains "COM15") {
        $dlPort = "COM15"
        Write-Host "Using existing download port COM15."
    } else {
        Write-Host "ERROR: No download port found. Press BOOT + RESET to enter download mode manually."
        exit 1
    }
}

Write-Host "Download port: $dlPort"

Write-Host "=== Flashing to $dlPort ==="
idf.py -B $BuildDir -p $dlPort flash
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "=== Done ==="