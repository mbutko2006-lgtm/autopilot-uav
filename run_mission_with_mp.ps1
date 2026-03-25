param(
    [string]$TargetLat      = "50.443326",
    [string]$TargetLon      = "30.448078",
    [double]$TargetAlt      = 100,
    [double]$WindSpd        = 3,
    [double]$WindDir        = 30,
    [double]$WindTurb       = 2,
    [double]$WindTurbFreq   = 0.2,
    [string]$MissionPlannerExe = "",
    [switch]$SkipMissionPlannerLaunch,
    [switch]$SkipPipInstall
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Root

$VenvDir    = Join-Path $Root ".venv"
$VenvPython = Join-Path $VenvDir "Scripts\python.exe"
$SitlExe    = Join-Path $VenvDir "Scripts\dronekit-sitl.exe"
$BridgePy   = Join-Path $Root "mavlink_telemetry_bridge.py"
$SitlLog    = Join-Path $Root "sitl.log"
$BridgeLog  = Join-Path $Root "bridge.log"
$BridgeErr  = Join-Path $Root "bridge.err.log"
$HomePoint  = "50.450739,30.461242,0,0"

# Locate Mission Planner executable unless user wants to run it manually
if (-not $SkipMissionPlannerLaunch) {
    if ([string]::IsNullOrWhiteSpace($MissionPlannerExe)) {
        $candidates = @(
            "C:\Program Files (x86)\Mission Planner\MissionPlanner.exe",
            "C:\Program Files\Mission Planner\MissionPlanner.exe"
        )
        $MissionPlannerExe = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
    }
    if ([string]::IsNullOrWhiteSpace($MissionPlannerExe) -or -not (Test-Path $MissionPlannerExe)) {
        throw "MissionPlanner.exe not found. Install the MSI or pass -MissionPlannerExe path."
    }
}

function Test-PortListening {
    param([int]$Port)
    $lines = netstat -ano -p tcp | Select-String -Pattern ":$Port"
    foreach ($line in $lines) {
        if ($line -match "LISTENING") { return $true }
    }
    return $false
}

function Stop-ProcessSafe {
    param([System.Diagnostics.Process]$Proc)
    if ($null -ne $Proc -and -not $Proc.HasExited) {
        try { cmd /c "taskkill /PID $($Proc.Id) /F" | Out-Null } catch {}
    }
}

function Stop-ProcessIdSafe {
    param([int]$ProcessId)
    if ($ProcessId -gt 0) {
        try { cmd /c "taskkill /PID $ProcessId /F" | Out-Null } catch {}
    }
}

function Stop-PortListeners {
    param([int]$Port)
    $lines = netstat -ano -p tcp | Select-String -Pattern ":$Port"
    $pids = @()
    foreach ($line in $lines) {
        if ($line -match "LISTENING") {
            $parts = ($line.ToString() -replace "\s+", " ").Trim().Split(" ")
            $listenerPid = $parts[-1]
            if ($listenerPid -match "^\d+$") { $pids += [int]$listenerPid }
        }
    }
    foreach ($listenerPid in ($pids | Select-Object -Unique)) {
        Stop-ProcessIdSafe -ProcessId $listenerPid
    }
}

function Stop-ManagedPythonProcesses {
    $patterns = @(
        'autopilot_rc_override.py',
        'mavlink_telemetry_bridge.py',
        'dronekit-sitl.exe'
    )

    $processes = Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'python.exe' }
    foreach ($proc in $processes) {
        $cmd = [string]$proc.CommandLine
        foreach ($pattern in $patterns) {
            if ($cmd -like "*$pattern*") {
                Stop-ProcessIdSafe -ProcessId $proc.ProcessId
                break
            }
        }
    }
}

function Wait-MavlinkHeartbeat {
    param(
        [string]$PythonExe,
        [string]$Connection = "tcp:127.0.0.1:5762",
        [int]$TimeoutSec = 45
    )

    $code = @"
import time
from pymavlink import mavutil

deadline = time.time() + $TimeoutSec
ok = False

while time.time() < deadline:
    conn = None
    try:
        conn = mavutil.mavlink_connection('$Connection', autoreconnect=True, robust_parsing=True)
        hb = conn.wait_heartbeat(timeout=3)
        if hb is not None:
            ok = True
            print('HEARTBEAT_OK')
            break
    except Exception:
        time.sleep(0.5)
    finally:
        try:
            if conn is not None:
                conn.close()
        except Exception:
            pass

if not ok:
    raise SystemExit(2)
"@

    & $PythonExe -c $code
    return ($LASTEXITCODE -eq 0)
}

# Clean up any leftover SITL from a previous run
Stop-ManagedPythonProcesses
Stop-PortListeners -Port 5760
Start-Sleep -Seconds 1

if (-not (Test-Path $VenvPython)) {
    Write-Host "Creating virtual environment..."
    py -3 -m venv .venv
}
if (-not $SkipPipInstall) {
    Write-Host "Installing dependencies..."
    & $VenvPython -m pip install -r requirements.txt | Out-Null
}
if (-not (Test-Path $SitlExe)) {
    throw "Missing dronekit-sitl.exe. Run without -SkipPipInstall once."
}
if (-not (Test-Path $BridgePy)) {
    throw "Missing mavlink_telemetry_bridge.py"
}

$SitlProc = $null
$MpProc   = $null
$BridgeProc = $null

try {
    # ── 1. SITL in its own minimized window ─────────────────────────────────
    Write-Host "Starting SITL (minimized window)..."
    $SitlProc = Start-Process -FilePath $SitlExe `
        -ArgumentList @("copter", "--home=$HomePoint", "--wipe") `
        -WorkingDirectory $Root `
        -WindowStyle Minimized `
        -PassThru
    Write-Host "SITL PID : $($SitlProc.Id)"

    Write-Host -NoNewline "Waiting for SITL on TCP 5760 "
    $ready = $false
    for ($i = 0; $i -lt 240; $i++) {
        if (Test-PortListening -Port 5760) { $ready = $true; break }
        Start-Sleep -Milliseconds 500
        if ($i % 4 -eq 3) { Write-Host -NoNewline "." }
    }
    Write-Host ""
    if (-not $ready) { throw "SITL did not become ready on TCP 5760 in time." }

    Write-Host "SITL ready. Waiting 3 s for full initialisation..."
    Start-Sleep -Seconds 3

    # 2. Start telemetry bridge for Mission Planner over UDP
    Write-Host "Starting telemetry bridge to UDP 14550..."
    $BridgeProc = Start-Process -FilePath $VenvPython `
        -ArgumentList @($BridgePy, "--master", "tcp:127.0.0.1:5760", "--udp-host", "127.0.0.1", "--udp-port", "14550") `
        -WorkingDirectory $Root `
        -RedirectStandardOutput $BridgeLog `
        -RedirectStandardError $BridgeErr `
        -WindowStyle Hidden `
        -PassThru
    Start-Sleep -Seconds 2

    # 3. Launch Mission Planner unless user runs it manually
    if (-not $SkipMissionPlannerLaunch) {
        Write-Host "Launching Mission Planner..."
        $MpProc = Start-Process -FilePath $MissionPlannerExe -PassThru
        Write-Host "Mission Planner PID: $($MpProc.Id)"
    } else {
        Write-Host "Mission Planner auto-launch skipped (-SkipMissionPlannerLaunch)."
    }
    Write-Host ""
    Write-Host "Connect Mission Planner manually:"
    Write-Host "1) Select UDP in the top-right connection dropdown"
    Write-Host "2) Click CONNECT"
    Write-Host "3) Port: 14550"
    Write-Host ""
    Write-Host "Waiting up to 45 s for MAVLink heartbeat on TCP 5762..."
    if (-not (Wait-MavlinkHeartbeat -PythonExe $VenvPython -Connection "tcp:127.0.0.1:5762" -TimeoutSec 45)) {
        throw "No MAVLink heartbeat detected on tcp:127.0.0.1:5762"
    }
    Write-Host "Heartbeat detected. Autopilot starts in 5 seconds..."
    Start-Sleep -Seconds 5

    # ── 4. Autopilot — runs inline so output is live in this terminal ─────────
    Write-Host ""
    Write-Host "=== Autopilot mission starting (live output) ==="
    Write-Host ""

    $env:PYTHONUNBUFFERED = "1"
    & $VenvPython -u autopilot_rc_override.py `
        --connect tcp:127.0.0.1:5762 `
        --target-lat $TargetLat `
        --target-lon $TargetLon `
        --target-alt $TargetAlt `
        --wind-spd $WindSpd `
        --wind-dir $WindDir `
        --wind-turb $WindTurb `
        --wind-turb-freq $WindTurbFreq

    if ($LASTEXITCODE -ne 0) {
        throw "autopilot_rc_override.py exited with code $LASTEXITCODE"
    }

    Write-Host ""
    Write-Host "=== Mission completed successfully ==="
}
finally {
    Stop-ManagedPythonProcesses
    Stop-ProcessSafe -Proc $BridgeProc
    Stop-ProcessSafe -Proc $SitlProc
    Get-Process -Name apm -ErrorAction SilentlyContinue | ForEach-Object { Stop-ProcessIdSafe -ProcessId $_.Id }
    Stop-PortListeners -Port 5760
}
