param(
    [string]$TargetLat = "50.443326",
    [string]$TargetLon = "30.448078",
    [double]$TargetAlt = 100,
    [double]$WindSpd = 3,
    [double]$WindDir = 30,
    [double]$WindTurb = 2,
    [double]$WindTurbFreq = 0.2,
    [switch]$SkipPipInstall
)

$ErrorActionPreference = "Stop"
# Do not turn native process stderr output into terminating errors.
if ($null -ne (Get-Variable -Name PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue)) {
    $PSNativeCommandUseErrorActionPreference = $false
}

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Root

$VenvDir = Join-Path $Root ".venv"
$VenvPython = Join-Path $VenvDir "Scripts\python.exe"
$SitlExe = Join-Path $VenvDir "Scripts\dronekit-sitl.exe"

$SitlLog = Join-Path $Root "sitl.log"
$HomePoint = "50.450739,30.461242,0,0"

function Test-PortListening {
    param([int]$Port)

    $pattern = ":$Port"
    $lines = netstat -ano -p tcp | Select-String -Pattern $pattern
    foreach ($line in $lines) {
        if ($line -match "LISTENING") {
            return $true
        }
    }
    return $false
}

function Stop-ProcessSafe {
    param([System.Diagnostics.Process]$Proc)

    if ($null -ne $Proc -and -not $Proc.HasExited) {
        try {
            $Proc.Kill()
            $Proc.WaitForExit(5000) | Out-Null
        }
        catch {
            Write-Warning "Failed to stop process $($Proc.Id): $_"
        }
    }
}

function Stop-PortListeners {
    param([int]$Port)

    $pattern = ":$Port"
    $lines = netstat -ano -p tcp | Select-String -Pattern $pattern
    $pids = @()
    foreach ($line in $lines) {
        if ($line -match "LISTENING") {
            $parts = ($line.ToString() -replace "\s+", " ").Trim().Split(" ")
            $listenerPid = $parts[-1]
            if ($listenerPid -match "^\d+$") {
                $pids += [int]$listenerPid
            }
        }
    }

    $pids = $pids | Select-Object -Unique
    foreach ($listenerPid in $pids) {
        Write-Host "Killing existing process on ${Port}: PID $listenerPid"
        try {
            Stop-Process -Id $listenerPid -Force -ErrorAction Stop
        }
        catch {
            Write-Warning "Could not stop PID $listenerPid on port ${Port}: $_"
        }
    }
}

# Free the SITL port if something is already listening.
Stop-PortListeners -Port 5760
Start-Sleep -Seconds 1

if (-not (Test-Path $VenvPython)) {
    Write-Host "Creating virtual environment..."
    py -3 -m venv .venv
}

if (-not (Test-Path $VenvPython)) {
    throw "Python venv was not created. Ensure Python launcher 'py' is installed."
}

if (-not $SkipPipInstall) {
    Write-Host "Installing dependencies..."
    & $VenvPython -m pip install -r requirements.txt | Out-Null
}

if (-not (Test-Path $SitlExe)) {
    throw "Missing dronekit-sitl.exe in .venv. Run without -SkipPipInstall once."
}

$SitlProc = $null

try {
    Write-Host "Starting SITL (separate window, minimized)..."
    # Open a new window for SITL so apm.exe output stays there.
    $SitlProc = Start-Process -FilePath $SitlExe `
        -ArgumentList @("copter", "--home=$HomePoint", "--wipe") `
        -WorkingDirectory $Root `
        -WindowStyle Minimized `
        -PassThru

    Write-Host "SITL PID : $($SitlProc.Id)  (minimized window)"

    Write-Host -NoNewline "Waiting for SITL on TCP 5760 "
    $ready = $false
    for ($i = 0; $i -lt 240; $i++) {
        if (Test-PortListening -Port 5760) { $ready = $true; break }
        Start-Sleep -Milliseconds 500
        if ($i % 4 -eq 3) { Write-Host -NoNewline "." }
    }
    Write-Host ""

    if (-not $ready) {
        Write-Host "SITL log tail:"
        Get-Content $SitlLog -Tail 30 -ErrorAction SilentlyContinue
        throw "SITL did not become ready on TCP 5760 in time."
    }

    Write-Host "SITL ready. Waiting 3 s for full initialisation..."
    Start-Sleep -Seconds 3

    Write-Host ""
    Write-Host "=== Autopilot mission starting (live output) ==="
    Write-Host ""

    # Run autopilot with unbuffered output so logs are printed immediately.
    # Keep stderr visible but do not let informational stderr lines terminate execution.
    $env:PYTHONUNBUFFERED = "1"
    $prevErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    & $VenvPython -u autopilot_rc_override.py `
        --connect tcp:127.0.0.1:5760 `
        --target-lat $TargetLat `
        --target-lon $TargetLon `
        --target-alt $TargetAlt `
        --wind-spd $WindSpd `
        --wind-dir $WindDir `
        --wind-turb $WindTurb `
        --wind-turb-freq $WindTurbFreq
    $pyExitCode = $LASTEXITCODE
    $ErrorActionPreference = $prevErrorActionPreference

    if ($pyExitCode -ne 0) {
        throw "autopilot_rc_override.py exited with code $pyExitCode"
    }

    Write-Host ""
    Write-Host "=== Mission completed successfully ==="
}
finally {
    Stop-ProcessSafe -Proc $SitlProc
    # Kill any orphan apm.exe children.
    Get-Process -Name apm -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
    Stop-PortListeners -Port 5760
}
