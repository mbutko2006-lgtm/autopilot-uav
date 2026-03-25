param(
    [string]$MissionPlannerExe
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($MissionPlannerExe)) {
    $candidates = @(
        "C:\Program Files (x86)\Mission Planner\MissionPlanner.exe",
        "C:\Program Files\Mission Planner\MissionPlanner.exe"
    )

    $MissionPlannerExe = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
}

if ([string]::IsNullOrWhiteSpace($MissionPlannerExe) -or -not (Test-Path $MissionPlannerExe)) {
    throw "MissionPlanner.exe not found. Install Mission Planner or pass -MissionPlannerExe path."
}

Write-Host "Starting Mission Planner: $MissionPlannerExe"
$mp = Start-Process -FilePath $MissionPlannerExe -PassThru
Write-Host "Mission Planner PID: $($mp.Id)"
