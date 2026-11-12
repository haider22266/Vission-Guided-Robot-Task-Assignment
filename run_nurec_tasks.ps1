param(
    [string]$IsaacRoot = 'C:\isaac-sim',
    [switch]$Headless,
    [switch]$ViewProps,
    [int]$Steps = 0
)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$isaacPython = Join-Path $IsaacRoot 'python.bat'
if (-not (Test-Path -LiteralPath $isaacPython)) {
    throw "Isaac Sim launcher missing: $isaacPython. Pass -IsaacRoot with your installation folder."
}
$scenePath = Join-Path $PSScriptRoot 'assets\nurec\nova_carter-galileo\3dgrt\real2sim_galileo.usd'
if (-not (Test-Path -LiteralPath $scenePath)) {
    throw 'NuRec Galileo scene is missing. See isaac_sim/NUREC.md for dataset download instructions.'
}
if ($Headless -and $Steps -eq 0) { $Steps = 1800 }
if ($Steps -lt 0) { throw 'Steps must be nonnegative.' }
$runArgs = @(
    'isaac_sim/nurec_scene.py', '--navigation',
    '--stage', $scenePath, '--fleet-mode', 'independent',
    '--tasks-file', 'isaac_sim/config/tasks_demo.json',
    '--explore-first', '0', '--camera-view', 'overview', '--robot-scale', '0.25',
    '--steps', "$Steps", '--output', 'isaac_sim/scenes/nurec_task_demo.usda'
)
if ($Headless) { $runArgs += @('--headless', '--stop-when-complete') }
if ($ViewProps) { Write-Host 'ViewProps: fixed full-room view includes the fleet, task markers and added props.' }
Write-Host 'Starting tasks inside NuRec Galileo (Neural Volume Rendering).'
Write-Host 'Includes Jackal, Crazyflie, Ingenuity, Forklift C, 12 taped cartons, tools, debris and fallen rack beams.'
Write-Host 'The overview camera stays fixed; this launcher runs the symbolic task demo, not the ROS 2 pipeline.'
Write-Host 'The task panel and console show assigned, in_progress, and completed states.'
& $isaacPython @runArgs
exit $LASTEXITCODE
