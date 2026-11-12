param([string]$IsaacRoot='C:\isaac-sim',[ValidateRange(1,5)][int]$Robots=1,[switch]$Headless,[double]$Seconds=0)
$ErrorActionPreference='Stop'
Set-Location -LiteralPath $PSScriptRoot
$runner=Join-Path $IsaacRoot 'python.bat'
if (-not (Test-Path -LiteralPath $runner)) { throw "Missing Isaac Sim launcher: $runner" }
$env:ROS_DISTRO='humble'
$env:RMW_IMPLEMENTATION='rmw_fastrtps_cpp'
$env:ROS_DOMAIN_ID='42'
$bridgeLib=Join-Path $IsaacRoot 'exts\isaacsim.ros2.bridge\humble\lib'
if (($env:PATH -split ';') -notcontains $bridgeLib) { $env:PATH += ';'+$bridgeLib }
$runArgs=@('isaac_sim/ros2_swarm_scene.py','--robots',"$Robots",'--seconds',"$Seconds")
if ($Headless) { $runArgs+='--headless' }
& $runner @runArgs
exit $LASTEXITCODE
