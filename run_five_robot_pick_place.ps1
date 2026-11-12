param([string]$IsaacRoot='C:\isaac-sim',[switch]$Headless,[int]$Steps=0,[int]$StopAfter=0)
$ErrorActionPreference='Stop'
Set-Location -LiteralPath $PSScriptRoot
$runner=Join-Path $IsaacRoot 'python.bat'
if (-not (Test-Path -LiteralPath $runner)) { throw "Missing Isaac launcher: $runner" }
if ($Headless -and $Steps -eq 0) { $Steps=6000 }
$runArgs=@('isaac_sim/five_robot_pick_place.py','--steps',"$Steps",'--stop-after',"$StopAfter")
if ($Headless) { $runArgs+='--headless' }
& $runner @runArgs
exit $LASTEXITCODE
