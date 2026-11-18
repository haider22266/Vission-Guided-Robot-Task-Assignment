param(
    [string]$IsaacRoot='F:\RSPE Project\isaac-sim-standalone-5.1.0-windows-x86_64',
    [ValidateSet(5,10,15,20)][int]$TaskCount=20,
    [ValidateSet('grouped','pile','batched')][string]$Layout='grouped',
    [switch]$Headless,
    [double]$Duration=0,
    [switch]$ExitOnComplete
)
$ErrorActionPreference='Stop'
Set-Location -LiteralPath $PSScriptRoot
$runner=Join-Path $IsaacRoot 'python.bat'
if (-not (Test-Path -LiteralPath $runner)) { throw "Isaac launcher missing: $runner" }
$running=Get-CimInstance Win32_Process -Filter "Name='kit.exe'" | Where-Object {
    $_.CommandLine -match 'environment_(cdd_project|small_tasks)\.py'
}
if ($running) { throw 'Close the existing warehouse simulation before starting the small-task version.' }
if ($Headless -and $Duration -eq 0) { $Duration=600 }
$runArgs=@('isaac_sim/environment_small_tasks.py','--duration',"$Duration",'--small-task-count',"$TaskCount",'--small-layout',$Layout)
if ($Headless) { $runArgs+='--headless' }
if ($ExitOnComplete) { $runArgs+='--exit-on-complete' }
& $runner @runArgs
exit $LASTEXITCODE
