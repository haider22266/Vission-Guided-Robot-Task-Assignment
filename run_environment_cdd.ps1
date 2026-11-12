param(
    [string]$IsaacRoot='F:\RSPE Project\isaac-sim-standalone-5.1.0-windows-x86_64',
    [ValidateSet('mock','efficientnet')][string]$PerceptionBackend='efficientnet',
    [string]$Model='', [string]$ClassifierPython='',
    [ValidateSet('auto','windows','wsl')][string]$ClassifierBackend='auto',
    [string]$ClassifierDistro='Ubuntu-22.04',
    [switch]$Headless, [double]$Duration=0, [ValidateSet(0,1,2)][int]$TasksPerRobot=0, [switch]$Serial, [switch]$ExitOnComplete
)
$ErrorActionPreference='Stop'
Set-Location -LiteralPath $PSScriptRoot
$runner=Join-Path $IsaacRoot 'python.bat'
if (-not (Test-Path -LiteralPath $runner)) { throw "Isaac launcher missing: $runner" }
if ($Headless -and $Duration -eq 0) { $Duration=3600 }
$runArgs=@('isaac_sim/environment_cdd_project.py','--perception-backend',$PerceptionBackend,'--duration',"$Duration",'--tasks-per-robot',"$TasksPerRobot")
if ($PerceptionBackend -eq 'efficientnet') {
    if (-not $Model) { $Model=Join-Path $PSScriptRoot 'models\cdd\efficientnet_cdd.keras' }
    if (-not (Test-Path -LiteralPath $Model -PathType Leaf)) { throw 'Pass -Model with the trained CDD Keras checkpoint.' }
    $runArgs+=@('--efficientnet-model',$Model)
    if ($ClassifierBackend -eq 'auto') {
        if ($ClassifierPython -and $ClassifierPython -match '^[A-Za-z]:') { $ClassifierBackend='windows' }
        else { $ClassifierBackend='wsl' }
    }
    if ($ClassifierBackend -eq 'wsl') {
        if (-not $ClassifierPython) { $ClassifierPython=Join-Path $PSScriptRoot '.venv-cdd-wsl\bin\python' }
        $runArgs+=@('--classifier-wsl','--classifier-distro',$ClassifierDistro)
    } elseif (-not $ClassifierPython) {
        $ClassifierPython=Join-Path $PSScriptRoot '.venv-cdd\Scripts\python.exe'
    }
}
if ($ClassifierPython) { $runArgs+=@('--classifier-python',$ClassifierPython) }
if ($Headless) { $runArgs+='--headless' }
if ($Serial) { $runArgs+='--serial-cleanup' }
if ($ExitOnComplete) { $runArgs+='--exit-on-complete' }
& $runner @runArgs
exit $LASTEXITCODE
