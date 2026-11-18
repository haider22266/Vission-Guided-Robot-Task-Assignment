param(
    [string]$IsaacRoot='F:\RSPE Project\isaac-sim-standalone-5.1.0-windows-x86_64',
    [ValidateSet(5,10,15,20)][int]$TaskCount=20,
    [ValidateSet('grouped','pile','batched')][string]$Layout='grouped',
    [switch]$Headless,
    [double]$Duration=0,
    [switch]$Restart,
    [switch]$ExitOnComplete
)
$ErrorActionPreference='Stop'
Set-Location -LiteralPath $PSScriptRoot
$runner=Join-Path $IsaacRoot 'python.bat'
if (-not (Test-Path -LiteralPath $runner)) { throw "Isaac launcher missing: $runner" }
$running=Get-CimInstance Win32_Process -Filter "Name='kit.exe'" | Where-Object {
    $_.CommandLine -match 'environment_(cdd_project|small_tasks)\.py'
}
if ($running -and -not $Restart) {
    $runningIds=($running.ProcessId -join ', ')
    throw "Warehouse simulation already running (PID $runningIds). Close its window, or add -Restart to close it gracefully and start a fresh run."
}
if ($running -and $Restart) {
    foreach ($instance in $running) {
        $existingProcess=Get-Process -Id $instance.ProcessId -ErrorAction SilentlyContinue
        if (-not $existingProcess) { continue }
        Write-Host "Closing warehouse simulation PID $($instance.ProcessId) before restarting..."
        if (-not $existingProcess.CloseMainWindow()) {
            throw "Warehouse PID $($instance.ProcessId) has no closable window. Stop that specific run in its terminal, then retry."
        }
        Wait-Process -Id $instance.ProcessId -Timeout 30 -ErrorAction SilentlyContinue
        if (Get-Process -Id $instance.ProcessId -ErrorAction SilentlyContinue) {
            throw "Warehouse PID $($instance.ProcessId) is still shutting down. Wait for it to exit, then retry."
        }
    }
}
if ($Headless -and $Duration -eq 0) { $Duration=600 }
$runArgs=@('isaac_sim/environment_small_tasks.py','--duration',"$Duration",'--small-task-count',"$TaskCount",'--small-layout',$Layout)
if ($Headless) { $runArgs+='--headless' }
if ($ExitOnComplete) { $runArgs+='--exit-on-complete' }
& $runner @runArgs
exit $LASTEXITCODE
