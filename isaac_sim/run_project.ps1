param([string]$IsaacRoot = $env:ISAAC_SIM_PATH, [switch]$BuildOnly)
$ErrorActionPreference = 'Stop'
if (-not $IsaacRoot) {
    foreach ($candidate in @('C:\isaac-sim','E:\isaacsim')) {
        if (Test-Path -LiteralPath (Join-Path $candidate 'python.bat')) { $IsaacRoot = $candidate; break }
    }
}
if (-not $IsaacRoot -or -not (Test-Path -LiteralPath (Join-Path $IsaacRoot 'python.bat'))) {
    throw 'Set ISAAC_SIM_PATH or pass -IsaacRoot pointing to your Isaac Sim installation.'
}
$sceneScript = Join-Path $PSScriptRoot 'disaster_scene.py'
$sceneArguments = @($sceneScript)
if ($BuildOnly) { $sceneArguments += @('--headless','--build-only') }
& (Join-Path $IsaacRoot 'python.bat') @sceneArguments
if ($LASTEXITCODE -ne 0) { throw "Isaac Sim exited with code $LASTEXITCODE" }
