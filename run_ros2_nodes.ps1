param([ValidateSet('demo','trained')][string]$Mode='demo',[ValidateRange(1,5)][int]$Robots=1,
      [string]$Distro='Ubuntu-22.04',[string]$EfficientNetModel='', [string]$QmixCheckpoint='')
$ErrorActionPreference='Stop'
$workspace=$PSScriptRoot.Replace('\','/')
if ($workspace -notmatch '^([A-Za-z]):/(.*)$') { throw 'Expected a Windows drive path for this workspace.' }
$wslRoot='/mnt/'+$Matches[1].ToLower()+'/'+$Matches[2]
function To-WslPath([string]$PathValue) {
    if ($PathValue.StartsWith('/')) { return $PathValue }
    if (-not (Test-Path -LiteralPath $PathValue -PathType Leaf)) { throw "Model file missing: $PathValue" }
    $resolved=(Resolve-Path -LiteralPath $PathValue).Path.Replace('\','/')
    if ($resolved -notmatch '^([A-Za-z]):/(.*)$') { throw "Unsupported model path: $resolved" }
    return '/mnt/'+$Matches[1].ToLower()+'/'+$Matches[2]
}
$runArgs=@('-d',$Distro,'--','bash',"$wslRoot/scripts/run_ros2_pipeline.sh",$Mode,"$Robots")
if ($Mode -eq 'trained') {
    if (-not $EfficientNetModel -or -not $QmixCheckpoint) {
        throw 'Trained mode requires -EfficientNetModel and -QmixCheckpoint. Demo mode is explicitly mock/baseline.'
    }
    $runArgs+=@((To-WslPath $EfficientNetModel),(To-WslPath $QmixCheckpoint))
}
& wsl.exe @runArgs
exit $LASTEXITCODE
