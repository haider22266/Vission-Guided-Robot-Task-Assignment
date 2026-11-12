param([string]$Distro='Ubuntu-22.04',[int]$HeadEpochs=30,[int]$FineTuneEpochs=3,[int]$BatchSize=32,[switch]$Setup)
$ErrorActionPreference='Stop'
Set-Location -LiteralPath $PSScriptRoot
$workspace=$PSScriptRoot.Replace('\','/')
if ($workspace -notmatch '^([A-Za-z]):/(.*)$') { throw 'Expected a Windows drive path.' }
$wslRoot='/mnt/'+$Matches[1].ToLower()+'/'+$Matches[2]
if ($Setup) {
    & wsl.exe -d $Distro -- bash "$wslRoot/scripts/setup_cdd_wsl.sh"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
& wsl.exe -d $Distro -- bash "$wslRoot/scripts/run_cdd_training.sh" --head-epochs "$HeadEpochs" --fine-tune-epochs "$FineTuneEpochs" --batch-size "$BatchSize"
exit $LASTEXITCODE
