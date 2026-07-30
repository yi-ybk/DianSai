param(
    [ValidateSet("incremental", "full")]
    [string]$BuildType = "full",
    [string]$CcsCli = "",
    [string]$Workspace = "",
    [string]$JLinkExe = "",
    [ValidateRange(1, 50000)]
    [int]$SpeedKHz = 100,
    [switch]$FlashDryRun
)

$ErrorActionPreference = "Stop"

$buildScript = Join-Path $PSScriptRoot "ccs-build.ps1"
$flashScript = Join-Path $PSScriptRoot "jlink-flash.ps1"

if (-not (Test-Path -LiteralPath $buildScript -PathType Leaf)) {
    throw "Build script not found: $buildScript"
}
if (-not (Test-Path -LiteralPath $flashScript -PathType Leaf)) {
    throw "Flash script not found: $flashScript"
}

Write-Host "============================================================"
Write-Host "MSPM0G3507 one-click build and flash"
Write-Host "============================================================"
Write-Host "[1/2] Building firmware..."

& $buildScript `
    -BuildType $BuildType `
    -CcsCli $CcsCli `
    -Workspace $Workspace

Write-Host "[2/2] Flashing firmware..."
if ($FlashDryRun) {
    & $flashScript `
        -JLinkExe $JLinkExe `
        -SpeedKHz $SpeedKHz `
        -DryRun
}
else {
    & $flashScript `
        -JLinkExe $JLinkExe `
        -SpeedKHz $SpeedKHz
}

Write-Host "============================================================"
Write-Host "MSPM0G3507 build and flash completed successfully."
Write-Host "============================================================"
