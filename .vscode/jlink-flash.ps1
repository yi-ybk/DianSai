param(
    [string]$JLinkExe = "",
    [ValidateRange(1, 50000)]
    [int]$SpeedKHz = 100,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$outputFile = Join-Path $projectRoot "Debug\DianSai_MSPM0G3507.out"

if (-not (Test-Path -LiteralPath $outputFile -PathType Leaf)) {
    throw "Build output not found: $outputFile"
}

if ([string]::IsNullOrWhiteSpace($JLinkExe)) {
    $registryKeys = @(
        "HKLM:\SOFTWARE\SEGGER\J-Link",
        "HKLM:\SOFTWARE\WOW6432Node\SEGGER\J-Link",
        "HKCU:\SOFTWARE\SEGGER\J-Link"
    )

    foreach ($registryKey in $registryKeys) {
        if (-not (Test-Path -LiteralPath $registryKey)) {
            continue
        }

        $installPath = (Get-ItemProperty -LiteralPath $registryKey -ErrorAction SilentlyContinue).InstallPath
        if ([string]::IsNullOrWhiteSpace($installPath)) {
            continue
        }

        $candidate = Join-Path $installPath "JLink.exe"
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            $JLinkExe = $candidate
            break
        }
    }
}

if ([string]::IsNullOrWhiteSpace($JLinkExe) -or
    (-not (Test-Path -LiteralPath $JLinkExe -PathType Leaf))) {
    throw "SEGGER J-Link Commander not found. Pass -JLinkExe with its full path."
}

$runningCommander = Get-Process -Name "JLink" -ErrorAction SilentlyContinue
if (($null -ne $runningCommander) -and (-not $DryRun)) {
    throw "JLink.exe is already running. Exit J-Link Commander before flashing."
}

$jlinkOutputPath = $outputFile.Replace("\", "/")
$commands = @(
    "h",
    "loadfile `"$jlinkOutputPath`"",
    "r",
    "g",
    "exit"
)

Write-Host "J-Link : $JLinkExe"
Write-Host "Device : MSPM0G3507"
Write-Host "Link   : SWD, $SpeedKHz kHz"
Write-Host "Image  : $outputFile"

if ($DryRun) {
    Write-Host "Dry run: no hardware operation was performed."
    $commands | ForEach-Object { Write-Host "  $_" }
    exit 0
}

$jlinkOutput = @(
    $commands | & $JLinkExe `
        -device MSPM0G3507 `
        -if SWD `
        -speed $SpeedKHz `
        -NoGui 1 `
        -ExitOnError 1 `
        -autoconnect 1 2>&1
)
$exitCode = $LASTEXITCODE
$jlinkOutput | ForEach-Object { Write-Host $_ }
$outputText = $jlinkOutput | Out-String

if ($exitCode -ne 0) {
    throw "J-Link Commander failed with exit code $exitCode."
}

if (($outputText -notmatch "Cortex-M0 identified") -or
    ($outputText -notmatch "Downloading file") -or
    ($outputText -notmatch "O\.K\.")) {
    throw "J-Link did not confirm a successful target connection and flash download."
}

Write-Host "Flash completed successfully."
