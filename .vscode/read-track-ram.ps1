param(
    [string]$JLinkExe = "",
    [ValidateRange(1, 50000)]
    [int]$SpeedKHz = 100,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

function Find-JLinkExe {
    $registryKeys = @(
        "HKLM:\SOFTWARE\SEGGER\J-Link",
        "HKLM:\SOFTWARE\WOW6432Node\SEGGER\J-Link",
        "HKCU:\SOFTWARE\SEGGER\J-Link"
    )

    foreach ($registryKey in $registryKeys) {
        if (-not (Test-Path -LiteralPath $registryKey)) {
            continue
        }
        $installPath = (Get-ItemProperty -LiteralPath $registryKey `
            -ErrorAction SilentlyContinue).InstallPath
        if ([string]::IsNullOrWhiteSpace($installPath)) {
            continue
        }
        $candidate = Join-Path $installPath "JLink.exe"
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
    }
    return ""
}

function Get-MapSymbolPlacement {
    param(
        [string]$MapText,
        [string]$SymbolName
    )

    $escapedName = [regex]::Escape($SymbolName)
    $pattern = "(?m)^\s*([0-9A-Fa-f]{8})\s+([0-9A-Fa-f]{8})\s+" +
               ".*\(\.(?:data|bss)\." + $escapedName + "\)\s*$"
    $match = [regex]::Match($MapText, $pattern)
    if (-not $match.Success) {
        throw "Symbol placement not found in linker map: $SymbolName"
    }

    return [PSCustomObject]@{
        Address = [Convert]::ToUInt32($match.Groups[1].Value, 16)
        Size = [Convert]::ToUInt32($match.Groups[2].Value, 16)
    }
}

$projectRoot = Split-Path -Parent $PSScriptRoot
$debugDirectory = Join-Path $projectRoot "Debug"
$mapFile = Join-Path $debugDirectory "DianSai_MSPM0G3507.map"
$decoder = Join-Path $PSScriptRoot "decode_track_trace.py"

if (-not (Test-Path -LiteralPath $mapFile -PathType Leaf)) {
    throw "Linker map not found. Build the Debug configuration first: $mapFile"
}
if (-not (Test-Path -LiteralPath $decoder -PathType Leaf)) {
    throw "Track trace decoder not found: $decoder"
}
if ([string]::IsNullOrWhiteSpace($JLinkExe)) {
    $JLinkExe = Find-JLinkExe
}
if ([string]::IsNullOrWhiteSpace($JLinkExe) -or
    (-not (Test-Path -LiteralPath $JLinkExe -PathType Leaf))) {
    throw "SEGGER J-Link Commander not found. Pass -JLinkExe with its full path."
}

$runningCommander = Get-Process -Name "JLink" -ErrorAction SilentlyContinue
if (($null -ne $runningCommander) -and (-not $DryRun)) {
    throw "JLink.exe is already running. Exit J-Link Commander before reading RAM."
}

$mapText = Get-Content -LiteralPath $mapFile -Raw -Encoding UTF8
$tracePlacement = Get-MapSymbolPlacement `
    -MapText $mapText -SymbolName "track_trace_store"
$summaryPlacement = Get-MapSymbolPlacement `
    -MapText $mapText -SymbolName "track_debug_state"

$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
$outputPrefix = Join-Path $debugDirectory "track-trace-$timestamp"
$traceFile = "$outputPrefix.bin"
$summaryFile = "$outputPrefix-summary.bin"
$traceJLinkPath = $traceFile.Replace("\", "/")
$summaryJLinkPath = $summaryFile.Replace("\", "/")
$traceAddress = "0x{0:X8}" -f $tracePlacement.Address
$traceSize = "0x{0:X}" -f $tracePlacement.Size
$summaryAddress = "0x{0:X8}" -f $summaryPlacement.Address
$summarySize = "0x{0:X}" -f $summaryPlacement.Size
$commands = @(
    "h",
    "savebin `"$traceJLinkPath`", $traceAddress, $traceSize",
    "savebin `"$summaryJLinkPath`", $summaryAddress, $summarySize",
    "g",
    "exit"
)

Write-Host "J-Link : $JLinkExe"
Write-Host "Device : MSPM0G3507"
Write-Host "Link   : SWD, $SpeedKHz kHz"
Write-Host "Trace  : $traceAddress, $($tracePlacement.Size) bytes"
Write-Host "Summary: $summaryAddress, $($summaryPlacement.Size) bytes"

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
if ($outputText -notmatch "Cortex-M0 identified") {
    throw "J-Link did not confirm a successful MSPM0 target connection."
}
if ((-not (Test-Path -LiteralPath $traceFile -PathType Leaf)) -or
    ((Get-Item -LiteralPath $traceFile).Length -ne $tracePlacement.Size)) {
    throw "Track trace RAM dump is missing or has an unexpected size: $traceFile"
}
if ((-not (Test-Path -LiteralPath $summaryFile -PathType Leaf)) -or
    ((Get-Item -LiteralPath $summaryFile).Length -ne $summaryPlacement.Size)) {
    throw "Track summary RAM dump is missing or has an unexpected size: $summaryFile"
}

$python = Get-Command "python" -ErrorAction SilentlyContinue
if ($null -eq $python) {
    throw "Python was not found; RAM was read, but automatic decoding cannot run."
}

& $python.Source $decoder $traceFile $summaryFile `
    --output-prefix $outputPrefix
if ($LASTEXITCODE -ne 0) {
    throw "Track trace decoder failed with exit code $LASTEXITCODE."
}

Write-Host "Track RAM read and decode completed successfully."
Write-Host "CSV    : $outputPrefix.csv"
Write-Host "Report : $outputPrefix.json"
