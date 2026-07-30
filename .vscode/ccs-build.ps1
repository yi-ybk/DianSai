param(
    [ValidateSet("incremental", "full", "clean")]
    [string]$BuildType = "incremental",
    [string]$CcsCli = "C:\TI\ccs2100\ccs\eclipse\ccs-server-cli.bat",
    [string]$Workspace = ""
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($Workspace)) {
    $Workspace = Join-Path (Split-Path -Parent $projectRoot) ".ccs-cli-workspace-compile-check"
}

$projectFile = Join-Path $projectRoot ".project"
$cprojectFile = Join-Path $projectRoot ".cproject"
$configurationName = "Debug"

if (-not (Test-Path -LiteralPath $CcsCli -PathType Leaf)) {
    throw "CCS CLI not found: $CcsCli"
}
if (-not (Test-Path -LiteralPath $projectFile -PathType Leaf)) {
    throw "CCS project file not found: $projectFile"
}
if (-not (Test-Path -LiteralPath $cprojectFile -PathType Leaf)) {
    throw "CCS build configuration not found: $cprojectFile"
}

New-Item -ItemType Directory -Path $Workspace -Force | Out-Null

[xml]$projectXml = Get-Content -LiteralPath $projectFile -Raw -Encoding UTF8
$projectName = [string]$projectXml.projectDescription.name
[xml]$cprojectXml = Get-Content -LiteralPath $cprojectFile -Raw -Encoding UTF8
$configuration = $cprojectXml.SelectSingleNode(
    "//storageModule[@moduleId='cdtBuildSystem']/configuration[@name='$configurationName']")

if ($null -eq $configuration) {
    throw "CCS configuration not found: $configurationName"
}

$artifactName = [string]$configuration.artifactName
$artifactExtension = [string]$configuration.artifactExtension
if ([string]::IsNullOrWhiteSpace($artifactName) -or
    [string]::IsNullOrWhiteSpace($artifactExtension)) {
    throw "CCS output artifact is not configured for $configurationName."
}

Write-Host "Project   : $projectName"
Write-Host "Path      : $projectRoot"
Write-Host "Workspace : $Workspace"
Write-Host "Build     : $BuildType"

function Invoke-CcsCommand {
    param([string[]]$Arguments)

    $previousErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $commandOutput = @(& $CcsCli @Arguments 2>&1)
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }
    $commandOutput | ForEach-Object { Write-Host $_ }

    return [PSCustomObject]@{
        ExitCode = $exitCode
        Text = ($commandOutput | Out-String)
    }
}

$buildArgs = @(
    "-noSplash"
    "-workspace"
    $Workspace
    "-application"
    "com.ti.ccs.apps.buildProject"
    "-ccs.projects"
    $projectName
    "-ccs.configuration"
    $configurationName
    "-ccs.buildType"
    $BuildType
    "-ccs.autoOpen"
    "-ccs.listProblems"
)

$buildResult = Invoke-CcsCommand -Arguments $buildArgs

if ($buildResult.Text -match "Only one CLI command at a time") {
    throw "The CCS CLI workspace is in use: $Workspace"
}

if ($buildResult.Text -match "was not found in the workspace") {
    Write-Host "Importing the main project and its referenced projects..."
    $importArgs = @(
        "-noSplash"
        "-workspace"
        $Workspace
        "-application"
        "com.ti.ccs.apps.projectImport"
        "-ccs.location"
        $projectRoot
        "-ccs.autoImportReferencedProjects"
        "true"
        "-ccs.referencedProjectSearchDirectory"
        (Split-Path -Parent $projectRoot)
    )

    $importResult = Invoke-CcsCommand -Arguments $importArgs
    if ($importResult.ExitCode -ne 0) {
        throw "CCS project import failed with exit code $($importResult.ExitCode)."
    }

    $buildResult = Invoke-CcsCommand -Arguments $buildArgs
}

if ($buildResult.ExitCode -ne 0) {
    throw "CCS build failed with exit code $($buildResult.ExitCode)."
}
if ($buildResult.Text -match "0 out of 0 projects") {
    throw "CCS did not build project: $projectName"
}

if ($BuildType -ne "clean") {
    $outputFile = Join-Path $projectRoot "$configurationName\$artifactName.$artifactExtension"
    if (-not (Test-Path -LiteralPath $outputFile -PathType Leaf)) {
        throw "Build output not found: $outputFile"
    }

    $elfHeader = [System.IO.File]::ReadAllBytes($outputFile)
    if (($elfHeader.Length -lt 4) -or
        ($elfHeader[0] -ne 0x7F) -or
        ($elfHeader[1] -ne 0x45) -or
        ($elfHeader[2] -ne 0x4C) -or
        ($elfHeader[3] -ne 0x46)) {
        throw "Build output is not a valid ELF file: $outputFile"
    }

    Write-Host "Build completed successfully."
    Write-Host "Output    : $outputFile"
}
