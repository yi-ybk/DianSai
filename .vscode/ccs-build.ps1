param(
    [ValidateSet("incremental", "full", "clean")]
    [string]$BuildType = "incremental",
    [string]$CcsCli = "C:\TI\ccs2100\ccs\eclipse\ccs-server-cli.bat",
    [string]$Workspace = ""
)

$ErrorActionPreference = "Stop"

$ccs = $CcsCli
if ([string]::IsNullOrWhiteSpace($Workspace)) {
    if ([string]::IsNullOrWhiteSpace($env:LOCALAPPDATA)) {
        throw "LOCALAPPDATA is not defined. Pass -Workspace with a writable CCS workspace path."
    }
    $Workspace = Join-Path $env:LOCALAPPDATA "TI\CCS\workspaces\DianSai_MSPM0G3507-cli"
}
$ws = $Workspace

$projectRoot = Split-Path -Parent $PSScriptRoot
$projectFile = Join-Path $projectRoot ".project"
$cprojectFile = Join-Path $projectRoot ".cproject"
$configurationName = "Debug"

if (-not (Test-Path $ccs)) {
    throw "找不到 CCS CLI: $ccs"
}

if (-not (Test-Path $projectFile)) {
    throw "找不到 CCS 工程文件: $projectFile"
}

if (-not (Test-Path $cprojectFile)) {
    throw "找不到 CCS 构建配置文件: $cprojectFile"
}

New-Item -ItemType Directory -Path $ws -Force | Out-Null

[xml]$projectXml = Get-Content $projectFile
$projectName = [string]$projectXml.projectDescription.name
[xml]$cprojectXml = Get-Content $cprojectFile
$configuration = $cprojectXml.SelectSingleNode("//storageModule[@moduleId='cdtBuildSystem']/configuration[@name='$configurationName']")

if ($null -eq $configuration) {
    throw "找不到 CCS 构建配置: $configurationName"
}

$artifactName = [string]$configuration.artifactName
$artifactExtension = [string]$configuration.artifactExtension

if ([string]::IsNullOrWhiteSpace($artifactName) -or [string]::IsNullOrWhiteSpace($artifactExtension)) {
    throw "CCS 构建配置缺少输出文件名或扩展名: $configurationName"
}

Write-Host "Project path : $projectRoot"
Write-Host "Project name : $projectName"
Write-Host "Workspace    : $ws"

function Invoke-CcsCommand {
    param([string[]]$Arguments)

    $commandOutput = @(& $ccs @Arguments 2>&1)
    $exitCode = $LASTEXITCODE
    $commandOutput | ForEach-Object { Write-Host $_ }

    return [PSCustomObject]@{
        ExitCode = $exitCode
        Text     = ($commandOutput | Out-String)
    }
}

$buildArgs = @(
    "-noSplash"
    "-workspace"
    $ws
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

if ($buildResult.Text -match "was not found in the workspace") {
    Write-Host "Importing project into CCS workspace..."

    $importArgs = @(
        "-noSplash"
        "-workspace"
        $ws
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
        exit $importResult.ExitCode
    }

    $buildResult = Invoke-CcsCommand -Arguments $buildArgs
}

if ($buildResult.ExitCode -ne 0) {
    exit $buildResult.ExitCode
}

if ($buildResult.Text -match "0 out of 0 projects") {
    throw "CCS 未构建任何工程: $projectName"
}

if ($BuildType -ne "clean") {
    $outputFile = Join-Path $projectRoot "$configurationName\$artifactName.$artifactExtension"

    if (-not (Test-Path $outputFile)) {
        throw "未生成输出文件: $outputFile"
    }

    $elfHeader = [System.IO.File]::ReadAllBytes($outputFile)
    if (($elfHeader.Length -lt 4) -or
        ($elfHeader[0] -ne 0x7F) -or
        ($elfHeader[1] -ne 0x45) -or
        ($elfHeader[2] -ne 0x4C) -or
        ($elfHeader[3] -ne 0x46)) {
        throw "输出文件不是有效 ELF: $outputFile"
    }

    Write-Host "Output: $outputFile"
}
