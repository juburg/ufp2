param(
    [switch]$SkipCoreInstall,
    [string]$ArduinoCli = $env:ARDUINO_CLI
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir

if (-not $ArduinoCli) {
    $cmd = Get-Command arduino-cli -ErrorAction SilentlyContinue
    if ($cmd) {
        $ArduinoCli = $cmd.Source
    }
}

if (-not $ArduinoCli -and (Test-Path -LiteralPath 'C:\Program Files\Arduino CLI\arduino-cli.exe')) {
    $ArduinoCli = 'C:\Program Files\Arduino CLI\arduino-cli.exe'
}

if (-not $ArduinoCli -or -not (Test-Path -LiteralPath $ArduinoCli)) {
    throw 'arduino-cli not found. Set ARDUINO_CLI or pass -ArduinoCli.'
}

$fqbn = 'adafruit:samd:adafruit_feather_m0:opt=small,usbstack=arduino,debug=off'
$core = 'adafruit:samd@1.7.17'
$adafruitIndex = 'https://adafruit.github.io/arduino-board-index/package_adafruit_index.json'

$workspaceRoot = Split-Path -Parent $repoRoot
$libraryRoot = Join-Path $repoRoot 'third_party\arduino-libraries'
$libraryNames = @(
    'RadioHead-1.112',
    'U8g2-2.31.2',
    'DimmerZero-1.0.0'
)

$libraryPaths = foreach ($name in $libraryNames) {
    $path = Join-Path $libraryRoot $name
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Required library missing: $path"
    }
    $path
}

$sketchDir = Join-Path $repoRoot 'UFP2_V160'
$sketchFile = Join-Path $sketchDir 'UFP2_V160.ino'
if (-not (Test-Path -LiteralPath $sketchFile)) {
    throw "Required sketch missing: $sketchFile"
}

$configRoot = Join-Path $workspaceRoot '.arduino-cli-ufp2'
$dataDir = Join-Path $configRoot 'data'
$downloadsDir = Join-Path $configRoot 'downloads'
$userDir = Join-Path $configRoot 'user'
$buildPath = Join-Path $repoRoot 'build\firmware-UFP2_V160'
$configFile = Join-Path $configRoot 'arduino-cli.yaml'

foreach ($path in @($configRoot, $dataDir, $downloadsDir, $userDir, $buildPath)) {
    New-Item -ItemType Directory -Path $path -Force | Out-Null
}

function Convert-ToYamlPath([string]$Path) {
    return ($Path -replace '\\', '/')
}

$config = @"
board_manager:
  additional_urls:
    - "$adafruitIndex"
directories:
  data: "$(Convert-ToYamlPath $dataDir)"
  downloads: "$(Convert-ToYamlPath $downloadsDir)"
  user: "$(Convert-ToYamlPath $userDir)"
"@

Set-Content -LiteralPath $configFile -Value $config -Encoding ascii

function Invoke-ArduinoCli {
    param([string[]]$Arguments)

    & $ArduinoCli @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "arduino-cli failed with exit code $LASTEXITCODE"
    }
}

Write-Host "Using Arduino CLI: $ArduinoCli"
Invoke-ArduinoCli @('--config-file', $configFile, 'version')

if (-not $SkipCoreInstall) {
    Invoke-ArduinoCli @('--config-file', $configFile, 'core', 'update-index', '--additional-urls', $adafruitIndex)
    Invoke-ArduinoCli @('--config-file', $configFile, 'core', 'install', $core, '--additional-urls', $adafruitIndex)
}

$compileArgs = @(
    '--config-file', $configFile,
    'compile',
    '--fqbn', $fqbn,
    '--build-path', $buildPath
)

foreach ($libraryPath in $libraryPaths) {
    $compileArgs += @('--library', $libraryPath)
}

$compileArgs += $sketchDir

Write-Host 'Building UFP2 firmware V1.6.0: UFP2_V160'
Invoke-ArduinoCli $compileArgs
