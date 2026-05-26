param(
    [string]$Port,
    [switch]$SkipBuild,
    [switch]$SkipCoreInstall,
    [switch]$Verify,
    [switch]$DryRun,
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
$workspaceRoot = Split-Path -Parent $repoRoot
$configRoot = Join-Path $workspaceRoot '.arduino-cli-ufp2'
$configFile = Join-Path $configRoot 'arduino-cli.yaml'
$buildPath = Join-Path $repoRoot 'build\firmware-UFP2_V160'
$sketchDir = Join-Path $repoRoot 'UFP2_V160'
$buildScript = Join-Path $scriptDir 'build-ufp2.ps1'

function Invoke-ArduinoCli {
    param([string[]]$Arguments)

    & $ArduinoCli @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "arduino-cli failed with exit code ${LASTEXITCODE}: $($Arguments -join ' ')"
    }
}

function Get-AutoPort {
    $ports = Get-CimInstance Win32_SerialPort |
        Where-Object { $_.DeviceID -match '^COM\d+$' } |
        Select-Object DeviceID, Description, Name

    $preferred = $ports | Where-Object {
        $_.Description -match 'Feather|Adafruit|Arduino|SAMD|M0|USB Serial Device' -or
        $_.Name -match 'Feather|Adafruit|Arduino|SAMD|M0|USB Serial Device'
    }

    if (($preferred | Measure-Object).Count -eq 1) {
        return $preferred[0].DeviceID
    }

    if (($ports | Measure-Object).Count -eq 1) {
        return $ports[0].DeviceID
    }

    Write-Host 'Serial ports found:'
    foreach ($portInfo in $ports) {
        Write-Host ("  {0}  {1}" -f $portInfo.DeviceID, $portInfo.Description)
    }
    throw 'Could not choose a unique port. Pass -Port COMx.'
}

if (-not $SkipBuild) {
    $buildArgs = @()
    if ($SkipCoreInstall) {
        $buildArgs += '-SkipCoreInstall'
    }
    if ($ArduinoCli) {
        $buildArgs += @('-ArduinoCli', $ArduinoCli)
    }

    & $buildScript @buildArgs
}

if (-not (Test-Path -LiteralPath $configFile)) {
    throw "Arduino CLI config missing: $configFile. Run tools\build-ufp2.ps1 first."
}

if (-not (Test-Path -LiteralPath $buildPath)) {
    throw "Build output missing: $buildPath. Run without -SkipBuild first."
}

if (-not $Port) {
    $Port = Get-AutoPort
}

$uploadArgs = @(
    '--config-file', $configFile,
    'upload',
    '--fqbn', $fqbn,
    '--port', $Port,
    '--discovery-timeout', '10s',
    '--input-dir', $buildPath
)

if ($Verify) {
    $uploadArgs += '--verify'
}

$uploadArgs += $sketchDir

Write-Host "Flashing UFP2_V160 to $Port"
if ($DryRun) {
    Write-Host "$ArduinoCli $($uploadArgs -join ' ')"
    exit 0
}

Invoke-ArduinoCli $uploadArgs
