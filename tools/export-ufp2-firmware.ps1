param(
    [switch]$SkipBuild,
    [switch]$SkipCoreInstall,
    [string]$ArduinoCli = $env:ARDUINO_CLI
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$buildScript = Join-Path $scriptDir 'build-ufp2.ps1'
$buildPath = Join-Path $repoRoot 'build\firmware-UFP2_V160'
$distPath = Join-Path $repoRoot 'dist\UFP2_V160'
$baseName = 'UFP2_V160.ino'
$binFile = Join-Path $buildPath "$baseName.bin"
$hexFile = Join-Path $buildPath "$baseName.hex"
$uf2File = Join-Path $distPath 'UFP2_V160.uf2'
$samd21FamilyId = 0x68ED2B88
$applicationStartAddress = 0x2000
$uf2PayloadSize = 256
$uf2MagicStart0 = [Convert]::ToUInt32('0A324655', 16)
$uf2MagicStart1 = [Convert]::ToUInt32('9E5D5157', 16)
$uf2MagicEnd = [Convert]::ToUInt32('0AB16F30', 16)
$uf2FlagFamilyIdPresent = [Convert]::ToUInt32('00002000', 16)

function Write-UInt32Le {
    param(
        [byte[]]$Buffer,
        [int]$Offset,
        [uint32]$Value
    )

    $bytes = [BitConverter]::GetBytes($Value)
    [Array]::Copy($bytes, 0, $Buffer, $Offset, 4)
}

function Convert-BinToUf2 {
    param(
        [string]$InputFile,
        [string]$OutputFile,
        [uint32]$StartAddress,
        [uint32]$FamilyId
    )

    $inputBytes = [IO.File]::ReadAllBytes($InputFile)
    $blockCount = [int][Math]::Ceiling($inputBytes.Length / $uf2PayloadSize)
    $output = [IO.File]::Open($OutputFile, [IO.FileMode]::Create, [IO.FileAccess]::Write)

    try {
        for ($blockNo = 0; $blockNo -lt $blockCount; $blockNo++) {
            $sourceOffset = $blockNo * $uf2PayloadSize
            $remaining = $inputBytes.Length - $sourceOffset
            $copyLength = [Math]::Min($uf2PayloadSize, $remaining)
            $block = New-Object byte[] 512

            Write-UInt32Le $block 0 $uf2MagicStart0
            Write-UInt32Le $block 4 $uf2MagicStart1
            Write-UInt32Le $block 8 $uf2FlagFamilyIdPresent
            Write-UInt32Le $block 12 ($StartAddress + [uint32]$sourceOffset)
            Write-UInt32Le $block 16 $uf2PayloadSize
            Write-UInt32Le $block 20 $blockNo
            Write-UInt32Le $block 24 $blockCount
            Write-UInt32Le $block 28 $FamilyId
            [Array]::Copy($inputBytes, $sourceOffset, $block, 32, $copyLength)
            Write-UInt32Le $block 508 $uf2MagicEnd

            $output.Write($block, 0, $block.Length)
        }
    }
    finally {
        $output.Dispose()
    }
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
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed with exit code ${LASTEXITCODE}"
    }
}

if (-not (Test-Path -LiteralPath $binFile)) {
    throw "Binary not found: $binFile"
}

if (-not (Test-Path -LiteralPath $hexFile)) {
    throw "HEX not found: $hexFile"
}

New-Item -ItemType Directory -Path $distPath -Force | Out-Null

Copy-Item -LiteralPath $binFile -Destination (Join-Path $distPath 'UFP2_V160.bin') -Force
Copy-Item -LiteralPath $hexFile -Destination (Join-Path $distPath 'UFP2_V160.hex') -Force
Convert-BinToUf2 -InputFile $binFile -OutputFile $uf2File -StartAddress $applicationStartAddress -FamilyId $samd21FamilyId

Get-Item -LiteralPath (Join-Path $distPath 'UFP2_V160.bin'), (Join-Path $distPath 'UFP2_V160.hex'), $uf2File |
    Select-Object FullName, Length, LastWriteTime
