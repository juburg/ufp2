param(
    [switch]$SkipExport
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$exportScript = Join-Path $scriptDir 'export-ufp2-firmware.ps1'
$distRoot = Join-Path $repoRoot 'dist\UFP2_V160'
$packageDir = Join-Path $repoRoot 'dist\UFP2_V160_Flashpaket'
$zipFile = Join-Path $repoRoot 'dist\UFP2_V160_Flashpaket.zip'
$bossac = Join-Path (Split-Path -Parent $repoRoot) '.arduino-cli-ufp2\data\packages\adafruit\tools\bossac\1.8.0-48-gb176eee\bossac.exe'

if (-not $SkipExport) {
    & $exportScript -SkipCoreInstall
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware export failed with exit code ${LASTEXITCODE}"
    }
}

$requiredFiles = @(
    (Join-Path $distRoot 'UFP2_V160.bin'),
    (Join-Path $distRoot 'UFP2_V160.hex'),
    (Join-Path $distRoot 'UFP2_V160.uf2'),
    $bossac
)

foreach ($file in $requiredFiles) {
    if (-not (Test-Path -LiteralPath $file)) {
        throw "Required file missing: $file"
    }
}

if (Test-Path -LiteralPath $packageDir) {
    Remove-Item -LiteralPath $packageDir -Recurse -Force
}

New-Item -ItemType Directory -Path $packageDir -Force | Out-Null

Copy-Item -LiteralPath (Join-Path $distRoot 'UFP2_V160.bin') -Destination $packageDir -Force
Copy-Item -LiteralPath (Join-Path $distRoot 'UFP2_V160.hex') -Destination $packageDir -Force
Copy-Item -LiteralPath (Join-Path $distRoot 'UFP2_V160.uf2') -Destination $packageDir -Force
Copy-Item -LiteralPath $bossac -Destination (Join-Path $packageDir 'bossac.exe') -Force

$flashBat = @'
@echo off
setlocal
cd /d "%~dp0"

echo UFP2 Firmware Flash fuer Adafruit Feather M0
echo.
echo Schritt 1: Feather M0 per USB anschliessen.
echo Schritt 2: Reset-Taster am Feather M0 zweimal kurz druecken.
echo Schritt 3: Den COM-Port im Geraetemanager pruefen.
echo.

if "%~1"=="" (
  echo Gefundene COM-Ports:
  powershell -NoProfile -ExecutionPolicy Bypass -Command "Get-CimInstance Win32_SerialPort | Select-Object DeviceID,Description | Format-Table -AutoSize"
  echo.
  set /p PORT=COM-Port eingeben, z.B. COM15: 
) else (
  set PORT=%~1
)

if "%PORT%"=="" (
  echo Kein COM-Port angegeben.
  pause
  exit /b 1
)

echo.
echo Flashe UFP2_V160.bin auf %PORT% ...
echo.
bossac.exe -i -d --port=%PORT% -U -i --offset=0x2000 -w -v UFP2_V160.bin -R

if errorlevel 1 (
  echo.
  echo FEHLER: Flashen fehlgeschlagen.
  echo Reset zweimal druecken, COM-Port pruefen und erneut starten.
  pause
  exit /b 1
)

echo.
echo Fertig. Das Board wurde neu gestartet.
pause
'@

Set-Content -LiteralPath (Join-Path $packageDir 'flash-UFP2.bat') -Value $flashBat -Encoding ascii

$readme = @'
UFP2 Firmware Flashpaket fuer Adafruit Feather M0
=================================================

Dieses Paket benoetigt keine Arduino IDE.

Inhalt
------
- UFP2_V160.bin       Firmware fuer bossac
- UFP2_V160.hex       Alternative Firmwaredatei
- UFP2_V160.uf2       Nur fuer Feather M0 mit UF2-Bootloader
- bossac.exe          Flashprogramm
- flash-UFP2.bat      Einfacher Windows-Flasher

Normaler Weg mit COM-Port
-------------------------
1. Feather M0 per USB anschliessen.
2. Reset-Taster am Feather M0 zweimal kurz druecken.
3. Im Windows-Geraetemanager den COM-Port merken, z.B. COM15.
4. flash-UFP2.bat doppelklicken.
5. COM-Port eingeben.
6. Warten bis "Fertig" erscheint.

Direkt per Kommandozeile
------------------------
flash-UFP2.bat COM15

Wenn ein Laufwerk FEATHERBOOT erscheint
--------------------------------------
Dann hat das Board einen UF2-Bootloader. In diesem Fall einfach
UFP2_V160.uf2 auf das Laufwerk FEATHERBOOT kopieren.

Wenn kein Laufwerk erscheint
----------------------------
Das ist bei vielen Feather M0 normal. Dann flash-UFP2.bat verwenden.

Fehlerhilfe
-----------
- USB-Kabel pruefen. Manche Kabel sind nur Ladekabel.
- Reset wirklich zweimal kurz druecken.
- COM-Port kann sich nach Reset aendern.
- Wenn flash-UFP2.bat fehlschlaegt: Reset zweimal druecken und sofort erneut starten.
'@

Set-Content -LiteralPath (Join-Path $packageDir 'README_KURZ.txt') -Value $readme -Encoding ascii

if (Test-Path -LiteralPath $zipFile) {
    Remove-Item -LiteralPath $zipFile -Force
}

$packageFiles = Get-ChildItem -LiteralPath $packageDir -File
Compress-Archive -LiteralPath $packageFiles.FullName -DestinationPath $zipFile -Force

Get-Item -LiteralPath $zipFile, $packageDir |
    Select-Object FullName, Length, LastWriteTime
