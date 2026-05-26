# Toolchain Lock for UFP2

This repository contains the locked Arduino CLI rebuild path for UFP-II firmware:

```text
UFP2_V160/UFP2_V160.ino
```

## Pinned Build Inputs

- Board package: `adafruit:samd@1.7.17`
- Board package index: `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`
- FQBN: `adafruit:samd:adafruit_feather_m0:opt=small,usbstack=arduino,debug=off`
- Board: Adafruit Feather M0, SAMD21G18A / Cortex-M0+

The build script uses a workspace-local Arduino CLI data directory under:

```text
../.arduino-cli-ufp2
```

This prevents the compile from silently using libraries from the user's Arduino
sketchbook. Silence is nice in churches, less so in firmware builds.

## Vendored Arduino Libraries

The script builds only with these repository-local libraries:

```text
third_party/arduino-libraries/RadioHead-1.112
third_party/arduino-libraries/U8g2-2.31.2
third_party/arduino-libraries/DimmerZero-1.0.0
```

Core libraries are supplied by `adafruit:samd@1.7.17`:

```text
Wire 1.0
SPI 1.0
Adafruit_ZeroDMA 1.1.3
```

## Build Command

```powershell
.\tools\build-ufp2.ps1
```

## Firmware Export

```powershell
.\tools\export-ufp2-firmware.ps1 -SkipCoreInstall
```

The batch wrapper avoids PowerShell execution policy issues:

```cmd
tools\export-ufp2-firmware.bat -SkipCoreInstall
```

The export creates `dist/UFP2_V160/UFP2_V160.bin`,
`dist/UFP2_V160/UFP2_V160.hex`, and `dist/UFP2_V160/UFP2_V160.uf2`.
The UF2 file uses SAMD21 family ID `0x68ED2B88` and target address `0x2000`,
matching the Feather M0 linker script with the 8 KB bootloader region.

## Verified Output

Fresh local build with the repository-local Arduino CLI path:

```text
UFP2_V160 / V1.6.0:
Sketch uses 53008 bytes (20%) of program storage space. Maximum is 262144 bytes.
```

The archived Arduino IDE 1.8.19 compiler log reported 52984 bytes. The CLI build is
24 bytes larger, matching the same small delta seen on the related UFP2_RRC rebuild.

## Offline Boundary

The third-party Arduino libraries are vendored in this repository.
The Adafruit SAMD core and ARM toolchain are installed by Arduino CLI from the
Adafruit and Arduino package indexes when the script is run without
`-SkipCoreInstall`.

For a fully offline rebuild, archive the workspace-local
`../.arduino-cli-ufp2/data/packages` directory separately.
