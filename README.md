# UFP2 Firmware

Locked rebuild repository for the UFP-II firmware.

The current production build is:

```text
UFP2_V160/UFP2_V160.ino
```

The source was recovered from:

```text
C:\Users\juerg\Documents\_onedrive_kopie\Desktop\SW\UFP2_V160.ino
```

That file is the V1.60 production source from 2021-11-15. The code still reports
`SWversion = 1.51` on the display. This is preserved as-is because the goal here is
to rebuild the known production binary, not to polish the archaeology.

## Build

Use Arduino CLI:

```powershell
.\tools\build-ufp2.ps1
```

The script installs and uses `adafruit:samd@1.7.17` in a workspace-local Arduino CLI
data directory and compiles only against the vendored third-party libraries in this
repository.

## Target

```text
Board: Adafruit Feather M0
FQBN:  adafruit:samd:adafruit_feather_m0:opt=small,usbstack=arduino,debug=off
```

## Historical Compiler Log

The original Arduino IDE 1.8.19 compiler log is archived at:

```text
docs/arduino-ide-1.8.19-build.log
```

