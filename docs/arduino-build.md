# Arduino IDE Build Fingerprint

Archived log:

```text
docs/arduino-ide-1.8.19-build.log
```

Original sketch path in the log:

```text
C:\Users\juerg\Documents\Arduino\Sketches\UFP2_V160\UFP2_V160.ino
```

Recovered source path:

```text
C:\Users\juerg\Documents\_onedrive_kopie\Desktop\SW\UFP2_V160.ino
```

## IDE Inputs

- Arduino IDE: 1.8.19
- Board package: `adafruit:samd@1.7.17`
- FQBN: `adafruit:samd:adafruit_feather_m0:opt=small,usbstack=arduino,debug=off`
- Board: `adafruit_feather_m0`
- Core: `arduino`
- CPU: SAMD21G18A / Cortex-M0+
- Toolchain: `arm-none-eabi-gcc 9-2019q4`

## Libraries Resolved by the IDE Log

```text
Wire 1.0                         Adafruit SAMD core
SPI 1.0                          Adafruit SAMD core
Adafruit_ZeroDMA 1.1.3           Adafruit SAMD core
RadioHead-master 1.112           Arduino sketchbook
U8g2 2.31.2                      Arduino sketchbook
DimmerZero-master 1.0.0          Arduino sketchbook
```

## Original IDE Size

```text
Der Sketch verwendet 52984 Bytes (20%) des Programmspeicherplatzes. Das Maximum sind 262144 Bytes.
```

## Known Oddity

`UFP2_V160.ino` contains a V1.60 release-history entry for 2021-11-15, but the source
still defines:

```cpp
const float SWversion = 1.51;
```

That mismatch is intentionally preserved. A build lock is not a beautician.

The recovered V1.60 source also contained `-1` sentinel values inside a `byte`
lookup table. The historical V1.51 source used `255` for the same table, which is
the actual byte value produced by `-1`. The repository source uses `255` so modern
Arduino CLI can compile it without C++ narrowing errors.
