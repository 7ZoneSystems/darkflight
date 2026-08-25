# ESP-FC Baseline Report

Date: 2026-08-24
Branch: `main` (synced with `origin/main`, fork of rtlopez/esp-fc; `upstream` treated read-only)
Working tree: clean — no repository files modified; all fixes were environment-level.

## Repository architecture

- Firmware entry: `src/main.cpp`; core firmware lives in `lib/Espfc/src`:
  - `Control/` (Controller, Pid, Rates, Actuator, Fusion)
  - `Device/` (gyro/baro/mag drivers, Input* receivers, BusI2C/BusSPI)
  - `Telemetry/`, `Blackbox/`, `Connect/` (MSP, CLI), `Utils/` (Crc, Filter, Math)
- Local libraries under `lib/`:
  - `AHRS` (Mahony, Madgwick, Rtqf, Kalman, Complementary)
  - `EscDriver` (per-MCU backends: Esp32/Esp32c3/Esp8266/RP2040 + base)
  - `EspWire`, `Gps`, `MultiButton`, `printf`
  - `betaflight` (vendored headers: blackbox, msp, serial_4way, platform glue)
- `platformio.ini` environments:
  - Default targets: `esp32`, `esp32s2`, `esp32s3`, `esp32c3`, `esp8266`, `rp2040`, `rp2350`
  - `[env:native]`: unit tests on host using Unity + ArduinoFake (`-DUNIT_TEST`)
  - `check_tool = cppcheck`; extra scripts: `bin/pio_format.py` (custom targets
    `format` / `check_format`), `bin/pio_merge_firmware.py`
- CI (`.github/workflows/platformio.yml`): `pio test -e native` gate, then builds of all 7 targets.
- Git state: `main` up to date with `origin/main`; nothing committed/pushed during baseline work.

## Development environment (ARM64-native, no emulation)

Host: aarch64, Debian 13 (trixie) inside proot-distro on Termux/Android, f2fs rootfs.

| Tool | Version | Source |
|---|---|---|
| PlatformIO Core | 6.1.19 | pip |
| cppcheck | 2.17.1 | apt |
| clang-format | 20.1.7 (matches repo pin; system had 21 from Termux) | pip |
| Host gcc/g++ | 14.2.0 | preinstalled |


## Unit tests — `pio test -e native`

Result: **190 test cases — 188 passed, 1 failed**.

Suites fully green: `test_ahrs`, `test_esc`, `test_fc`, `test_gps`,
`test_gyro`, `test_math`, `test_msp`.

Failure: `test_input_crsf.test_crsf_encode_msp_v1_fragmented`
(`Expected 33 Was 46`). Investigation showed this is **not** a flaky or
environmental failure but collateral damage from a genuine memory-corruption bug
(described below). A SIGHUP at process exit was observed once (sandbox artifact);
all 14 tests in the suite did run to completion.

## Genuine repository bug found (not yet fixed)

**`CrsfMessage::payload` buffer overflow on fragmented MSP-over-CRSF responses**

- Location: `lib/Espfc/src/Rc/Crsf.h` (`struct CrsfMessage`),
  used by `Crsf::encodeMspData()` in `lib/Espfc/src/Rc/Crsf.cpp:93` and invoked
  from `lib/Espfc/src/Telemetry/TelemetryCRSF.h:88`.
- `uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]` = **59 bytes**, but a maximum-size
  MSP_RESP frame writes **61 bytes** into it:
  - 3 extended-header bytes (`dst`, `origin`, `status`) +
  - 57-byte MSP chunk (`CRSF_PAYLOAD_SIZE_MAX - 1`) +
  - CRC byte stored at `payload[size - 2]`.
- Proven with a standalone heap-canary reproduction: two canary bytes past the
  struct end were overwritten with the last data byte (`0x36`) and the CRC
  (`0xFE`).
- Consequences:
  - In the failing unit test it corrupts the adjacent stack variable `seq`,
    which is why the status byte assertion sees `0x2E` instead of `0x21`.
  - On-device it is undefined behavior corrupting adjacent RAM whenever a
    fragmented MSP response is sent over CRSF (e.g., Betaflight Configurator
    reading large parameter groups).
- Note: `crc()` also performs a 1-byte OOB read for such frames.
- Suggested minimal fix: enlarge the array to
  `uint8_t payload[CRSF_FRAME_SIZE_MAX - 3]` (= 61, exact worst-case fit) or the
  more conservative `CRSF_FRAME_SIZE_MAX`. Not applied yet per ground rules
  ("explain before changing", "do not modify code merely to make tests pass").

## Static checks and formatting

- `pio run -t check_format` (**FAILS — genuine repo state, not environment**):
  3157 clang-format violations across all 178 files in `src/` + `lib/Espfc/src`.
  The codebase predates full `.clang-format` adoption (post "code-tidy" gap).
  Resolution belongs in a dedicated reformat commit
  (`pio run -e native -t format`).
- `bin/format-check.sh` flaw: it scans `.pio/` build output too (60 bogus hits
  among its 180 flags); it should exclude `.pio/` (and probably vendored
  `lib/betaflight`).
- `pio check` (cppcheck static analysis): not yet executed.

## Builds

- `esp32`: interrupted at user request ~90% through compilation (framework,
  libraries, and most of Espfc compiled; linking not reached). Bootloader and/
  partition binaries produced. Xtensa toolchain + Arduino framework installed OK.
- `esp32s2`, `esp32s3`, `esp32c3`, `esp8266`, `rp2040`, `rp2350`: not started.
  Each requires additional multi-hundred-MB toolchain downloads and serial
  compilation (~hours each on this CPU-throttled sandbox). CI covers them.
- Installed PlatformIO packages so far: `tool-scons`,
  `toolchain-xtensa-esp32`, `framework-arduinoespressif32`, `tool-esptoolpy`;
  platforms: `espressif32`, `native`.

## Warnings/errors summary and causes

| Symptom | Cause | Class |
|---|---|---|
| `cc1plus` ICE / futex errors during parallel builds | proot sandbox + memory pressure at `-j8` | environment |
| Package install `[Errno 22] Invalid argument` | proot d_type misreport -> stdlib copytree `readlink()` on regular files | environment |
| `Errno 38` / SIGHUP artifacts in logs | proot syscall gaps / sandbox restarts | environment |
| ASan refuses to init (`CHECK failed ... kSpaceBeg`) | sandbox address-space restrictions | environment |
| `test_crsf_encode_msp_v1_fragmented` failure | genuine 2-byte OOB write in `CrsfMessage` | **repository bug** |
| `check_format` target failure | codebase not formatted against `.clang-format` | repository state |
