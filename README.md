# DarkFlight

DarkFlight is an independent, DIY, low-cost flight-controller firmware for ESP32-class microcontrollers. It focuses on stability and safety through extensive testing, and on a modular architecture that lets proven open-source control, navigation, estimation, sensor, and telemetry implementations be integrated, evaluated, and replaced where their licenses permit.

DarkFlight is derived from [rtlopez/esp-fc](https://github.com/rtlopez/esp-fc) (**ESP-FC**) and builds upon its MIT-licensed codebase. It is developed independently — it is **not** an official ESP-FC project and is not endorsed by or affiliated with the upstream author.

Repository: <https://github.com/7ZoneSystems/espfc>

## Based on ESP-FC (upstream)

- **Origin:** DarkFlight started as a fork of [rtlopez/esp-fc](https://github.com/rtlopez/esp-fc) and retains most of its architecture, toolchain, and documentation conventions.
- **License:** the original ESP-FC project is distributed under the [MIT license](/LICENSE). All upstream copyright and attribution is retained; see [License](#license).
- **Relationship:** DarkFlight is an independent derivative project with a broader, integration-oriented scope. It does not represent, speak for, or continue the upstream project in any official capacity.
- **Upstream community:** the original project's documentation and community resources (e.g., the [ESP-FC Discord](https://discord.gg/jhyPPM5UEH)) remain valuable references for hardware inherited from upstream.
- **Upstream contributions:** DarkFlight intends to contribute relevant bug reports and appropriate fixes back to upstream ESP-FC when they are useful there. Memory-safety defects found during DarkFlight audits of inherited CRSF/MSP code have already been fixed locally and documented for upstream submission (see `report/msp-crsf-memory-safety-audit.md`).

## Project direction

Compared to upstream ESP-FC, DarkFlight aims at:

- faster development through modular integration,
- stability and safety through extensive testing and simulation,
- integration of proven open-source flight-control, navigation, estimation, sensor, and telemetry implementations where their licenses permit,
- support for a broader range of hardware and sensors,
- a modular architecture allowing alternative implementations to be evaluated and replaced,
- strong automated testing, static analysis, simulation, and hardware validation,
- continued contribution of useful bug reports and appropriate fixes back to upstream ESP-FC.

These are project goals; items listed under *Not yet implemented* below must not be assumed to work.

# Features

Inherited from ESP-FC and verified present in this repository:

* Espressif targets (ESP32, ESP32-S3 recommended; ESP32-S2/C3 experimental)
* ESC protocols (PWM, Oneshot125/42, Multishot, Brushed, Dshot150/300/600 bidirectional)
* PPM, SBUS, IBUS and CRSF receivers, plus builtin ESP-NOW receiver ([read more...](/docs/wireless.md))
* SPI and I2C gyro modules (MPU6050, MPU9250, ICM20602, ICM42688, BMI160)
* Flight modes: ACRO, ANGLE, AIRMODE
* Configurable gyro filters (LPF, dynamic notches, dTerm, RPM)
* Blackbox recording (OpenLog/OpenLager/Flash)
* Up to 4kHz gyro/loop on ESP32 with SPI gyro
* MSP and CLI protocol interfaces
* Betaflight configuration tool compatible (v10.10)
* Resource/pin mapping, in-flight PID tuning
* Buzzer, LED and voltage monitor, failsafe mode

Developed within DarkFlight (implemented and covered by unit tests):

* **GPS position hold** — cascade position/velocity control based on ideas documented in public INAV/PX4 design notes (original implementation, no copied source; see [references](/docs/GPS_POSITION_HOLD_REFERENCES.md)). Activates only with a healthy GPS fix and falls back to Angle mode otherwise.
* **GNSS support** — self-contained GPS library (`lib/Gps`, UBX/NMEA protocol parsing, auto-config/auto-baud) driving u-blox M8/M9/F9/M10 modules, including dual-band L1+L5 on M10 ([GPS setup](/docs/setup.md), [configuration](/docs/GPS_CONFIGURATION.md)).
* **Attitude estimation options** — selectable Madgwick / Mahony / RTQF sensor fusion, with additional Kalman and Complementary estimators available in the AHRS library for evaluation.
* **Altitude hold** — barometer/accelerometer altitude estimation (complementary fusion) feeding a thrust PID in the control loop.

### Not yet implemented

* GPS return-to-home / GPS rescue navigation (only configurator compatibility placeholders exist — do not rely on them)
* MS5611 barometer driver

Frames: Quad X. As with upstream, many options displayed in Betaflight Configurator are not backed by functionality; if an option cannot be changed, it is not supported.

# Documentation

 * [Setup Guide](/docs/setup.md)
 * [Wiring](/docs/wiring.md)
 * [CLI Commands](/docs/cli.md)
 * [WiFi and ESP-NOW Receiver](/docs/wireless.md)
 * [GPS Configuration](/docs/GPS_CONFIGURATION.md)

For convenience the firmware mimics Betaflight 4.2 compatibility so it can be configured with [betaflight-configurator](https://github.com/betaflight/betaflight-configurator), and blackbox logs can be analyzed with the [online blackbox-log-viewer](https://blackbox.betaflight.com/) — but this software is not Betaflight, nor identical to upstream ESP-FC; expect limitations and differences.

> [!IMPORTANT]
> Before you begin, **read the documentation carefully first!**

# Quick Start

## Requirements

Hardware:
* ESP32 or ESP32-S3 board
* MPU9250 SPI or MPU6050 I2C gyro (GY-88, GY-91, GY-521 or similar)
* u-blox GPS module for position hold features
* PDB with 5V BEC
* Buzzer and some electronic components (optional)

Software:
* [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) (v10.10)
* [CH340 usb-serial converter driver](https://sparks.gogo.co.nz/ch340.html)

## Building and flashing

DarkFlight firmware must be built from source:

```sh
pip install platformio
pio run -e esp32            # build (also: esp32s3, esp32s2, esp32c3, ...)
pio run -e esp32 -t merge   # produce a single flashable image (bin/pio_merge_firmware.py)
```

Then flash the resulting image (`.pio/build/esp32/firmware.bin` or the merged
image):

1. Visit the [ESP Tool Website](https://espressif.github.io/esptool-js/)
2. Click "Connect" and choose device port in dialog
3. Add firmware file and set Flash Address to `0x00`
4. Click "Program"
5. After success power cycle board

![Flashing with esptool-js](/docs/images/esptool-js-flash-connect.png)

> [!NOTE]
> Prebuilt binaries on the [upstream releases page](https://github.com/rtlopez/esp-fc/releases) are ESP-FC builds, not DarkFlight builds.

## Setup

After flashing you need to configure few things first:

 1. Configure pinout according to your wiring, especially pin functions — see the [CLI Reference](/docs/cli.md)
 2. Connect with [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) and set up to your preferences
 3. Test motors without propellers
 4. Follow the [setup guide](/docs/setup.md) before first flight

## Wiring diagrams

[![Example wiring diagrams](/docs/images/espfc_wiring_combined.png)](/docs/wiring.md)

## Supported Modules

 * **ESP32** - recommended
 * **ESP32-S3** - recommended
 * **ESP32-S2** - experimental
 * **ESP32-C3** - experimental, lack of performance, no FPU
 * **RP2350** - experimental, partially works
 * **RP2040** - experimental, lack of performance, no FPU
 * **ESP8266** - obsolete, inherited from upstream, no longer developed

## Supported Sensors and Protocols

 * Gyro: MPU6050, MPU6000, MPU6500, MPU9250, ICM20602, ICM42688, BMI160
 * Barometers: BMP180, BMP280, SPL06
 * Magnetometers: HMC5883, QMC5883, AK8963, QMC5883P
 * Receivers: PPM, SBUS, IBUS, CRSF/ELRS, ESP-NOW
 * ESC protocols: PWM, BRUSHED, ONESHOT125, ONESHOT42, MULTISHOT, DSHOT150, DSHOT300, DSHOT600
 * GPS: u-blox M8, M9, F9 & M10 (dual band, all constellations configurable via CLI)
 * Other protocols: MSP, CLI, BLACKBOX, ESPNOW

## Issues

Report DarkFlight issues using the GitHub [tracker](https://github.com/7ZoneSystems/espfc/issues).
Defects that also affect upstream ESP-FC are documented so they can be reported upstream as well.

## Development

* Visual Studio Code
* [PlatformIO](https://platformio.org/install/ide?install=vscode) extension
* Git

Quality gates used by this project:

```sh
pio test -e native        # host unit test suite (194 test cases at time of writing)
pio check                 # cppcheck static analysis
```

A development container is available via `docker compose` (see [Development docs](/docs/development.md)).

## Licence

This project is distributed under the MIT Licence, inherited from and shared with the original ESP-FC project. Upstream copyright is retained:

```
MIT License

Copyright (c) 2016 rtlopez
```

Bear in mind that:

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
