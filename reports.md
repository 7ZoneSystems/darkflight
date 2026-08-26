# CWE Arithmetic-Defect Audit Report

**Scope:** whole-repo C++ audit of ESP-FC/DarkFlight flight-controller firmware for numeric-computation and arithmetic weakness classes, anchored to the MITRE CWE database v4.14 (vendored at `resources/cwe/cwec_v4.14.xml`, [download](https://cwe.mitre.org/data/xml/cwec_v4.14.xml.zip)).
**Trigger:** commit `f1748fe` ("Suspicious add with sizeof", GitHub code-scanning alert no. 3).
**Target platforms:** ESP32 family first (ESP32 / S2 / S3 / C3 per README); esp8266 and RP2040/RP2350 are partial targets only.

## 1. Weakness classes audited (per CWE v4.14)

| CWE | Name | Audit focus in this repo |
|-----|------|--------------------------|
| CWE-190/191 | Integer Overflow / Underflow (Wraparound) | wire length math, EMA accumulators, timestamp deltas |
| CWE-192/197/681 | Integer Coercion / Numeric Truncation | `uint16_t`↔`int` narrowing on MSP/GPS fields |
| CWE-369 | Divide By Zero | timer denominators, sensor calibration math, rate conversions |
| CWE-467 | Use of sizeof() on Pointer Type | pointer-arithmetic offset math (the fixed alert) |
| CWE-680/787/120 | Overflow → Buffer Overflow; buffer over-read/write | MSP/CRSF/UBX frame parsers |
| CWE-1284/805/806 | Improper Validation of Quantity in Input | wire-supplied lengths used as offsets/sizes |
| CWE-1335 | Incorrect Bitwise Shift of Integer | shift macros (`LOG2`), channel packing |

## 2. Validation of the existing autofix (`f1748fe`, GpsSensor.cpp:745)

**Original (CWE-467):**
```cpp
const uint32_t key = *(reinterpret_cast<const uint32_t*>(_ubxMsg.payload) + sizeof(Gps::UbxCfgValsetHeader));
```
Pointer arithmetic on `uint32_t*` scales by 4 → reads byte offset **16** instead of **4**.

**Autofix:** moved the cast inside — now reads offset 4. **Verdict: correct**, per u-blox M10 Interface Description (UBX-21035062 §3.10.4.2): UBX-CFG-VALGET payload = `version(u1) | layers(u1) | position(u2) | keyId(u4)...`; first key sits at byte offset 4 = `sizeof(UbxCfgValsetHeader)` (packed 1+1+2). Cross-checked against u-blox's own reference implementation (`ubxlib` `u_gnss_cfg.c`: `pMessageIn + 4`, minimum-length check before decode).

**Residual defects found in the same statement (fixed here):**
1. **CWE-1284/CWE-806** — no `_ubxMsg.length >= 8` guard; the parser accepts any checksum-valid frame of length 0–510 and the 512 B payload buffer is *not* cleared between frames, so a truncated CFG-VALGET read stale bytes from the previous message.
2. Alignment assumption of a raw `uint32_t` dereference — replaced with `memcpy` (free on aligned ESP32 targets, safe everywhere).

## 3. Defects found and fixed

### Batch A — wire-frame validation (`bdf1f7e`)

| # | File:line | CWE | Defect | Fix |
|---|-----------|-----|--------|-----|
| A1 | `Sensor/GpsSensor.cpp:745` `handleCfgValGet` | 1284/806 | 4-byte key read without min-length check; stale-buffer exposure | length ≥ header+key guard + `memcpy` |
| A2 | `Sensor/GpsSensor.cpp:207` `handleNavPvt` dispatch | 1284/806 | missing `_ubxMsg.length >= sizeof(UbxNavPvt92)` guard present on all sibling handlers (POSLLH/VELNED/SOL/SVINFO); 92-byte struct decoded from possibly-shorter frame | guard added |
| A3 | `Sensor/GpsSensor.cpp:1030` `handleVersion` | 806/1284 | reads sw/hw version strings at `payload+30..39` with no minimum length; MON-VER truncated frame → stale/garbage data drives GPS_M8/M9/F9/M10 detection | require length ≥ 40 |
| A4 | `Sensor/GpsSensor.cpp:1013` `handleNmeaGsv` | 191 | `(msgNum - 1) * 4` underflows when msgNum==0 (wire-controlled) producing `SIZE_MAX-3` base index; currently harmless only because wrap lands outside bounds | explicit `msgNum == 0` early return |
| A5 | `Device/InputCRSF.cpp:174` `applyChannels` | 1284/806 | RC-channels frame decoded as 22-byte `CrsfData` without checking `msg.size`; CRC-valid short frame re-parses stale payload bytes into channel values (pre-failsafe path) | require `size >= 2 + sizeof(CrsfData)` |

### Batch B — divide-by-zero / wraparound (`536e182`)

| # | File:line | CWE | Defect | Fix |
|---|-----------|-----|--------|-----|
| B1 | `Utils/Timer.cpp:13` `setInterval` | 369 | `setInterval(0)` divides by zero at boot (`telemetryInterval=0` via CLI/MSP is unclamped) | zero interval → timer disabled, return 0 |
| B2 | `Utils/Timer.cpp:23` `setRate` | 369 | `denom == 0` divides (`mixerSync=0` via CLI reaches `Model::begin` → `mixer.timer.setRate(rate, 0)`); computed `rate == 0` also divided downstream | denom floor at 1; zero rate → disabled timer |
| B3 | `Model.h` `sanitize()` | 1284→369 | `loopSync` was clamped ≥ 1 but `mixerSync` and `telemetryInterval` were not — both boot-panic reachable through config | clamped `mixerSync ≥ 1`, `telemetryInterval ≥ 1 ms` |
| B4 | `Device/Baro/BaroBMP085.cpp:85` `readPressure` | 369 | Bosch compensation divides by `b4`; all-zero calibration (failed I2C read) yields `b4 == 0` → runtime exception mid-flight. BMP280 sibling already had the equivalent `var1 == 0` guard; BMP085 did not | `if (b4 == 0) return NAN;` |
| B5 | `Input.cpp:300` `updateFrameRate` | 191→369 | EMA `frameDelta += ((target - x) >> 3)` reaches 0 after sustained zero-length frames (arithmetic-shift truncation), then `1000000ul / frameDelta` panics | floor delta at 1 µs |

### Batch C — buffer-capacity enforcement (`902bf21`)

| # | File:line | CWE | Defect | Fix |
|---|-----------|-----|--------|-----|
| C1 | `Connect/Msp.cpp:60` `MspMessage::append` | 787/120 | unchecked `std::copy(data, data+len, buffer+received)` into `uint8_t[192]` | truncate to capacity |
| C2 | `Connect/Msp.cpp:37` `MspMessage::readU8` | 127 | unchecked `buffer[read++]`; a handler over-read escapes the object | return 0 past `received` |
| C3 | `Connect/Msp.cpp:95` `MspResponse::writeU8` | 787 | unchecked `data[len++]` into `uint8_t[240]` for any handler that over-writes | drop excess bytes |
| C4 | `Rc/Crsf.h:150` `CrsfMessage::writeU8` | 787 | unchecked `payload[size++ - 2]`; telemetry writers can grow `size` past the 61-byte payload | stop at capacity |

## 4. Verified-safe sites (audited, no change needed)

- **GpsParser.hpp** — UBX/NMEA state machines cap `length < 511`, `written ≤ 511` into `payload[512]`; NMEA NUL-terminates. No overflow.
- **UbxRequest/UbxFrame** — write-guarded against its 72-byte payload; max serialized frame 76 ≤ `data[80]`.
- **BMP280** — has the Bosch-recommended `var1 == 0` guard; int64 intermediate math per datasheet.
- **MSP V1/V2 parser** — rejects `hdr->size > MSP_BUF_SIZE` before payload fill.
- **CRSF `parse()`** — max write index 61 into exactly-64-byte packed struct (`static_assert` enforced).
- **SBUS/IBUS parsers** — writes bounded by fixed frame-size constants.
- **Blackbox sampling** — `pDenom == 0` takes the direct-rate branch (safe); `blackboxCalculateSampleRate` never receives 0; `LOG2` macro is division-based and total for all inputs ≥ 0 (no CWE-1335).
- **Actuator conditions** — `c.ch` range-checked before use (`Control/Actuator.cpp:20`), so MSP-set channel bytes cannot OOB-index inputs.
- **`micros()` timestamp subtraction** (`gps.interval`, input lossTime) — unsigned wraparound idiom, correct across rollover.
- **`alignToClock`** — divisor starts at 1, monotonic termination.
- **Float-divisor sites** (Rates/Pid/Filter config math) — IEEE inf/NaN semantics, no exception risk; flagged as data-quality-only residual.

## 5. Verification

- Unit tests: `pio test -e native` — **197/197 passed** after each fix batch (test_gps, test_msp, test_input_crsf, test_fc, test_math, test_ahrs, test_gyro, test_esc).
- cppcheck (warning profile, C++17) over all modified files: no new findings; only pre-existing uninitialized-member style notes unrelated to this audit.
- Commits kept per defect family; nothing pushed.

## 6. Residual recommendations (not fixed, tracked)

1. `TelemetryCRSF::_current` and several state structs rely on `begin()` initialization rather than constructors (cppcheck `uninitMemberVar`) — hygiene, not arithmetic.
2. NMEA checksum is parsed but never verified in `NmeaParser` — corrupted sentences are processed if syntactically valid.
3. `handleVersion()` extension-string scans (`strstr` at offsets 40/70/100/130) trust u-blox's fixed 30-byte field layout once minimum lengths pass; a hostile module could still steer GNSS-support detection within received data (acceptance risk inherent to auto-config).
