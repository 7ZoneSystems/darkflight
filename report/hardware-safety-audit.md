# DarkFlight — Deep Correctness/Safety Audit (hardware & regression focus)

Date: 2026-08-25 · Auditor scope: ESP32 targets, GPS/navigation/sensor paths,
motor-output path, resource conflicts. All commits local; nothing pushed.

## 1. Documentation sources used

See `resources/esp32/README.md` for the full index (URLs, versions, targets,
relevance). Official Espressif sources: IDF v4.4 peripheral/API pages (GPIO,
UART, I2C, SPI, GPTimer, LEDC, MCPWM, RMT, ADC, interrupt allocation,
FreeRTOS-SMP), bootloader/system-reset pages, ESP32 datasheet PDF (strapping
pins), ECO/errata PDF. Key constraints extracted there drive the findings
below.

## 2. Audit methodology

1. Fetched official docs first; built a hardware-constraint checklist
   (strapping pins, flash-reserved GPIOs, input-only pads, ADC2/Wi-Fi rule,
   RMT channel/divider/IRAM rules, reset behavior).
2. Git-history reconstruction: fork merge point (`ef56cdd`, upstream
   `ba93f25`), fork-only GPS series (`cc62f8b`…`ad9cff3`), EscDriver history
   (`eb7f048`, `f756ee3`, `0fa390a`, …), tidy commits.
3. Path tracing for the reported motor/D4 symptom: config → pin defines →
   mixer → ESC driver attach/write → RMT transmit → GPIO matrix.
4. Source-level unit/bounds review of every GPS producer/consumer pair
   (NAV-PVT, NAV-VELNED, NAV-SOL, NAV-SAT, NAV-SVINFO, NMEA) against the
   u-blox interface-manual units.
5. Resource-conflict sweep: RMT/LEDC/MCPWM/timer/UART/I2C/SPI/ADC usage map.
6. Findings classified strictly; fixes applied only where root cause proven;
   native suite re-run after each change (197/197 at finish).

## 3. Confirmed bugs

### DF-001 — NAV-PVT velocities stored 10× too small (unit mismatch)

- Severity: HIGH (navigation safety margin; headline GPS-hold feature)
- Affected targets: all (any receiver sending NAV-PVT — M8/M9/F9/M10 default)
- Files/functions: `lib/Espfc/src/Sensor/GpsSensor.cpp :: handleNavPvt()`
- Introduced by: `8eb07a5`/`f9d3585` "gps initial work" (2025-02-08)
- First known good: n/a (bug present since GPS ingestion landed)
- Root cause: NAV-PVT `velN/velE/velD/gSpeed/sAcc` are **cm/s** per the
  u-blox interface manual; handler stored them raw into the shared GPS state
  whose convention is **mm/s** (NAV-VELNED scales ×10; NMEA path produces
  mm/s; `handleNavVelned` even documents "like NAV-PVT"). Consumers
  (`PosHold` ×0.001, `MSP_RAW_GPS` ÷10, blackbox ÷10, CLI) therefore saw
  velocity 10× too small from PVT-equipped receivers → position hold corrects
  drift 10× too weakly; MSP/blackbox report wrong speeds. CRSF telemetry
  formula happened to compensate only on this broken path.
- Evidence: side-by-side producer audit (VELNED ×10 vs PVT raw vs NMEA mm/s);
  consumer expectations in PosHold.hpp:227-228 and MspProcessor.cpp:1711;
  u-blox spec units. Reproducible by feeding equal physical velocity via both
  messages and comparing state (now covered by test).
- Fix: commit `87d6f4a` — scale PVT values ×10 at ingestion; speed3d
  recomputed from scaled components; comment corrected.
- Regression tests: `test_pvt_velocity_units_mm_s`,
  `test_pvt_and_velned_paths_agree_on_units`.

### DF-002 — NAV-SAT parsed without length validation

- Severity: LOW-MEDIUM (data integrity; not memory-unsafe)
- Files/functions: `GpsSensor.cpp :: handleNavSat()`
- Introduced by: same "gps initial work" commits
- Root cause: sibling handlers guard `_ubxMsg.length >= sizeof(...)`; NAV-SAT
  trusted `numSvs` unconditionally, so a short-but-CRC-valid frame (or legacy
  variant) reports stale buffer bytes as satellites. Reads stay inside the
  512-byte payload (flexible-array struct + SAT_MAX cap) → no memory hazard.
- Fix: commit `87d6f4a` — header-length guard + clamp count to delivered
  entries (mirrors NAV-SVINFO pattern).
- Regression test: `test_nav_sat_count_clamped_to_delivered_payload`.

## 4. Likely bugs

None beyond DF-002 at classification time (it was LIKELY pre-fix, CONFIRMED
by code-path proof during fix).

## 5. Potential issues (need hardware-in-the-loop)

- **PosHold earth→body rotation & heading source** (`rotateEarthToBody`):
  sign convention vs esp-fc euler-yaw and angle-setpoint conventions not
  provable from source alone; fused yaw (mag-less builds drifts) used as the
  rotation angle. A sign/convention error would tilt away from drift instead
  of into it. Requires bench/HIL validation with a known heading.
- **Async RMT tx-end ISR jitter under load**: GPS UART + WiFi(ESP-NOW)
  interrupts can delay `txDoneCallback`; no timing evidence yet; sync mode
  unaffected.
- **GPS fix acquired airborne**: home capture only on mode entry; entering
  POSHOLD right after in-air fix acquisition starts hold from current point —
  intended, but worth HIL confirmation of transients.

## 6. Investigated and ruled out

- **"Motor 4 / D4 regression" — no firmware defect found in output path.**
  Evidence:
  - Full diff of output stack between pre-GPS baseline (`ef56cdd^1`) and HEAD:
    `EscDriver*`, `Mixer`, `Output` changes are cosmetic except the upstream
    minthrottle→motor_idle rename (global, not pin-specific).
  - Fork GPS series touches none of: EscDriver, Output, Target headers,
    timer/RMT allocation (verified per-commit `--stat`).
  - GPIO4 per official docs: ordinary digital IO; **not** a strapping pin;
    analog functions (ADC2_CH0/TOUCH0) unused by DarkFlight; any GPIO can be
    driven by any RMT channel through the GPIO matrix → pin choice cannot
    fail from routing.
  - RMT channel assignment depends on output *index*, not pin number; nothing
    else in the tree uses RMT (grep-verified), so no channel collision.
  - NVS config load is magic+version+`sizeof(ModelConfig)` checked
    (`Utils/Storage`); schema growth after `ad9cff3` forces clean defaults —
    stale-layout corruption impossible.
  - `EscDriverEsp32::instances[]` overwrite theory disproven: motor/servo
    attach index sets are disjoint.
  Residual real-world suspects (checklist for bench repro): board silkscreen
  label vs GPIO number confusion ("D4" on ESP8266 boards = GPIO2!);
  bidir-DShot idle-high line into a non-telemetry ESC; persisted-config reset
  to defaults (protocol/idle change) after upgrade. Capture `diff all` from
  good/bad firmware + scope the signal line to close this out.
- **UBX/NMEA parser overruns**: payload caps enforced (`written < 511`,
  `length ≥ 511` rejected, NUL-terminated NMEA), consumers reset state per
  message; CRC gates READY state.
- **NAV-SOL/SVINFO/POSLLH/VELNED handlers**: all length-guarded (SVINFO even
  computes delivered-entry count).
- **Storage deserialization across schema change**: safe (see above).
- **I2C/SPI address conflicts, ADC2 usage, GPIO34-39 misuse, flash-pin
  reuse**: current target maps comply with the documented constraints.

## 7. Environment issues

None new. Prior proot/codespace artifacts remain documented in git history
only.

## 8. Commit ranges examined

- `ef56cdd^1..HEAD` (fork-side full delta, focus on Output/EscDriver/Target)
- `cc62f8b`, `1f420e3`, `964de32`, `f9dd583`, `3533581`, `7303f75`,
  `8136e6a`, `45d6b84`, `ad9cff3` (GPS series, per-commit stats+diffs)
- `ba93f25`, `78c9800`, `4b1cb64`, `29901cb`/`0459e88` (upstream tidy/target/
  buzzer history touching pins)
- EscDriver lineage: `eb7f048`, `aec5ccd`, `4b1cb64`, `6692e87`, `f81047d`
- Tidy pass `08dec02`…`1388930` (token-proofs already on file)

## 9. Fixes made

| Commit | Fix | Tests |
|---|---|---|
| `87d6f4a` | DF-001 NAV-PVT mm/s scaling + DF-002 NAV-SAT bounds | 3 new tests (`test_gps`), suite 197/197 |

## 10. Remaining risks / follow-ups

1. HIL validation list above (§5) — especially PosHold rotation direction.
2. Bench isolation checklist for the historical D4/motor report (§6).
3. ESP32 boards with PSRAM: UART2 pins 16/17 conflict — document per-board.
4. Default motor on strapping pin GPIO12 (`ESPFC_OUTPUT_3`): consider a
   documented warning or future default change with migration note (wiring
   compatibility makes silent renumbering unsafe).
5. Open items ESPFC-006…009 (see `msp-crsf-memory-safety-audit.md`).
