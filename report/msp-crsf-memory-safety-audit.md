# DarkFlight — Audit & Quality Report

Date: 2026-08-25
Repository: fork of rtlopez/esp-fc (`upstream` read-only). All work below is committed locally on `main`; nothing pushed.

## Current state

| Check | Result |
|---|---|
| `pio test -e native` | **197/197 passed** (re-run after every change) |
| `pio run -t check_format` (CI gate, `src/` + `lib/Espfc/src`, 178 files) | **Success** |
| `bin/format-check.sh` (full repo) | 16 findings, all vendored third-party (`lib/betaflight` ×13, `lib/printf`, `lib/MultiButton`) — intentionally not reformatted |
| `pio check` (cppcheck 2.13) | PASSED (~4 min); findings are pre-existing codebase characteristics |
| Formatter | clang-format **20.1.7** (repo pin), `.clang-format` with `SortIncludes: Never` |

## Deep hardware/GPS audit — 2026-08-25

Second audit pass driven by official Espressif documentation
(`resources/esp32/`, full index + constraint checklist) targeting GPS,
navigation and motor-output paths. Full details in `hardware-safety-audit.md`.

Newly found and fixed (commit `87d6f4a`, regression tests added):

| ID | Defect (short) | Severity | Status | Regression test |
|---|---|---|---|---|
| DF-001 | NAV-PVT velocities/sAcc stored cm/s into mm/s state → GPS hold corrected drift 10× too weakly; MSP/blackbox speeds wrong for PVT receivers | HIGH | FIXED LOCALLY (`87d6f4a`) | `test_pvt_velocity_units_mm_s`, `test_pvt_and_velned_paths_agree_on_units` |
| DF-002 | NAV-SAT parsed without length validation → stale bytes reported as satellites on malformed frames | LOW-MEDIUM | FIXED LOCALLY (`87d6f4a`) | `test_nav_sat_count_clamped_to_delivered_payload` |

Investigated and ruled out with evidence: firmware-side cause of the historical
"motor 4 / D4" misbehavior (output stack unchanged functionally since baseline;
GPIO4 is an ordinary digital IO per official docs; no RMT/pin/NVS-config
corruption mechanism exists). Residual suspects are hardware/configuration-side;
bench checklist in hardware-safety-audit §6.

Potential issues requiring hardware-in-the-loop testing: PosHold earth→body
rotation sign/heading-source convention; async RMT ISR jitter under load.


## Code tidy pass — 2026-08-25

Baseline `08dec02` → final `1388930`: 108 files, +4942/−4229 lines, formatting only.

Safety methodology: include reordering disabled in `.clang-format` before mass edit (some sources depend on include order, e.g. `Arduino.h` before `Utils/MemoryHelper.h` for `IRAM_ATTR`); every formatted file verified token-identical to its pre-image via a comment/string-aware comparator; native test suite green at each checkpoint.

| Commit | Scope |
|---|---|
| `4c5b345` | `.clang-format`: preserve include order |
| `5363039` | Rc sources |
| `9b29b8b` | Connect sources |
| `b5f591e` | Control loop sources |
| `085da9f` | Sensor and mixer sources |
| `0b871af` | Device driver sources |
| `85c554a` | Telemetry and blackbox sources |
| `a2c3a48` | Core framework sources |
| `8d79684` | Firmware entry point (`src/main.cpp`) |
| `607394b` | Unit tests |
| `acc58dd` | First-party libraries (AHRS, EscDriver, Gps, EspWire) |
| `1388930` | Stabilize esp_twi comment wrap (non-idempotent case) |

Excluded as vendored/generated: `.pio/`, `lib/betaflight`, `lib/printf`, `lib/MultiButton`.

No functional behavior changed: code tokens proven identical everywhere; only layout, alignment, comment wrapping, and clang-format's standard namespace-end comments differ.

## Defects found during audits

All memory-safety defects were fixed locally in commits `f333d33`–`7049c0c` era work and are covered by regression tests in `test/test_input_crsf/test_input_crsf.cpp`.

| ID | Defect (short) | Severity | Status | Regression test |
|---|---|---|---|---|
| ESPFC-001 | `CrsfMessage::payload` undersized → OOB write/read on max-size MSP-over-CRSF frames (TX) | CRITICAL | FIXED LOCALLY | `test_crsf_encode_msp_v1_fragmented_no_overflow` |
| ESPFC-002 | `InputCRSF::parse()` OOB writes on max-size inbound frames, pre-CRC-validation | CRITICAL | FIXED LOCALLY | `test_input_crsf_max_size_frame_parse_no_overflow` |
| ESPFC-003 | `decodeMsp()` continuation integer underflow → unbounded `append()` | CRITICAL | FIXED LOCALLY | `test_crsf_decode_msp_short_continuation_frame_ignored` |
| ESPFC-004 | `fillMessage()` short-start-frame underflow → `MspMessage::buffer[192]` overflow + OOB read | CRITICAL | FIXED LOCALLY | `test_crsf_decode_msp_short_start_frame_ignored` |
| ESPFC-005 | Stale `MspMessage::read` offset across CRSF messages → config applied from wrong bytes | HIGH | FIXED LOCALLY | `test_crsf_decode_msp_resets_read_on_start` |

**Note:** upstream rtlopez/esp-fc `master` still contains ESPFC-001…005 (verified 2026-08-25). The detailed upstream-ready write-ups were removed from this file during cleanup but remain recoverable from git history (`git show b8b34bf:report.md`) should they be submitted upstream.

## Known open issues

| ID | Issue | Severity |
|---|---|---|
| ESPFC-006 | `TelemetryCRSF::sendMsp()` silently truncates MSP responses needing more than four CRSF chunks (>226 B v1 / >222 B v2 payload) | MEDIUM |
| ESPFC-007 | `MspResponse::write*` lack bounds checks against `data[240]` (latent; no current caller exceeds it) | LOW |
| ESPFC-008 | Most `MSP_SET_*` handlers consume payloads without `remain()` validation → stale-byte config values on short requests | LOW |
| ESPFC-009 | `bin/format-check.sh` flags vendored third-party sources (`lib/betaflight`, `lib/printf`, `lib/MultiButton`) — consider excluding them like `.pio/` | LOW |
| — | cppcheck reports pre-existing style/warning-level findings across drivers/sensors (101 E / 976 W / 12118 style at last run) | INFO |

## Historical notes

- Original baseline audit (2026-08-24): proot/Termux sandbox environment issues documented there were environment-only and no longer relevant to this repository's state; the original failing test (`test_crsf_encode_msp_v1_fragmented`) passes deterministically since ESPFC-001/002 were fixed.
- Rebranding: project renamed to DarkFlight (commit `7049c0c`); README carries origin/attribution and upstream relationship.
