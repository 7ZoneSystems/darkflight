# ESP-FC Baseline Report

Date: 2026-08-24
Updated: 2026-08-25 (post-audit follow-up — see *Audit Update* and *Upstream Bug Reports* below)
Branch: `main` (synced with `origin/main`, fork of rtlopez/esp-fc; `upstream` treated read-only)
Git state: `main`; audit fixes landed in local commit `f333d33` ("bugfix"). Nothing pushed.

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

## Development environment (ARM64-native, no emulation)

Host: aarch64, Debian 13 (trixie) inside proot-distro on Termux/Android, f2fs rootfs.

| Tool | Version | Source |
|---|---|---|
| PlatformIO Core | 6.1.19 | pip |
| cppcheck | 2.17.1 | apt |
| clang-format | 20.1.7 (matches repo pin; system had 21 from Termux) | pip |
| Host gcc/g++ | 14.2.0 | preinstalled |

No x86 emulation, Wine, or GUI dependencies introduced.

### Environment problems diagnosed and worked around

This sandbox is a proot/Termux-on-Android environment with two quirks that break
default PlatformIO operation. Both are handled by an out-of-repo wrapper
(`/tmp/opencode/pio_serial.py`) that:

1. Forces `multiprocessing.cpu_count() = 1`.
   - Symptom: with the default `-j8`, the host compiler crashed repeatedly
     (`cc1plus` ICE: "Bus error", "Segmentation fault", "futex facility returned
     an unexpected error code") while compiling large test TUs.
2. Replaces `shutil.copytree` with an `os.lstat()`-based implementation.
   - Symptom: every PlatformIO package install failed with
     `OSError: [Errno 22] Invalid argument` while copying toolchain files.
   - Root cause chain (proven empirically):
     proot's `getdents64` misreports hard-linked entries as `DT_LNK` ->
     Python `DirEntry.is_symlink()` returns true for regular files ->
     stdlib `copytree(symlinks=True)` calls `os.readlink()` on them -> EINVAL.
   - The wrapper decides entry types via `lstat()` instead of the lying d_type.

These are **environment-only** problems; they do not affect the ESP-FC codebase
and are listed here solely to explain log artifacts.

## Unit tests — `pio test -e native`

### Baseline (2026-08-24)

Result: **190 test cases — 188 passed, 1 failed**.

Suites fully green: `test_ahrs`, `test_esc`, `test_fc`, `test_gps`,
`test_gyro`, `test_math`, `test_msp`.

Failure: `test_input_crsf.test_crsf_encode_msp_v1_fragmented`
(`Expected 33 Was 46`). Investigation showed this is **not** a flaky or
environmental failure but collateral damage from a genuine memory-corruption bug
(described below). A SIGHUP at process exit was observed once (sandbox artifact);
all 14 tests in the suite did run to completion.

### Current (2026-08-25, after local fixes — commit `f333d33`)

Result: **194 test cases — 194 passed, 0 failed** (all 8 suites green).
Delta vs baseline: +5 new regression tests added by the audit (one per fixed
bug family; see *Upstream Bug Reports*). The originally failing
`test_crsf_encode_msp_v1_fragmented` passes deterministically now.

Note: because the original failure was undefined-behavior collateral damage,
its manifestation depends on compiler/stack layout (it happened to pass in one
re-run before the fix). The bug itself was always present; see ESPFC-001.

## Genuine repository bug found (baseline entry — now FIXED LOCALLY)

**`CrsfMessage::payload` buffer overflow on fragmented MSP-over-CRSF responses**
→ full write-up promoted to **ESPFC-001 / ESPFC-002** in the
*Upstream Bug Reports* section below.

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
  more conservative `CRSF_FRAME_SIZE_MAX`.
- Status 2026-08-25: **applied locally** (first variant), together with fixes
  for the receive-side manifestation of the same undersized buffer (ESPFC-002)
  and three MSP-decoder defects (ESPFC-003/004/005). See *Audit Update*.

## Audit Update — 2026-08-25

Scope: systematic memory-safety and flight-critical review of the CRSF/MSP
telemetry path, RC input parsers, MSP processor/parser, and spot checks of
control-path code. Findings are filed as structured reports in the
*Upstream Bug Reports* section (IDs `ESPFC-001` … `ESPFC-009`).

### Bugs fixed locally (commit `f333d33`, unpushed)

| ID | One-line summary | Files touched |
|---|---|---|
| ESPFC-001 | `CrsfMessage::payload` undersized → TX-side OOB write/read on max-size MSP-over-CRSF frames | `lib/Espfc/src/Rc/Crsf.h` |
| ESPFC-002 | `InputCRSF::parse()` OOB writes at raw indexes 62/63 when receiving any max-size (len=62) frame, before CRC validation | `lib/Espfc/src/Rc/Crsf.h` (shared root cause) |
| ESPFC-003 | `Crsf::decodeMsp()` continuation-path integer underflow (`size_t` wrap) defeats the `MSP_BUF_SIZE` guard → unbounded `append()` | `lib/Espfc/src/Rc/Crsf.cpp` |
| ESPFC-004 | `fillMessage()` underflow for short start-frames → up-to-65535-byte `append()` overflowing `MspMessage::buffer[192]` + massive OOB read | `lib/Espfc/src/Rc/Crsf.cpp` |
| ESPFC-005 | Stale `MspMessage::read` offset across messages on the CRSF path (serial path resets it; CRSF path did not) | `lib/Espfc/src/Rc/Crsf.cpp` |

Fix summary (exact diff in commit `f333d33`):

1. `Crsf.h`: `payload[CRSF_PAYLOAD_SIZE_MAX + 1]` (59 B) →
   `payload[CRSF_FRAME_SIZE_MAX - 3]` (61 B; exact worst-case fit so that
   `sizeof(CrsfMessage)` = 64 = full wire frame `<addr><len><type><payload><crc>`),
   plus `static_assert(sizeof(CrsfMessage) >= CRSF_FRAME_SIZE_MAX)` pinning the
   invariant at compile time. Resolves ESPFC-001 and ESPFC-002 (both stem from
   the same undersized struct).
2. `Crsf.cpp` `fillMessage()`: chunk length computed in signed arithmetic;
   return early if `frame.size - 5 - sizeof(HeaderType) < 0` (before reading the
   MSP header). Resolves ESPFC-004.
3. `Crsf.cpp` `decodeMsp()` start branch: added `m.read = 0;` alongside the
   existing `state/received/expected` resets, mirroring `MspParser`. Resolves
   ESPFC-005.
4. `Crsf.cpp` `decodeMsp()` continuation branch: signed intermediate +
   `framePayloadSize >= 0` guard ahead of the `MSP_BUF_SIZE` bound check
   (the old `size_t` wrap made `m.received + huge <= 192` pass when
   `received ≥ 3`). Resolves ESPFC-003.

RAM cost: +2 bytes per `CrsfMessage` instance. No API/behavior changes for
well-formed traffic (all pre-existing tests pass unchanged).

### Remaining unresolved repository issues

| ID | Summary | Severity |
|---|---|---|
| ESPFC-006 | `TelemetryCRSF::sendMsp()` silently truncates MSP responses needing more than 4 CRSF chunks | MEDIUM |
| ESPFC-007 | `MspResponse::writeU8/U16/U32/writeData` have no bounds check against `data[MSP_BUF_OUT_SIZE]` | LOW |
| ESPFC-008 | Most `MSP_SET_*` handlers consume payload without `remain()` validation → stale-buffer config values on short payloads (in-bounds, correctness/hardening) | LOW |
| ESPFC-009 | `bin/format-check.sh` scans `.pio/` build output → bogus formatting violations | LOW |

### Verified non-issues / false positives

- `toMspBoxId(MODE_ALTHOLD) { return MODE_ALTHOLD; }`
  (`Connect/MspProcessor.cpp`) looks wrong but is functionally identical to the
  `default:` passthrough — redundant, not a defect.
- SBUS/IBUS input parsers: state-machine index math re-verified bounded
  (`_data[SBUS_FRAME_SIZE-1]`, `_data[IBUS_FRAME_SIZE-1]` maxima respected).
- GPS/NMEA parsing: bounded copies and field caps verified
  (`GpsSensor.cpp:896`, `handleNmeaSentence`, `handleNavSvInfo`).
- Baseline items classified **environment-only** (proot): `cc1plus` ICE/futex,
  package-install `EINVAL`, SIGHUP/Errno 38 artifacts, ASan init failure.
  Not reported upstream as ESP-FC bugs.

## Static checks and formatting

- `pio run -t check_format` (**FAILS — genuine repo state, not environment**):
  3157 clang-format violations across all 178 files in `src/` + `lib/Espfc/src`.
  The codebase predates full `.clang-format` adoption (post "code-tidy" gap).
  Resolution belongs in a dedicated reformat commit
  (`pio run -e native -t format`).
- `bin/format-check.sh` flaw: it scans `.pio/` build output too (60 bogus hits
  among its 180 flags); it should exclude `.pio/` (and probably vendored
  `lib/betaflight`). Filed as ESPFC-009.
- `pio check` (cppcheck static analysis): not yet executed.

## Builds

- `esp32`: interrupted at user request ~90% through compilation (framework,
  libraries, and most of Espfc compiled; linking not reached). Bootloader and/
  partition binaries produced. Xtensa toolchain + Arduino framework installed OK.
  Post-fix firmware targets not rebuilt here; CI must validate (changes are
  plain C++11, ABI-safe, +2 bytes per `CrsfMessage`).
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
| `test_crsf_encode_msp_v1_fragmented` failure | genuine 2-byte OOB write in `CrsfMessage` | **repository bug** (fixed: ESPFC-001) |
| `check_format` target failure | codebase not formatted against `.clang-format` | repository state |

## Recommended next steps

1. ~~Apply the `CrsfMessage::payload` size fix~~ **DONE** — commit `f333d33`,
   incl. RX-side and MSP-underflow fixes (ESPFC-001…005); suite 194/194.
2. Build remaining targets one at a time as wall-time permits
   (`esp32s3`, `esp32c3`, `esp8266` first; RP2040/RP2350 pull an external
   GitHub platform and are experimental upstream anyway).
3. Run `pio check` (cppcheck) for the static-analysis gate.
4. Apply repo-wide clang-format as a dedicated commit
   (`pio run -e native -t format`), after agreeing scope with upstream.
5. Patch `bin/format-check.sh` to exclude `.pio/` (and consider excluding
   vendored `lib/betaflight`). Filed as ESPFC-009.
6. Optionally document the `-j1` requirement and proot caveats for this box
   (or use the provided `/tmp/opencode/pio_serial.py` wrapper).
7. Submit ESPFC-001…005 upstream (all reproducible against upstream `master`;
   verified 2026-08-25 that upstream `lib/Espfc/src/Rc/Crsf.h` still contains
   every unfixed defect described below). Submit ESPFC-006…009 for discussion.

---

# Upstream Bug Reports

Target project: rtlopez/esp-fc (ESP-FC).
All line references match this fork at commit `f333d33` unless stated otherwise;
upstream `master` verified 2026-08-25 to contain identical unfixed code for
every report.

---

## ESPFC-001

### Bug title
`CrsfMessage::payload` buffer overflow (2-byte OOB write + CRC OOB read) when encoding maximum-size fragmented MSP-over-CRSF responses

### Severity
CRITICAL

### Status
FIXED LOCALLY (commit `f333d33`, unpushed)

### Affected component
CRSF telemetry encoding — `lib/Espfc/src/Rc/Crsf.h` (`struct CrsfMessage`),
`lib/Espfc/src/Rc/Crsf.cpp` (`Crsf::encodeMspData`, `CrsfMessage::crc`,
`CrsfMessage::writeCRC`), invoked from `lib/Espfc/src/Telemetry/TelemetryCRSF.h`
(`TelemetryCRSF::sendMsp`, `TelemetryCRSF::send`).

### Summary
The transmit buffer inside `CrsfMessage` is too small for the largest frame the
firmware itself generates. Encoding a maximum-size MSP_RESP frame (57-byte MSP
chunk + 3 extended-header bytes + CRC) writes 2 bytes past the end of the
struct, and computing the CRC reads 1 byte past the array. The subsequent
serial write (`send()`) additionally copies 2 bytes past the object.

### Technical root cause
Wire frame layout: `<addr><len><type><payload...><crc>`, where the `len` field
counts `type + payload + crc`. Constants:
`CRSF_FRAME_SIZE_MAX = 64`, `CRSF_PAYLOAD_SIZE_MAX = 64 - 6 = 58`.

`CrsfMessage` declared `uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]` — 59 bytes
(valid indexes 0..58), sized as "58 payload + 1 crc". That accounting ignores
that MSP-over-CRSF consumes 3 of the 58 payload bytes for its extended header
(`dst`, `origin`, `status`), leaving at most 55 bytes for the MSP chunk if the
CRC is to stay in bounds. However:

- `encodeMspData()` fragments at `CRSF_PAYLOAD_SIZE_MAX - 1 = 57` bytes/chunk
  (correct for the 64-byte wire limit: 3 ext + 57 chunk + crc ⇒ wire len 62),
  so a full chunk occupies payload indexes 0..59 and stores the CRC via
  `writeCRC()` at `payload[size - 2] = payload[60]` — **indexes 59 and 60 are
  out of bounds for a 59-byte array** (2-byte OOB write).
- `crc()` sums `payload[0 .. size-3] = payload[0..59]` — index 59 is an
  **OOB read**.
- `sizeof(CrsfMessage)` was 62 bytes, while `send()` transmits
  `msg.size + 2 = 64` bytes starting at the object — a 2-byte **OOB read**
  feeding the UART/DMA.

Because the struct is `__packed__` with the array last, the overflow lands
directly in whatever follows the object on the stack (in the failing unit test,
the adjacent `seq` variable; on-device, arbitrary adjacent RAM).

### Reproduction
Standalone canary program compiled against the repo headers (pre-fix), packing
guard bytes directly after a `CrsfMessage` and calling
`Crsf::encodeMspData(msg, CRSF_ADDRESS_RADIO_TRANSMITTER, 1, 0, true, body, body+66)`
with ≥ 57 body bytes; or simply `pio test -e native -f test_input_crsf` on a
compiler/ABI where the OOB write hits the adjacent `seq` variable.

### Observed behavior
Pre-fix canary output (gcc 13.3, x86-64, sources at `8c741df`):

```
CRSF_PAYLOAD_SIZE_MAX=58  payload array=59  sizeof(CrsfMessage)=62
TX: frame.size=62; CRC slot=payload[60]; crc() reads payload[0..59]; array valid idx 0..58
TX: GUARDS CLOBBERED: guard_a=0xA8 (want 0xAA) guard_b=0xC7 (want 0xBB)
TX: send() would read 64 bytes from 62-byte object -> OOB READ
RESULT: BUG PRESENT
```

In the original unit-test environment the overflow corrupted the adjacent
stack variable `seq`, producing the baseline failure
`test_crsf_encode_msp_v1_fragmented … Expected 33 Was 46` (status byte seen as
`0x2E` instead of `0x21`). In another environment the same code passed the test
while still performing the OOB accesses — classic layout-dependent UB.

### Expected behavior
Encoding the largest frame the protocol allows must touch only bytes owned by
the struct: payload capacity ≥ `size_max - 2` = 60 data+CRC slots, i.e.
`sizeof(CrsfMessage) ≥ CRSF_FRAME_SIZE_MAX`.

### Evidence
- Canaries above (guards clobbered with the chunk's last data byte and the CRC).
- Source-level proof: `Crsf.h` (old) line 120
  `uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1];` vs required ≥ 61 bytes;
  `Crsf.cpp:95` fragmenting at `CRSF_PAYLOAD_SIZE_MAX - 1`.
- Baseline unit-test failure `Expected 33 Was 46` (report baseline, 2026-08-24).
- Upstream `master` `lib/Espfc/src/Rc/Crsf.h` (fetched 2026-08-25) still
  declares `payload[CRSF_PAYLOAD_SIZE_MAX + 1]` — unfixed upstream.

### Impact
Undefined behavior / adjacent RAM corruption on every fragmented MSP response
over CRSF (e.g., Betaflight Configurator reading parameter groups). Corrupted
telemetry buffers, stack variables, or scheduler objects; nondeterministic,
hard-to-diagnose field failures. Flight-critical because MSP/CRSF runs on the
same radio link used for control with ELRS/Pioneer stacks.

### Fix status
FIXED LOCALLY (commit `f333d33`):

```diff
-  uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for crc
+  // worst case frame: addr + len + type + [ext header (dst, origin, status)] + max payload chunk + crc
+  // = CRSF_FRAME_SIZE_MAX bytes total, thus payload capacity is CRSF_FRAME_SIZE_MAX - 3
+  uint8_t payload[CRSF_FRAME_SIZE_MAX - 3];
...
+// the whole struct must be able to hold a full-size wire frame (<addr><len><type><payload...><crc>)
+static_assert(sizeof(CrsfMessage) >= CRSF_FRAME_SIZE_MAX, "CrsfMessage too small for max size CRSF frame");
```

61 bytes is the exact worst case: encoder-produced max `size` = 62 ⇒ highest
touched index `size - 2` = 60; `sizeof(CrsfMessage)` becomes 64, which also
makes `send()`'s `size + 2` copy exactly object-sized and covers the RX-side
writes of ESPFC-002. The `static_assert` prevents silent regression. Behavior
for well-formed traffic is unchanged (all pre-existing tests pass).

Regression result: `pio test -e native` → **194/194 pass**, including the new
`test_crsf_encode_msp_v1_fragmented_no_overflow`.

### Regression coverage
`test/test_input_crsf/test_input_crsf.cpp ::
test_crsf_encode_msp_v1_fragmented_no_overflow` — packs guard bytes after the
message, encodes a 66-byte MSPv1 response, asserts `frame.size == 62`,
CRC `0xFE`, and both guards untouched. Fails on unfixed code on any ABI
(guards are struct members, not layout luck). The pre-existing
`test_crsf_encode_msp_v1_fragmented` remains as functional coverage.

### Upstream recommendation
Apply the same change (or the more conservative `payload[CRSF_FRAME_SIZE_MAX]`)
plus the `static_assert`. Consider also asserting `size - 2 < capacity` inside
`writeCRC()` in debug builds.

---

## ESPFC-002

### Bug title
`InputCRSF::parse()` writes out of bounds (raw indexes 62–63) when receiving any maximum-size CRSF frame, before CRC validation

### Severity
CRITICAL

### Status
FIXED LOCALLY (commit `f333d33`; shares root cause and fix with ESPFC-001)

### Affected component
CRSF receiver input path — `lib/Espfc/src/Device/InputCRSF.cpp`
(`InputCRSF::parse`, states `CRSF_DATA`/`CRSF_CRC`),
`lib/Espfc/src/Device/InputCRSF.h` (member layout), root cause in
`lib/Espfc/src/Rc/Crsf.h` (`CrsfMessage` size).

### Summary
The byte-wise CRSF parser writes each received byte at raw index `_idx++` of
the `CrsfMessage` object. For an inbound frame with `len = 62` (the maximum the
parser accepts), the DATA state writes raw index 62 and the CRC state writes
raw index 63 — both past the end of the 62-byte `CrsfMessage` object —
*before* the frame's CRC has been validated.

### Technical root cause
Parser accepts `len ∈ [2, CRSF_FRAME_SIZE_MAX - 2] = [2, 62]` (InputCRSF.cpp:85).
Index trace for `len = 62`: ADDR writes idx 0 (`_idx=1`), SIZE idx 1 (`_idx=2`),
TYPE idx 2 (`_idx=3`); DATA writes idx 3..62 (stays in DATA while
post-increment `_idx <= len`), CRC state writes idx 63. Required object size is
therefore `len_max + 2 = 64` bytes; the pre-fix packed struct was
`3 + 59 = 62` bytes → 2-byte overflow on every maximum-size frame.

Member order in `InputCRSF` places `_frame` immediately before
`uint16_t _channels[16]` (InputCRSF.h:52-53): the stray bytes land in padding
or directly in live RC channel data.

Crucially, the corruption happens during reception — the CRC check runs only
after the whole frame has been consumed — so even frames that fail CRC
(e.g., bit errors, or deliberately malformed traffic) overwrite memory first.

### Reproduction
Feed a 64-byte stream `{0xC8, 62, <type>, 61 arbitrary payload bytes, <any byte>}`
byte-by-byte into `InputCRSF::parse(frame, b)` where `frame` is a guarded
`CrsfMessage` (or member of an `InputCRSF`). Covered directly by the regression
test below.

### Observed behavior
Canary simulation of the exact parser index arithmetic (pre-fix struct):

```
RX: size=62 frame -> DATA writes raw idx up to 62, CRC state writes idx 63; object valid idx 0..61
RX: CONFIRMED OOB WRITE during parse of max-size frame
```

With the regression test applied to pre-fix sources, the suite aborts with
memory corruption (SIGTRAP) instead of completing.

### Expected behavior
Receiving any syntactically acceptable frame (including ones that later fail
CRC) must never write outside the message object.

### Evidence
- Index arithmetic quoted above is directly from `InputCRSF.cpp:106-121`.
- Fresh pre-fix canary run (sources `8c741df`): `BUG PRESENT`, exit 1.
- Post-fix run (commit `f333d33`): `object valid idx 0..63`,
  `RESULT: no OOB detected`, exit 0.

### Impact
Radio-link-reachable memory corruption on the flight controller: a corrupted
or hostile max-size frame (types RC_CHANNELS_PACKED / LINK_STATISTICS /
MSP_REQ / MSP_WRITE are accepted by the TYPE filter) can scramble the
immediately adjacent `_channels[]` array — i.e., live RC inputs — or other
adjacent RAM, potentially mid-flight. Severity driven by reachability (plain
UART bytes) and pre-validation timing.

### Fix status
FIXED LOCALLY by the ESPFC-001 buffer enlargement: `sizeof(CrsfMessage)` is now
64, so DATA (max raw idx 62) and CRC (max raw idx 63) writes are in bounds for
every accepted `len`. Parser logic intentionally left unchanged (minimal,
reviewable change).

Regression result: `pio test -e native` → 194/194 pass.

### Regression coverage
`test/test_input_crsf/test_input_crsf.cpp ::
test_input_crsf_max_size_frame_parse_no_overflow` — builds a valid 64-byte
max-size MSP_REQ frame, feeds it through the real `InputCRSF::parse()`, asserts
guard bytes intact and frame contents/CRC preserved.

### Upstream recommendation
Same one-line buffer fix as ESPFC-001 (plus `static_assert`). Optionally add a
debug-mode `_idx` bounds assert in `parse()`.

---

## ESPFC-003

### Bug title
`Crsf::decodeMsp()` continuation-path integer underflow (`size_t` wraparound) bypasses the `MSP_BUF_SIZE` guard, causing an unbounded `append()` (memory corruption)

### Severity
CRITICAL

### Status
FIXED LOCALLY (commit `f333d33`)

### Affected component
MSP-over-CRSF receive path — `lib/Espfc/src/Rc/Crsf.cpp`
(`Crsf::decodeMsp`, continuation branch; pre-fix lines ~190-201),
`lib/Espfc/src/Connect/Msp.cpp` (`MspMessage::append`).

### Summary
For a non-start MSP chunk carried in a CRSF frame whose `len` field is smaller
than 5, the chunk-length expression underflows; storing it in `size_t` produces
a near-2^64 length that then wraps back under the `MSP_BUF_SIZE` bound check,
so `append()` performs an effectively unbounded `std::copy` from the frame
buffer into `MspMessage::buffer[192]`.

### Technical root cause
Pre-fix code:

```cpp
size_t framePayloadSize = std::min(frame.size - 5, m.expected - m.received);
if(m.received + framePayloadSize <= Connect::MSP_BUF_SIZE)
{
  m.append(frame.payload + 3, framePayloadSize);
```

`frame.size` is `uint8_t`; `frame.size - 5` promotes to `int` and is negative
whenever `frame.size < 5` (parser accepts sizes down to 2). `std::min` yields
that negative `int`, which converts to `size_t` ≈ 2^64−k. The guard evaluates
`(m.received + huge) mod 2^64`, which equals `m.received − k` and passes
whenever `m.received ≥ k`. `append()` then executes
`std::copy(data, data + 2^64−k, buffer + received)` — an unbounded read of
stack memory and unbounded write through `buffer`.

Reachability: `decodeMsp()` is called for any CRC-valid `MSP_REQ`/`MSP_WRITE`
frame (InputCRSF.cpp:167-181). An attacker controls both frames of the
sequence: (1) a start frame establishing `expected > received ≥ 3`, then
(2) a short continuation frame (`len < 5`, start bit clear, matching sequence
number). CRSF CRC8 provides only ~1/256 random-acceptance protection for
corrupt traffic and none against deliberate crafting.

### Reproduction
Unit-level (regression test): decode a valid start frame announcing
`expected = 200` with 10 payload bytes (`m.received == 10`), then decode a hand-built continuation frame with `size = 4` and matching sequence. On pre-fix
code the second call corrupts state/crashes; post-fix it is ignored.

### Observed behavior
Pre-fix run of `test_crsf_decode_msp_short_continuation_frame_ignored`
(sources `8c741df`, current test file):

```
test_crsf_decode_msp_short_continuation_frame_ignored: Expected 10 Was 9	[FAILED]
...
Program received signal SIGTRAP (Trace/breakpoint trap)
============ 19 test cases: 5 failed, 13 succeeded ============
```

`m.received` mutated 10 → 9 by the wrapped `append()` bookkeeping — direct
observable proof of the wrap-around path executing.

### Expected behavior
A continuation frame shorter than the extended header must contribute nothing
and leave the partially assembled message intact.

### Evidence
Test output above; source-level analysis in *Technical root cause*
(pre-fix `Crsf.cpp:193-196`).

### Impact
Remote memory corruption from the control link: unbounded stack read +
unbounded write through a 192-byte buffer embedded in `InputCRSF` (last
member), i.e., smash of adjacent RAM/heap. Requires crafted packets, but CRSF
links are routinely exposed to third-party traffic (relay/groundstation stacks),
and random corruption needs only a 1-in-256 CRC coincidence per frame pair.

### Fix status
FIXED LOCALLY (commit `f333d33`):

```diff
-      size_t framePayloadSize = std::min(frame.size - 5, m.expected - m.received);
-      if(m.received + framePayloadSize <= Connect::MSP_BUF_SIZE)
+      const int framePayloadSize = std::min(frame.size - 5, (int)m.expected - (int)m.received);
+      if(framePayloadSize >= 0 && m.received + framePayloadSize <= Connect::MSP_BUF_SIZE)
```

Signed intermediate keeps the true (possibly negative) length; the explicit
`>= 0` check rejects impossible chunks before the bound check can be defeated
by modular wrap. Zero-length chunks retain legacy behavior (harmless
`append(ptr, 0)` and completion re-check).

Regression result: 194/194 pass; the targeted test fails on pre-fix sources
(shown above) and passes post-fix.

### Regression coverage
`test_crsf_decode_msp_short_continuation_frame_ignored` (see Reproduction).

### Upstream recommendation
Apply the two-line fix. A broader hardening would validate `frame.size >= 5`
once at the top of `decodeMsp()` and reject such frames entirely.

---

## ESPFC-004

### Bug title
`fillMessage()` integer underflow on short MSP start-frames → `MspMessage::buffer[192]` overflow (up to 65535-byte append) plus large OOB read

### Severity
CRITICAL

### Status
FIXED LOCALLY (commit `f333d33`)

### Affected component
MSP-over-CRSF receive path — `lib/Espfc/src/Rc/Crsf.cpp` (static
`fillMessage<HeaderType>()`, pre-fix lines 112-128), used by
`Crsf::decodeMsp()` for MSPv1/MSPv2 start frames.

### Summary
When a start-flagged MSP-over-CRSF frame is too short to contain the full MSP
header (V1: `len < 7`, V2: `len < 9`), the available-payload computation
underflows. Combined with the attacker-controlled declared MSP size
(`hdr->size`, up to 65535 for MSPv2), `std::min()` selects that declared size
and `append()` copies that many bytes — from beyond the frame object into the
192-byte message buffer.

### Technical root cause
Pre-fix code:

```cpp
const auto * hdr = reinterpret_cast<const HeaderType*>(frame.payload + 3);
const size_t framePayloadSize = frame.size - 5 - sizeof(HeaderType);   // negative → huge
...
m.append(frame.payload + 3 + sizeof(HeaderType),
         std::min(framePayloadSize, (size_t)hdr->size));
```

For `frame.size < 5 + sizeof(HeaderType)` the signed intermediate is negative
and becomes ≈2^64 as `size_t`; `min(huge, hdr->size)` returns `hdr->size`
(uint8 ≤ 255 for V1, uint16 ≤ 65535 for V2). Two consequences in one call:

1. `m.buffer[192]` receives up to `hdr->size` bytes — overflow by up to
   65343 bytes (`buffer` is the last member of `MspMessage`, itself the last
   member of `InputCRSF`, so writes run past the entire receiver object).
2. The source pointer walks far past the 62-byte frame object — a large OOB
   read.

Header bytes are attacker-controlled even for short frames: the parser does
not clear `payload[]` between frames, so a preceding full-size frame seeds
`payload[3..]` with chosen values, and a short frame with `len ≤ 3` has a
trivially constant CRC (`crc8_dvb_s2(type)` only). Single crafted frame ⇒
deterministic corruption.

### Reproduction
Regression test: build a start-flagged MSPv2 REQ frame with `len = 6` (only one
byte after the extended header) and call `Crsf::decodeMsp()`. Pre-fix, the
message can become spuriously READY with corrupted counters; with seeded
payload bytes and larger declared sizes it performs the oversized copy.

### Observed behavior
Pre-fix run of `test_crsf_decode_msp_short_start_frame_ignored`
(sources `8c741df`):

```
test_crsf_decode_msp_short_start_frame_ignored: Expected 0 Was 1	[FAILED]
```

i.e., the malformed 6-byte frame was accepted as a complete MSP message
(state = RECEIVED) instead of being rejected.

### Expected behavior
A start frame shorter than `5 + sizeof(MSP header)` must be rejected without
touching `hdr` or appending anything.

### Evidence
Test line above; fresh full pre-fix suite run:
`19 test cases: 5 failed, 13 succeeded` + SIGTRAP abort. Post-fix: passes.

### Impact
Single-frame, radio-reachable heap/stack corruption with controlled length and
content — the most severe class of defect in this audit. Practical trigger
surfaces include Betaflight Configurator passthrough tooling and any host that
emits MSP-over-CRSF requests toward the FC.

### Fix status
FIXED LOCALLY (commit `f333d33`):

```diff
+  const int framePayloadSize = frame.size - 5 - sizeof(HeaderType);
+  if(framePayloadSize < 0) return; // frame too short to contain full msp header
   const auto * hdr = reinterpret_cast<const HeaderType*>(frame.payload + 3);
...
-  m.append(..., std::min(framePayloadSize, (size_t)hdr->size));
+  m.append(..., std::min((size_t)framePayloadSize, (size_t)hdr->size));
```

Early return precedes the header dereference, so neither `hdr->cmd` nor
`hdr->size` is read from a truncated frame. Valid empty-payload commands
(`framePayloadSize == 0`, e.g. V1 `len = 7`) keep working: `append(ptr, 0)` is
skipped-equivalent and the `expected == received` completion still fires,
matching prior behavior (`test_crsf_decode_msp_v1` unchanged and passing).

Regression result: 194/194 pass.

### Regression coverage
`test_crsf_decode_msp_short_start_frame_ignored` (asserts return 0,
not-ready, `received == 0`). A stronger variant seeding `payload[3..]` with a
large declared V2 size and asserting `received` stays 0 would tighten coverage.

### Upstream recommendation
Apply the fix; consider additionally clamping `expected` to `MSP_BUF_SIZE` in
`fillMessage()` so oversized declared lengths cannot wedge the fragmentation
state machine (they currently just never complete — DoS-noop).

---

## ESPFC-005

### Bug title
`MspMessage::read` offset not reset between messages on the MSP-over-CRSF path → commands processed from stale payload offsets

### Severity
HIGH

### Status
FIXED LOCALLY (commit `f333d33`)

### Affected component
MSP-over-CRSF receive path — `lib/Espfc/src/Rc/Crsf.cpp`
(`Crsf::decodeMsp`, start branch), consumer
`lib/Espfc/src/Connect/MspProcessor.cpp` (`processCommand` readers).
Reference behavior: `lib/Espfc/src/Connect/MspParser.cpp:17`
(`msg.read = 0;` on the serial path).

### Summary
`InputCRSF` keeps a single persistent `MspMessage _msg`. The serial MSP parser
resets `read` when a new frame begins, but the CRSF decoder's start branch
reset only `state/received/expected`. Consequently, the second and subsequent
MSP commands received over CRSF are parsed by `processCommand()` beginning at
the *previous* command's consumed offset.

### Technical root cause
`MspProcessor::processCommand()` consumes request payloads via
`m.readU8()/readU16()/readU32()` which advance `m.read` without bounds or
reset. After command A (say a `MSP_SET_*` with 40 payload bytes) completes,
`_msg.read == 40` persists. When command B arrives, `decodeMsp()`'s start
branch refills `buffer[0..N)` and sets `received = N`, but leaves
`read == 40`; handlers therefore read B's "payload" from stale bytes of A
(or uninitialized tail), and each unchecked `readU8()` can walk `read`
arbitrarily far, eventually past `buffer[192]` into adjacent memory
(in-bounds reads of stale config data occur much earlier).

### Reproduction
Regression test: decode a complete 4-payload-byte command, set
`m.read = m.received` (simulating processor consumption), then decode a second
3-payload-byte command and assert `m.read == 0` afterwards.

### Observed behavior
Pre-fix run of `test_crsf_decode_msp_resets_read_on_start`
(sources `8c741df`):

```
test_crsf_decode_msp_resets_read_on_start: Expected 0 Was 4	[FAILED]
```

### Expected behavior
Each newly assembled MSP message must expose `read == 0` to its consumer,
identically to the serial (`MspParser`) path.

### Evidence
Test output above. Structural proof: compare
`MspParser.cpp:17-20` (resets `read/received/checksum`) with the start branch
of `decodeMsp()` (pre-fix reset only `state/received/expected`).

### Impact
Persistent configuration corruption via MSP/CRSF: e.g., after any SET command,
subsequent `MSP_SET_MODE_RANGE` / `MSP_SET_PID` / `MSP_SET_RX_MAP` style
commands silently apply values assembled from stale bytes — mode ranges and
arming-related settings among them. Also enables `read` to advance beyond the
buffer (memory-unsafe reads). Deterministic and exploitable by any MSP client
on the link; no CRC games needed.

### Fix status
FIXED LOCALLY (commit `f333d33`): added `m.read = 0;` to the start branch of
`decodeMsp()`, mirroring `MspParser` semantics.

```diff
     m.state = Connect::MSP_STATE_IDLE;
     m.received = 0;
     m.expected = 0;
+    m.read = 0;
```

Continuation chunks never mark a *new* message ready without a preceding start
frame, so resetting in the start branch is sufficient.

Regression result: 194/194 pass.

### Regression coverage
`test_crsf_decode_msp_resets_read_on_start` (two sequential commands; asserts
`cmd`, `received`, and `read == 0` after the second decode).

### Upstream recommendation
Apply the one-line fix. Longer term, consider centralizing "message complete"
bookkeeping so serial and CRSF paths cannot drift apart again.

---

## ESPFC-006

### Bug title
`TelemetryCRSF::sendMsp()` silently truncates MSP responses that require more than four CRSF fragments

### Severity
MEDIUM

### Status
NEEDS UPSTREAM REVIEW (not modified)

### Affected component
`lib/Espfc/src/Telemetry/TelemetryCRSF.h` — `TelemetryCRSF::sendMsp`
(loop condition `while(beg != end && iter < 4);`, line 91).

### Summary
MSP responses are split into CRSF frames carrying at most
`CRSF_PAYLOAD_SIZE_MAX - 1 = 57` body bytes each. `sendMsp()` stops after four
iterations regardless of remaining data, so any response whose serialized body
exceeds 4 × 57 = 228 bytes (MSPv1 payload > 226 bytes, MSPv2 payload > 222
bytes) is transmitted incomplete: no error is raised and the trailing frames
never arrive, leaving the client waiting until its MSP timeout.

### Technical root cause
`MSP_BUF_OUT_SIZE = 240` permits building responses up to 240 data bytes, but
four 57-byte chunks can carry at most 228 body bytes (plus the v1
len/cmd or v2 flags/cmd/len prefix bytes that share the same stream).
The `iter < 4` cap appears to be an anti-runaway guard rather than a
protocol-derived limit; betaflight's `msp_shared.c` loops until completion.

### Reproduction
Construct an `MspResponse` with 230 data bytes (v1), serialize, and step
through `sendMsp()` with a counting serial fake: after 4 calls `beg != end`
remains true and ~13 bytes are undelivered. No current built-in handler emits
such responses (largest audited: `MSP_DATAFLASH_READ`, sized to fit by design),
so on-device triggering requires future handlers or external modification —
this is a latent protocol-correctness defect, hence MEDIUM, not HIGH.

### Observed behavior
Silent truncation (loop exits early, return value 4).

### Expected behavior
Either deliver the full response (cap derived from actual size, e.g.
`iter < ceil(region_len / 57)`) or fail loudly (`result = -1`) when a response
cannot fit within the transport policy.

### Evidence
Code inspection of `TelemetryCRSF.h:78-94` and `Connect/Msp.hpp`
(`MSP_BUF_OUT_SIZE = 240`); chunk arithmetic above. Not exercised on device.

### Impact
Protocol hang/timeouts for Configurator-style clients if a handler ever grows
past the cap; no memory-safety impact today.

### Fix status
Unchanged (behavioral decision belongs to the maintainer).

Minimal remediation direction (not implemented): replace the constant cap with
a computed upper bound, or clamp `MspResponse` construction to what four chunks
can carry and reject overflow at write time (which would also cover ESPFC-007).

### Regression coverage
Suggested test: 240-byte `MspResponse` → assert either full delivery across
⌈242/57⌉ = 5 frames or explicit failure; currently no test covers multi-chunk
upper-bound behavior.

### Upstream recommendation
Decide intended policy (complete delivery vs hard cap) and document it;
prefer completing delivery to match betaflight `msp_shared.c` semantics.

---

## ESPFC-007

### Bug title
`MspResponse::writeU8/U16/U32/writeData` perform unchecked writes into `data[MSP_BUF_OUT_SIZE]`

### Severity
LOW (latent; no current caller can exceed the buffer)

### Status
UNFIXED

### Affected component
`lib/Espfc/src/Connect/Msp.cpp` (`writeData/writeU8/writeU16/writeU32`,
`data[len++]` at line 95), `lib/Espfc/src/Connect/Msp.hpp`
(`uint8_t data[MSP_BUF_OUT_SIZE]`, `MSP_BUF_OUT_SIZE = 240`).

### Summary
Response builders increment `len` without comparing against
`MSP_BUF_OUT_SIZE`. Only the final `serialize()` guards its *destination*
buffer; the `MspResponse::data[]` array itself can be overrun by any handler
that writes more than 240 bytes.

### Technical root cause
`void MspResponse::writeU8(uint8_t v) { data[len++] = v; }` — no bound check.
Audited handlers write at most ~100 bytes (largest: `MSP_SERVO_CONFIGURATIONS`
8×12 = 96; `MSP_BOARD_INFO` ~50), so the defect is not currently triggerable —
it is a missing invariant, not an active vulnerability.

### Reproduction
Not reachable through the public command set; demonstrated by inspection only.
(A unit test writing >240 bytes via the public API would show the overflow.)

### Observed behavior / Expected behavior
Expected: writers clamp or assert at `remain() == 0` (note `remain()` exists
but is only consulted by `serializeFlashData`).

### Evidence
Source lines cited above; grep of all `r.write*` call sites in
`MspProcessor.cpp` confirms current maximum < 240.

### Impact
Future handlers (e.g., richer board-info strings, vtx-table support) could
silently corrupt adjacent RAM. Hardening issue.

### Fix status
Unfixed. Minimal remediation direction (not implemented): make `writeU8`
bound-checked (`if(len < MSP_BUF_OUT_SIZE) data[len++] = v;`) or assert, and
have handlers honor `result = -1` on truncation.

### Regression coverage
Suggested: unit test that fills a response beyond capacity and asserts no
out-of-bounds store (would require the bound check to pass).

### Upstream recommendation
Add the bounds check; it is two lines and removes an entire class of future
bugs.

---

## ESPFC-008

### Bug title
Most `MSP_SET_*` handlers consume request payloads without `remain()` validation, applying stale-buffer bytes as configuration

### Severity
LOW (correctness/hardening; in-bounds)

### Status
NEEDS UPSTREAM REVIEW

### Affected component
`lib/Espfc/src/Connect/MspProcessor.cpp` — e.g. `MSP_SET_MODE_RANGE`
(line 402, reads 4+2 bytes unconditionally), `MSP_SET_RX_MAP` (line 854, always
reads 8), `MSP_SET_PID` (line 1310, always reads 30), `MSP_SET_MIXER_CONFIG`,
`MSP_SET_SENSOR_CONFIG`, and similar; underlying `MspMessage::readU8()` has no
bounds check (`Connect/Msp.cpp:36`).

### Summary
Handlers assume the declared payload length is always present. For short or
zero-length requests (which parse successfully), reads fall off the end of
`received` and return leftover bytes from the previously received message
stored in the same buffer. The resulting garbage is written into live
configuration (mode ranges, PID terms, channel maps…) without error.

### Technical root cause
`MspMessage` buffers persist across requests; nothing zeroes `buffer` between
messages and `readU8()` returns `buffer[read++]` unchecked. Handlers lack the
`if(m.remain() >= N)` pattern already used elsewhere in the same file (e.g.,
`MSP_SET_CF_SERIAL_CONFIG`). With ESPFC-005 fixed, `read` starts at 0 per
message, so accesses remain inside `buffer[192]` for all current handlers
(maximum unconditional consumption audited: 30 bytes) — hence LOW/in-bounds —
but the semantic bug (garbage config accepted) is real today.

### Reproduction
Send a `MSP_SET_RX_MAP` command with `expected = 0` payload after any longer
SET command on the same transport: the new map is taken from the previous
message's tail. Observable via a following `MSP_RX_MAP` read.

### Observed behavior
Configuration fields updated with stale bytes; no `result = -1`.

### Expected behavior
Undersized SET payloads should be rejected (`r.result = -1`) without mutating
config, consistent with the handlers that do check.

### Evidence
Code paths cited; consistent with betaflight's MSP handling which validates
incoming frame sizes per command.

### Impact
Misconfiguration via buggy or hostile MSP clients (e.g., mode-range/arming
settings), silent rather than failing. No direct memory unsafety after
ESPFC-005.

### Fix status
Unfixed. Minimal remediation direction (not implemented): add `remain()`
guards per handler mirroring the existing patterns; mechanical, low-risk.

### Regression coverage
Suggested: per-command unit tests issuing truncated SET payloads and asserting
`result == -1` and unchanged config.

### Upstream review ask
Whether to enforce strict length checking per command (behavior change some
clients might rely on) or clamp-and-ignore. Maintainer's call.

---

## ESPFC-009

### Bug title
`bin/format-check.sh` counts formatting violations inside `.pio/` build output, reporting dozens of bogus findings

### Severity
LOW (developer tooling)

### Status
UNFIXED

### Affected component
`bin/format-check.sh`.

### Summary
The script's file enumeration does not exclude the PlatformIO build directory,
so generated/copied sources under `.pio/` are checked as if they were tracked
sources.

### Technical root cause
Baseline measurement: of 180 reported violations, 60 originate under `.pio/`
(duplicated third-party/generated files produced during builds). Vendored
`lib/betaflight` headers arguably should also be excluded from this project's
format contract.

### Reproduction
Run a build, then `bin/format-check.sh`: violation count grows by the `.pio/`
artifacts compared to a clean tree.

### Observed behavior / Expected behavior
Only repository-tracked, first-party sources should be evaluated.

### Evidence
Baseline report measurement (2026-08-24), reproduced numbers above.

### Impact
False CI/local failures; noise discourages adoption of the format gate.

### Fix status
Unfixed. Minimal remediation direction (not implemented): add
`--exclude .pio` (and consider `lib/betaflight`) to the clang-format invocations
inside the script.

### Regression coverage
n/a (script self-check: run after a build and assert zero `.pio/` paths in
output).

### Upstream recommendation
Exclude build output directories in the script.

---

## Non-issues examined (documented to prevent duplicate reports)

| Item | Verdict |
|---|---|
| `toMspBoxId(MODE_ALTHOLD)` returning `MODE_ALTHOLD` (MspProcessor.cpp) | Redundant duplicate of `default:` passthrough — harmless |
| `InputSBUS`/`InputIBUS` parser index arithmetic | Verified bounded to `*_FRAME_SIZE - 1`; no defect |
| GPS/NMEA coordinate & sentence parsing (`GpsSensor.cpp`) | Bounded copies (`min(degLen, sizeof(degBuf)-1)`), capped field count (20), `numCh` clamped to `SAT_MAX` (32) — no defect |
| `MspParser` V1/V2 size rejection (`> MSP_BUF_SIZE`) | Correctly bounds `expected` ≤ 192 on the serial path |
| Baseline sandbox artifacts (proot ICEs, EINVAL installs, SIGHUP, ASan init) | Environment-only; excluded from upstream reports per scope |

---

# Bug Report Index

| ID | Bug | Severity | Component | Status | Regression Test |
|----|-----|----------|-----------|--------|-----------------|
| ESPFC-001 | `CrsfMessage::payload` 2-byte OOB write + CRC/`send()` OOB reads on max-size MSP-over-CRSF encode | CRITICAL | `Rc/Crsf.h`, `Rc/Crsf.cpp`, `Telemetry/TelemetryCRSF.h` | FIXED LOCALLY (`f333d33`) | `test_crsf_encode_msp_v1_fragmented_no_overflow` |
| ESPFC-002 | `InputCRSF::parse()` OOB writes (raw idx 62/63) on any inbound len=62 frame, pre-CRC-validation | CRITICAL | `Device/InputCRSF.*`, root cause `Rc/Crsf.h` | FIXED LOCALLY (`f333d33`) | `test_input_crsf_max_size_frame_parse_no_overflow` |
| ESPFC-003 | `decodeMsp()` continuation `size_t` underflow defeats `MSP_BUF_SIZE` guard → unbounded `append()` | CRITICAL | `Rc/Crsf.cpp` (`decodeMsp`) | FIXED LOCALLY (`f333d33`) | `test_crsf_decode_msp_short_continuation_frame_ignored` |
| ESPFC-004 | `fillMessage()` underflow on short start-frames → `MspMessage::buffer[192]` overflow + large OOB read | CRITICAL | `Rc/Crsf.cpp` (`fillMessage`) | FIXED LOCALLY (`f333d33`) | `test_crsf_decode_msp_short_start_frame_ignored` |
| ESPFC-005 | Stale `MspMessage::read` offset across messages on CRSF path → config applied from wrong bytes | HIGH | `Rc/Crsf.cpp` (`decodeMsp`) vs `Connect/MspParser.cpp` | FIXED LOCALLY (`f333d33`) | `test_crsf_decode_msp_resets_read_on_start` |
| ESPFC-006 | `sendMsp()` truncates MSP responses needing >4 CRSF chunks | MEDIUM | `Telemetry/TelemetryCRSF.h` | NEEDS UPSTREAM REVIEW | suggested (none yet) |
| ESPFC-007 | `MspResponse::write*` unchecked vs `data[240]` | LOW | `Connect/Msp.cpp` | UNFIXED (latent) | suggested (none yet) |
| ESPFC-008 | `MSP_SET_*` handlers lack `remain()` validation → stale-bytes config writes | LOW | `Connect/MspProcessor.cpp` | NEEDS UPSTREAM REVIEW | suggested (none yet) |
| ESPFC-009 | `bin/format-check.sh` scans `.pio/` build output | LOW | `bin/format-check.sh` | UNFIXED | script self-check suggested |

Current verification state (2026-08-25, this fork @ `f333d33`):
`pio test -e native` → **194/194 passed**; canary reproduction → no OOB detected.
Upstream `rtlopez/esp-fc` `master` (checked 2026-08-25) still contains
ESPFC-001…005 unfixed; these reports are ready to file upstream.
