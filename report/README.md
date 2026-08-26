# Reports Index

Audit and analysis reports for DarkFlight. All work local; nothing pushed unless stated.

| File | Contents |
|------|----------|
| [report.md](report.md) | Memory-safety audit of inherited CRSF/MSP code — upstream-submission summary (see README §Upstream contributions) |
| [bug-audit.md](bug-audit.md) | Deep correctness/safety audit (hardware & regression focus): ESP32 targets, GPS/nav/sensor/motor-output paths; references `resources/esp32/` docs |
| [reports.md](reports.md) | CWE arithmetic-defect audit: whole-repo scan against MITRE CWE v4.14 (`resources/cwe/`), autofix validation, fixes in commits `bdf1f7e`, `536e182`, `902bf21` |
| [loc-report.txt](loc-report.txt) | Lines-of-code statistics |

Related resources kept outside this folder:

- `resources/cwe/` — MITRE CWE List v4.14 database used by reports.md
- `resources/esp32/` — Espressif documentation snapshot used by bug-audit.md
