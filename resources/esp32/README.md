# ESP32 Official Documentation Index

Local copies of **official Espressif** documentation used by the DarkFlight
hardware-behavior audit (2026-08-25). The firmware targets the
`espressif32` PlatformIO platform (Arduino core 2.x / ESP-IDF v4.4 lineage),
so IDF **v4.4** pages are fetched to match the actual runtime.

| File | Title | Source URL | Version/Date | Target | Why relevant |
|---|---|---|---|---|---|
| gpio/esp-idf-v4.4-gpio.html | GPIO & RTC GPIO | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/gpio.html | IDF v4.4 | ESP32 | Pad table: strapping pins (0,2,5,12,15), flash pins 6-11/16-17, input-only 34-39, ADC2+WiFi restriction, GPIO matrix/IOMUX APIs |
| peripherals/esp-idf-v4.4-uart.html | UART | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/uart.html | IDF v4.4 | ESP32 | UART routing of GPS/telemetry/CRSF ports; any-GPIO mapping via matrix |
| peripherals/esp-idf-v4.4-i2c.html | I2C | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/i2c.html | IDF v4.4 | ESP32 | Sensor bus behavior; software I2C fallback used by EspWire |
| peripherals/esp-idf-v4.4-spi-master.html | SPI Master | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/spi_master.html | IDF v4.4 | ESP32 | Gyro/baro SPI buses, DMA alignment notes |
| peripherals/esp-idf-v4.4-timer.html | General Purpose Timer | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/timer.html | IDF v4.4 | ESP32 | GPTimer allocation (multi-core PID timing) |
| peripherals/esp-idf-v4.4-ledc.html | LED Control (LEDC) | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/ledc.html | IDF v4.4 | ESP32 | PWM capability NOT used for motors (RMT is); buzzer/LED tone potential user |
| peripherals/esp-idf-v4.4-mcpwm.html | MCPWM | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/mcpwm.html | IDF v4.4 | ESP32 | Evaluated, not used; kept for future motor-control alternatives |
| peripherals/esp-idf-v4.4-rmt.html | Remote Control (RMT) | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/rmt.html | IDF v4.4 | ESP32 | Motor/ESC signal generator: 8 channels, 8-bit divider, 64×32-bit mem/channel, half-duplex bidir DShot, IRAM ISR requirements |
| peripherals/esp-idf-v4.4-adc.html | Analog to Digital Converter | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/adc.html | IDF v4.4 | ESP32 | ADC1 vs ADC2; ADC2 unusable while Wi-Fi AP (config portal) is active |
| peripherals/esp-idf-v4.4-intr_alloc.html | Interrupt Allocation | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/system/intr_alloc.html | IDF v4.4 | ESP32 | `ESP_INTR_FLAG_IRAM` rules for the RMT tx-end callback |
| freertos/esp-idf-v4.4-freertos-smp.html | FreeRTOS (SMP) | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-guides/freertos-smp.html | IDF v4.4 | ESP32 | Task/pin affinity (gyro/PID cores), ISR-task interaction, stack sizes |
| memory/esp-idf-v4.4-bootloader.html | ESP32 Bootloader | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-guides/bootloader.html | IDF v4.4 | ESP32 | Boot process, strapping influence, reset behavior |
| memory/esp-idf-v4.4-system-reset.html | System API (reset reasons, restart) | https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/system/system.html | IDF v4.4 | ESP32 | Reboot/reset-reason semantics used by MSP_REBOOT |
| datasheet/esp32_datasheet_en.pdf | ESP32 Series Datasheet | https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf | latest at download (2026-08-25) | ESP32 | §Strapping Pins, §Pin definitions, electrical limits |
| errata/eco_and_workarounds_for_bugs_in_esp32_en.pdf | ECO and Workarounds for Bugs in ESP32 | https://www.espressif.com/sites/default/files/documentation/eco_and_workarounds_for_bugs_in_esp32_en.pdf | latest at download (2026-08-25) | ESP32 | GPIO36/39 input-glitch workaround referenced by IDF GPIO docs |

Not downloaded (too large, cited instead):

- ESP32 Technical Reference Manual (IO MUX / GPIO Matrix chapter):
  https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
- Arduino-ESP32 core docs (LEDC/attach API differences between core 2.x and 3.x):
  https://docs.espressif.com/projects/arduino-esp32/en/latest/

## Hardware constraint checklist derived for DarkFlight (ESP32 target)

1. Strapping pins **GPIO0, GPIO2, GPIO5, GPIO12(MTDI), GPIO15(MTDO)**: external
   pull devices at reset change boot mode / flash voltage. `TargetESP32.h`
   defaults put a motor on **GPIO12** (`ESPFC_OUTPUT_3`) — an ESC/servo with an
   input pull-up on that pin can prevent boot. GPIO12 as output is fine after
   boot; document, do not silently renumber (would break existing wiring).
2. **GPIO6-11** (flash) and **GPIO16/17 on PSRAM modules**: never reuse.
   `TargetESP32.h` maps UART2 (GPS/serial RX) to 16/17 — valid on WROOM
   modules, broken on WROVER; board-specific caution.
3. **GPIO34-39 input-only**, no internal pulls: PPM input 35, ADC 36/39 comply.
4. **ADC2 unusable while Wi-Fi is active**: DarkFlight uses ADC1 pins only; the
   Wi-Fi config AP therefore cannot break battery sensing.
5. **RMT** drives all ESC protocols: 8 channels total, 8-bit clock divider,
   per-channel 64×32-bit memory, tx-end ISR must be IRAM-resident and only call
   IRAM code (integer-only paths in `EscDriverEsp32` comply).
6. **No LEDC/MCPWM/timer conflicts**: motors use RMT exclusively; LEDC is free
   for buzzer/LED use; GPTimer usage is limited to multi-core PID scheduling.
7. **Interrupts**: RMT tx-end callback registered once with
   `ESP_INTR_FLAG_IRAM`; handlers avoid non-IRAM calls (verified in source).
8. **Reset behavior**: NVS-stored config is magic+version+size checked
   (`Utils/Storage`); schema growth invalidates old blobs safely (defaults),
   no stale-layout deserialization possible.

Sources read fully for this audit: gpio, rmt, intr_alloc, adc, uart,
datasheet strapping section, errata index.
