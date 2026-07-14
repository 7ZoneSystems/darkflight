# GPS Configuration

## CLI

The GPS function is assigned through the same serial function mask used by
MSP. The exposed serial IDs are Betaflight `UART1`, `UART2`, and `UART3`.
On the classic ESP32 target these map to Arduino `Serial`, `Serial1`, and
`Serial2`; UART3 is the third exposed FC UART, not an ESP hardware UART 3.

```text
gps status
gps config
gps port UART2
set gps_min_sats 8
set gps_auto_config 1
set gps_auto_baud 1
set gps_sbas_mode 0
set gps_enable_galileo 1
set pin_uart2_tx 17
set pin_uart2_rx 16
save
reboot
```

`gps status` reports the complete parsed position, direct NED velocity,
accuracy, fix, satellite, and home state. `gps config` reports the selected
GPS port and effective baud. The existing `pin_serial_0_*` names remain
supported; `pin_uart1_*`, `pin_uart2_*`, and `pin_uart3_*` are aliases.

The `serial_N` CLI parameters use the Betaflight-compatible six-value shape:

```text
set serial_1 <function-mask> <msp-baud> <gps-baud> <telemetry-baud> <blackbox-baud>
```

For example, `set serial_1 2 115200 9600 0 0` assigns GPS to UART2 at
9600 baud. A GPS baud of `0` uses the port baud as the initial detection baud
and retains the receiver baud scan.

## MSP / Configurator

Betaflight Configurator 10.10 can assign the GPS function and GPS baud from
the Ports tab through `MSP_CF_SERIAL_CONFIG` / `MSP_SET_CF_SERIAL_CONFIG` and
their MSP2 common equivalents. The GPS tab receives the standard
`MSP_GPS_CONFIG`, `MSP_RAW_GPS`, `MSP_COMP_GPS`, and `MSP_GPSSVINFO` payloads.
The Modes tab exposes GPS HOLD through the permanent Betaflight
`BOXGPSRESCUE` box ID; the box is used as the AUX switch for this assisted
hold implementation.

Standard Betaflight Ports MSP does not carry arbitrary target GPIO resource
assignments, so ESP TX/RX pin changes are intentionally CLI-only and must be
followed by `save` and `reboot`.

## u-blox NEO-6M

NEO-6M is supported through its legacy UBX messages: `NAV-POSLLH`,
`NAV-VELNED`, `NAV-SOL`, and `NAV-SVINFO`. `NAV-VELNED` reports cm/s and is
converted to the common GPS state’s mm/s representation before it reaches
GPS hold. The message layouts and scales follow the public u-blox 6 receiver
protocol specification; no u-blox, INAV, or PX4 source code was copied into
this repository. See the source specification at
https://content.u-blox.com/sites/default/files/products/documents/u-blox6-GPS-GLONASS-QZSS-V14_ReceiverDescrProtSpec_%28GPS.G6-SW-12013%29_Public.pdf.
