# GPS Position Hold References

The GPS position-hold controller in `lib/Espfc/src/Control/PosHold.hpp` is
original esp-fc code. It does not copy source code from the projects below.
Their documented control behavior was used as an engineering reference:

- INAV documents a POS/POSR cascade: position error produces a desired
  velocity, and velocity error produces the navigation correction. Its POS
  controller is documented as P-only, and its POSHOLD modes distinguish
  attitude-stick control from velocity-stick control.
  Source: https://github.com/iNavFlight/inav/blob/master/docs/Navigation.md
  License: GNU General Public License v3, as stated in INAV's LICENSE file:
  https://github.com/iNavFlight/inav/blob/master/LICENSE
- PX4 documents a position-to-velocity stage with constrained horizontal
  velocity, followed by a velocity controller with sample-time-aware
  integration and saturation handling. The comparable implementation is
  `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`.
  Source: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/PositionControl/PositionControl.cpp
  License: BSD 3-Clause, as stated in PX4's LICENSE file:
  https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE

This adaptation keeps the esp-fc architecture and Betaflight compatibility:
GPS NAV-PVT `velN`/`velE` values remain the direct velocity measurements, the
controller still outputs only roll/pitch angles, and no PX4 or INAV source
file is added to the firmware. The Betaflight MSP/PID compatibility code is
covered by the vendored Betaflight license notices already present in
`lib/betaflight`.
