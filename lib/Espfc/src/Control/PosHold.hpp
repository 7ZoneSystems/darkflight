#pragma once

#include "Control/Pid.h"
#include "Gps.hpp"
#include "Model.h"
#include "Utils/Math.hpp"
#include <algorithm>
#include <cmath>

namespace Espfc::Control {

/*
 * The POS/POSR cascade follows the control structure documented by INAV.
 * Velocity limiting, sample-time handling, and saturation-aware integration
 * are original adaptations of ideas documented in PX4 PositionControl. No
 * source code is copied; see docs/GPS_POSITION_HOLD_REFERENCES.md.
 */
class PosHold
{
public:
  PosHold(Model& model): _model(model) {}

  int begin()
  {
    beginPositionPid(AXIS_ROLL);
    beginPositionPid(AXIS_PITCH);
    beginVelocityPid(AXIS_ROLL);
    beginVelocityPid(AXIS_PITCH);
    reset();
    return 1;
  }

  bool update()
  {
    if (!_model.isModeActive(MODE_POSHOLD))
    {
      if (_fallbackAngle && !_model.isSwitchActive(MODE_POSHOLD))
      {
        _fallbackAngle = false;
      }
      if (_active)
      {
        _model.logger.info().logln(F("GPS POSHOLD EXIT"));
      }
      reset();
      return false;
    }

    if (!isGpsHealthy())
    {
      _model.logger.err().logln(F("GPS POSHOLD BAIL"));
      _model.clearMode(MODE_POSHOLD);
      _fallbackAngle = true;
      reset();
      return false;
    }

    if (!_active)
    {
      captureHoldPoint();
      resetPids();
      _active = true;
      _fallbackAngle = false;
      _model.logger.info().logln(F("GPS POSHOLD ENTER"));
    }

    if (isStickOverrideActive())
    {
      captureHoldPoint();
      resetPids();
      _overrideActive = true;
      updateManualAngles();
      return true;
    }

    if (_overrideActive)
    {
      captureHoldPoint();
      resetPids();
      _lastGpsTs = 0;
      _overrideActive = false;
    }

    if (updateGpsSample())
    {
      updateHoldAngles();
    }
    return true;
  }

  bool isAngleFallbackActive() const
  {
    return _fallbackAngle;
  }

private:
  float maxTargetVelocity() const
  {
    return std::clamp((float)_model.config.gps.posHoldMaxVelocity * 0.01f, 0.2f, 10.0f);
  }

  void beginPositionPid(size_t axis)
  {
    const int rate = std::max(1, (int)_model.state.loopTimer.rate);
    const auto& pc = _model.config.pid[FC_PID_POS];

    auto& pid = _positionPid[axis];
    pid.Kp = (float)pc.P * 0.01f;
    pid.Ki = (float)pc.I * 0.01f;
    pid.Kd = 0.0f;
    pid.Kf = 0.0f;
    pid.iLimitLow = -maxTargetVelocity();
    pid.iLimitHigh = maxTargetVelocity();
    pid.oLimitLow = -maxTargetVelocity();
    pid.oLimitHigh = maxTargetVelocity();
    pid.rate = rate;
    pid.begin();
  }

  void beginVelocityPid(size_t axis)
  {
    const int rate = std::max(1, (int)_model.state.loopTimer.rate);
    const auto& pc = _model.config.pid[FC_PID_POSR];

    auto& pid = _velocityPid[axis];
    pid.Kp = (float)pc.P * 0.1f;
    pid.Ki = (float)pc.I * 0.01f;
    pid.Kd = (float)pc.D * 0.001f;
    pid.Kf = 0.0f;
    pid.iLimitLow = -_model.config.gps.posHoldMaxAngle;
    pid.iLimitHigh = _model.config.gps.posHoldMaxAngle;
    pid.oLimitLow = -_model.config.gps.posHoldMaxAngle;
    pid.oLimitHigh = _model.config.gps.posHoldMaxAngle;
    pid.rate = rate;
    pid.begin();
  }

  void reset()
  {
    _active = false;
    _overrideActive = false;
    _holdLat = 0;
    _holdLon = 0;
    _lastGpsTs = 0;
    _velocityNorth = 0.0f;
    _velocityEast = 0.0f;
    _velocityValid = false;
    _wasClamped = false;
    _wasVelocityClamped = false;
    resetPids();
  }

  void resetPids()
  {
    for (size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      _positionPid[i].resetIterm();
      _velocityPid[i].resetIterm();
      _positionPid[i].outputSaturated = false;
      _velocityPid[i].outputSaturated = false;
    }
  }

  void captureHoldPoint()
  {
    _holdLat = _model.state.gps.location.raw.lat;
    _holdLon = _model.state.gps.location.raw.lon;
  }

  bool isStickOverrideActive() const
  {
    const float deadband = std::clamp((float)_model.config.gps.posHoldStickDeadband * 0.01f, 0.0f, 1.0f);
    return std::fabs(_model.state.input.ch[AXIS_ROLL]) > deadband
        || std::fabs(_model.state.input.ch[AXIS_PITCH]) > deadband;
  }

  bool isGpsHealthy() const
  {
    const auto& gps = _model.state.gps;
    const auto& config = _model.config.gps;

    if (!gps.present || !gps.fix || gps.fixType < 3 || gps.numSats < config.minSats)
    {
      return false;
    }
    if (config.posHoldMaxHorizontalAccuracy <= 0 || config.posHoldGpsTimeout <= 0)
    {
      return false;
    }
    if (gps.accuracy.horizontal > (uint32_t)config.posHoldMaxHorizontalAccuracy)
    {
      return false;
    }
    if (!gps.lastMsgTs)
    {
      return false;
    }

    return micros() - gps.lastMsgTs <= (uint32_t)config.posHoldGpsTimeout * 1000u;
  }

  bool updateGpsSample()
  {
    const auto& gps = _model.state.gps;
    if (!gps.lastMsgTs || gps.lastMsgTs == _lastGpsTs)
    {
      return false;
    }

    uint32_t interval = _lastGpsTs ? gps.lastMsgTs - _lastGpsTs : gps.interval;
    if (interval == 0)
    {
      interval = 100000;
    }
    const float dt = std::clamp(interval * 1e-6f, 0.02f, 2.0f);
    _lastGpsTs = gps.lastMsgTs;

    _positionPid[AXIS_ROLL].dt = dt;
    _positionPid[AXIS_PITCH].dt = dt;
    _positionPid[AXIS_ROLL].rate = 1.0f / dt;
    _positionPid[AXIS_PITCH].rate = 1.0f / dt;
    _velocityPid[AXIS_ROLL].dt = dt;
    _velocityPid[AXIS_PITCH].dt = dt;
    _velocityPid[AXIS_ROLL].rate = 1.0f / dt;
    _velocityPid[AXIS_PITCH].rate = 1.0f / dt;

    const float north = gps.velocity.raw.north * 0.001f;
    const float east = gps.velocity.raw.east * 0.001f;
    const float filterSeconds = _model.config.gps.posHoldVelocityFilter * 0.001f;
    const float alpha = filterSeconds > 0.0f ? std::clamp(dt / (filterSeconds + dt), 0.0f, 1.0f) : 1.0f;

    if (!_velocityValid)
    {
      _velocityNorth = north;
      _velocityEast = east;
      _velocityValid = true;
    }
    else
    {
      _velocityNorth += alpha * (north - _velocityNorth);
      _velocityEast += alpha * (east - _velocityEast);
    }

    return true;
  }

  void updateManualAngles()
  {
    const float angleLimit = Utils::toRad(_model.config.level.angleLimit);
    _model.state.setpoint.angle.set(AXIS_ROLL, _model.state.input.ch[AXIS_ROLL] * angleLimit);
    _model.state.setpoint.angle.set(AXIS_PITCH, _model.state.input.ch[AXIS_PITCH] * angleLimit);
  }

  void updateHoldAngles()
  {
    const auto offset = Gps::calculateLocalOffset(
      _model.state.gps.location.raw.lat,
      _model.state.gps.location.raw.lon,
      _holdLat,
      _holdLon
    );

    float targetNorth = _positionPid[AXIS_ROLL].update(offset.north, 0.0f);
    float targetEast = _positionPid[AXIS_PITCH].update(offset.east, 0.0f);

    const float targetVelocity = std::sqrt(targetNorth * targetNorth + targetEast * targetEast);
    const float maxVelocity = maxTargetVelocity();
    const bool velocityClamped = targetVelocity > maxVelocity && targetVelocity > 0.0f;
    if (velocityClamped)
    {
      const float scale = maxVelocity / targetVelocity;
      targetNorth *= scale;
      targetEast *= scale;
    }
    _positionPid[AXIS_ROLL].outputSaturated = velocityClamped;
    _positionPid[AXIS_PITCH].outputSaturated = velocityClamped;
    if (velocityClamped != _wasVelocityClamped)
    {
      _model.logger.info().logln(velocityClamped ? F("GPS POSHOLD VEL LIMIT") : F("GPS POSHOLD VEL UNLIMIT"));
      _wasVelocityClamped = velocityClamped;
    }

    // velocity.raw is the direct UBX NAV_PVT velN/velE measurement in mm/s;
    // convert to m/s for the velocity PID. It is never derived from position.
    // Scaling verified: FC_PID_POSR P/I/D values use { P*10, I*100, D*1000 }
    // so velocity input in m/s gives angle output in degrees
    const float velocityNorth = _velocityNorth;
    const float velocityEast = _velocityEast;

    _velocityPid[AXIS_ROLL].outputSaturated = _wasClamped;
    _velocityPid[AXIS_PITCH].outputSaturated = _wasClamped;
    float earthNorthAngle = Utils::toRad(_velocityPid[AXIS_ROLL].update(targetNorth, velocityNorth));
    float earthEastAngle = Utils::toRad(_velocityPid[AXIS_PITCH].update(targetEast, velocityEast));

    clampEarthAngle(earthNorthAngle, earthEastAngle);
    rotateEarthToBody(earthNorthAngle, earthEastAngle);
  }

  void clampEarthAngle(float& northAngle, float& eastAngle)
  {
    const float maxAngleDeg = std::min((float)_model.config.gps.posHoldMaxAngle, (float)_model.config.level.angleLimit);
    const float maxAngle = Utils::toRad(std::max(0.0f, maxAngleDeg));
    const float angle = std::sqrt(northAngle * northAngle + eastAngle * eastAngle);

    const bool clamped = angle > maxAngle && angle > 0.0f;
    if (clamped)
    {
      const float scale = maxAngle / angle;
      northAngle *= scale;
      eastAngle *= scale;
    }

    if (clamped != _wasClamped)
    {
      _model.logger.info().logln(clamped ? F("GPS POSHOLD CLAMP") : F("GPS POSHOLD UNCLAMP"));
      _wasClamped = clamped;
    }
  }

  void rotateEarthToBody(float northAngle, float eastAngle)
  {
    const float heading = _model.state.attitude.euler[AXIS_YAW];
    const float cosHeading = cosf(heading);
    const float sinHeading = sinf(heading);

    _model.state.setpoint.angle.set(AXIS_ROLL, eastAngle * cosHeading - northAngle * sinHeading);
    _model.state.setpoint.angle.set(AXIS_PITCH, northAngle * cosHeading + eastAngle * sinHeading);
  }

  Model& _model;
  Pid _positionPid[AXIS_COUNT_RP];
  Pid _velocityPid[AXIS_COUNT_RP];
  bool _active = false;
  bool _overrideActive = false;
  bool _wasClamped = false;
  bool _wasVelocityClamped = false;
  bool _fallbackAngle = false;
  bool _velocityValid = false;
  uint32_t _lastGpsTs = 0;
  float _velocityNorth = 0.0f;
  float _velocityEast = 0.0f;
  int32_t _holdLat = 0;
  int32_t _holdLon = 0;
};

} // namespace Espfc::Control
