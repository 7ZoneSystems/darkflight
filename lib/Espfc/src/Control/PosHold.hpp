#pragma once

#include "Control/Pid.h"
#include "Gps.hpp"
#include "Model.h"
#include "Utils/Math.hpp"
#include <algorithm>
#include <cmath>

namespace Espfc::Control {

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
      reset();
      return false;
    }

    if (!_active)
    {
      captureHoldPoint();
      resetPids();
      _active = true;
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
      _overrideActive = false;
    }

    updateHoldAngles();
    return true;
  }

private:
  static constexpr float MAX_TARGET_VELOCITY = 2.0f; // m/s

  void beginPositionPid(size_t axis)
  {
    const int rate = std::max(1, (int)_model.state.loopTimer.rate);
    const auto& pc = _model.config.pid[FC_PID_POS];

    auto& pid = _positionPid[axis];
    pid.Kp = (float)pc.P * 0.01f;
    pid.Ki = (float)pc.I * 0.01f;
    pid.Kd = 0.0f;
    pid.Kf = 0.0f;
    pid.iLimitLow = -MAX_TARGET_VELOCITY;
    pid.iLimitHigh = MAX_TARGET_VELOCITY;
    pid.oLimitLow = -MAX_TARGET_VELOCITY;
    pid.oLimitHigh = MAX_TARGET_VELOCITY;
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
    resetPids();
  }

  void resetPids()
  {
    for (size_t i = 0; i < AXIS_COUNT_RP; i++)
    {
      _positionPid[i].resetIterm();
      _velocityPid[i].resetIterm();
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

    const float targetNorth = _positionPid[AXIS_ROLL].update(offset.north, 0.0f);
    const float targetEast = _positionPid[AXIS_PITCH].update(offset.east, 0.0f);

    // velocity.raw is in mm/s from UBX NAV_PVT; convert to m/s for PID
    // Scaling verified: FC_PID_POSR P/I/D values use { P*10, I*100, D*1000 }
    // so velocity input in m/s gives angle output in degrees
    const float velocityNorth = _model.state.gps.velocity.raw.north * 0.001f;
    const float velocityEast = _model.state.gps.velocity.raw.east * 0.001f;

    float earthNorthAngle = Utils::toRad(_velocityPid[AXIS_ROLL].update(targetNorth, velocityNorth));
    float earthEastAngle = Utils::toRad(_velocityPid[AXIS_PITCH].update(targetEast, velocityEast));

    clampEarthAngle(earthNorthAngle, earthEastAngle);
    rotateEarthToBody(earthNorthAngle, earthEastAngle);
  }

  void clampEarthAngle(float& northAngle, float& eastAngle) const
  {
    const float maxAngleDeg = std::min((float)_model.config.gps.posHoldMaxAngle, (float)_model.config.level.angleLimit);
    const float maxAngle = Utils::toRad(std::max(0.0f, maxAngleDeg));
    const float angle = std::sqrt(northAngle * northAngle + eastAngle * eastAngle);

    if (angle > maxAngle && angle > 0.0f)
    {
      const float scale = maxAngle / angle;
      northAngle *= scale;
      eastAngle *= scale;
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
  int32_t _holdLat = 0;
  int32_t _holdLon = 0;
};

} // namespace Espfc::Control
