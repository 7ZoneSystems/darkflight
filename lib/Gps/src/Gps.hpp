#pragma once

#include "GpsProtocol.hpp"
#include "GpsParser.hpp"
#include <cmath>
#include <tuple>

namespace Gps {

struct LocalCoordinate
{
  float north = 0.0f;
  float east = 0.0f;
};

static constexpr float LAT_TO_M = 1.113e-2f;     // deg * 1e-7 to meters (111300 m/deg / 1e7)
static constexpr int64_t LON_180 = 1800000000LL; // 180 * 1e7
static constexpr int64_t LON_360 = 3600000000LL; // 360 * 1e7

inline int64_t calculateLongitudeDelta(int32_t originLon, int32_t pointLon)
{
  int64_t dlon = (int64_t)pointLon - (int64_t)originLon;

  if (dlon > LON_180)
    dlon -= LON_360;
  else if (dlon < -LON_180)
    dlon += LON_360;

  return dlon;
}

/**
 * Calculates a signed local north/east offset using the same short-range
 * equirectangular approximation as calculateDistanceAndBearing().
 * valid for short distances < few km
 * @param originLat Origin latitude in degrees * 1e7
 * @param originLon Origin longitude in degrees * 1e7
 * @param pointLat Point latitude in degrees * 1e7
 * @param pointLon Point longitude in degrees * 1e7
 * @return LocalCoordinate where north/east are in meters
 */
inline LocalCoordinate calculateLocalOffset(int32_t originLat, int32_t originLon, int32_t pointLat, int32_t pointLon)
{
  const float north = (pointLat - originLat) * LAT_TO_M;
  const float east =
      (float)calculateLongitudeDelta(originLon, pointLon) * LAT_TO_M * cosf(originLat * 1e-7f * (float)M_PI / 180.0f);

  return LocalCoordinate{north, east};
}

/**
 * Calculates the distance between two GPS coordinates using the equirectangular approximation.
 * valid for short distances < few km
 * @param homeLat Home latitude in degrees * 1e7
 * @param homeLon Home longitude in degrees * 1e7
 * @param curLat Current latitude in degrees * 1e7
 * @param curLon Current longitude in degrees * 1e7
 * @return Tuple of (distance, bearing) where distance is in meters and bearing is in radians (0-2PI)
 */
inline std::tuple<float, float> calculateDistanceAndBearing(int32_t homeLat, int32_t homeLon, int32_t curLat,
                                                            int32_t curLon)
{
  const LocalCoordinate offset = calculateLocalOffset(homeLat, homeLon, curLat, curLon);

  const float distance = sqrtf(offset.north * offset.north + offset.east * offset.east);
  float bearing = atan2f(offset.east, offset.north);
  if (bearing < 0.0f) bearing += 2.0f * (float)M_PI; // normalize to [0, 2PI]

  return std::make_tuple(distance, bearing);
}

} // namespace Gps
