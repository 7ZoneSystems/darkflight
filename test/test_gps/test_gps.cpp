// test/test_gps/test_gps.cpp
// Tests for GPS distance and bearing math used in GpsSensor::calculateHomeVector()

#include <unity.h>
#include <cmath>
#include <cstdint>
#include <Gps.hpp>
#include <GpsParser.hpp>

static constexpr float toRad(float deg) { return deg * (float)M_PI / 180.0f; }

static_assert(sizeof(Gps::UbxNavPosLlh28) == 28, "u-blox legacy POSLLH layout");
static_assert(sizeof(Gps::UbxNavVelned36) == 36, "u-blox legacy VELNED layout");
static_assert(sizeof(Gps::UbxNavSol52) == 52, "u-blox legacy SOL layout");
static_assert(sizeof(Gps::UbxNavSvInfo) == 8, "u-blox legacy SVINFO header layout");

// ---------------------------------------------------------------------------

void test_distance_north()
{
  // 0.1 degree north from equator ≈ 11130 m (within uint16_t range)
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 1000000, 0);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 11130.0f, distance);
}

void test_distance_zero_at_same_position()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(377490000, -1224194000, 377490000, -1224194000);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, distance);
}

void test_local_offset_zero_at_same_position()
{
  const auto offset = Gps::calculateLocalOffset(377490000, -1224194000, 377490000, -1224194000);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, offset.north);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, offset.east);
}

void test_local_offset_signed_axes()
{
  const auto offset = Gps::calculateLocalOffset(0, 0, 10000, -20000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 111.3f, offset.north);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -222.6f, offset.east);
}

void test_local_offset_known_pair_mid_latitude()
{
  // San Francisco: 0.0009 deg north, 0.0011 deg east. Error is bounded by
  // the equirectangular approximation and fixed 111300 m/deg scale used here.
  const auto offset = Gps::calculateLocalOffset(377749000, -1224194000, 377758000, -1224183000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.17f, offset.north);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 96.80f, offset.east);
}

void test_local_offset_longitude_scales_by_origin_latitude()
{
  const auto equator = Gps::calculateLocalOffset(0, 0, 0, 100000);
  const auto sixtyDeg = Gps::calculateLocalOffset(600000000, 0, 600000000, 100000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1113.0f, equator.east);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 556.5f, sixtyDeg.east);
}

void test_bearing_north()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 10000000, 0);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, bearing);
}

void test_bearing_east()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 0, 10000000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(90.0f), bearing);
}

void test_bearing_south()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, -10000000, 0);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(180.0f), bearing);
}

void test_bearing_west()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 0, -10000000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(270.0f), bearing);
}

void test_bearing_northeast()
{
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 10000000, 10000000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(45.0f), bearing);
}

void test_bearing_wraps_360()
{
  // West is 270°, not -90°
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, 0, 0, -5000000);
  TEST_ASSERT_TRUE(bearing >= 0.0f && bearing <= 2.0f * M_PI);
}

// Date line crossing tests
void test_date_line_crossing_east_to_west()
{
  // Home at 179° East, current at -179° (179° West)
  // Actual shortest distance: ~2° (2° * 111300m/deg ≈ 222600m)
  // Without date line handling, would calculate 358° (40067000m) - WRONG!
  const int32_t homeLon = 1790000000;   // 179° * 1e7
  const int32_t curLon = -1790000000;   // -179° * 1e7
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, homeLon, 0, curLon);
  
  // Distance should be ~2° (222600m) not 358° (40000000m+)
  TEST_ASSERT_FLOAT_WITHIN(100.0f, 222600.0f, distance);
}

void test_date_line_crossing_west_to_east()
{
  // Home at -179° (179° West), current at 179° East
  // Actual shortest distance: ~2°
  const int32_t homeLon = -1790000000;  // -179° * 1e7
  const int32_t curLon = 1790000000;    // 179° * 1e7
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, homeLon, 0, curLon);
  
  TEST_ASSERT_FLOAT_WITHIN(100.0f, 222600.0f, distance);
}

void test_date_line_crossing_bearing()
{
  // Home at -179° West, current at 179° East
  // Shortest path: 2° west (across date line from 179°W → 179°E going west/backward)
  // Bearing should be ~270° (west)
  const int32_t homeLon = -1790000000;
  const int32_t curLon = 1790000000;
  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(0, homeLon, 0, curLon);
  
  TEST_ASSERT_FLOAT_WITHIN(0.01f, toRad(270.0f), bearing);
}

void test_local_offset_date_line_crossing_east()
{
  const auto offset = Gps::calculateLocalOffset(0, 1799990000, 0, -1799990000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, offset.north);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 222.6f, offset.east);
}

void test_local_offset_date_line_crossing_west()
{
  const auto offset = Gps::calculateLocalOffset(0, -1799990000, 0, 1799990000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, offset.north);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -222.6f, offset.east);
}

void test_legacy_ubx_velocity_payload_layout()
{
  Gps::UbxNavVelned36 payload{
    .iTow = 1234,
    .velN = -25,
    .velE = 40,
    .velD = 3,
    .speed = 50,
    .gSpeed = 47,
    .heading = 9000000,
    .sAcc = 6,
    .cAcc = 120000,
  };
  TEST_ASSERT_EQUAL_INT32(-25, payload.velN);
  TEST_ASSERT_EQUAL_INT32(40, payload.velE);
  TEST_ASSERT_EQUAL_UINT32(47, payload.gSpeed);
  // NAV-VELNED is cm/s; GpsSensor converts this to the shared mm/s state.
  TEST_ASSERT_EQUAL_INT32(-250, payload.velN * 10);
  TEST_ASSERT_EQUAL_INT32(400, payload.velE * 10);
}

void test_legacy_ubx_sol_num_satellite_offset()
{
  Gps::UbxNavSol52 payload{};
  payload.gpsFix = 3;
  payload.flags = 1;
  payload.pDOP = 125;
  payload.numSV = 9;
  TEST_ASSERT_EQUAL_UINT8(3, payload.gpsFix);
  TEST_ASSERT_EQUAL_UINT8(1, payload.flags);
  TEST_ASSERT_EQUAL_UINT8(9, payload.numSV);
}

// ---------------------------------------------------------------------------

int main()
{
  UNITY_BEGIN();

  RUN_TEST(test_distance_north);
  RUN_TEST(test_distance_zero_at_same_position);
  RUN_TEST(test_local_offset_zero_at_same_position);
  RUN_TEST(test_local_offset_signed_axes);
  RUN_TEST(test_local_offset_known_pair_mid_latitude);
  RUN_TEST(test_local_offset_longitude_scales_by_origin_latitude);
  RUN_TEST(test_bearing_north);
  RUN_TEST(test_bearing_east);
  RUN_TEST(test_bearing_south);
  RUN_TEST(test_bearing_west);
  RUN_TEST(test_bearing_northeast);
  RUN_TEST(test_bearing_wraps_360);
  RUN_TEST(test_date_line_crossing_east_to_west);
  RUN_TEST(test_date_line_crossing_west_to_east);
  RUN_TEST(test_date_line_crossing_bearing);
  RUN_TEST(test_local_offset_date_line_crossing_east);
  RUN_TEST(test_local_offset_date_line_crossing_west);
  RUN_TEST(test_legacy_ubx_velocity_payload_layout);
  RUN_TEST(test_legacy_ubx_sol_num_satellite_offset);

  return UNITY_END();
}
