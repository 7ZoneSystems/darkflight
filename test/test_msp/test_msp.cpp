#include <unity.h>
#include <ArduinoFake.h>
#include <Hal/Gpio.h>
#include <helper_3dmath.h>
#include <Kalman.h>
#include <EscDriver.h>
#include <printf.h>
#include "Connect/Msp.hpp"
#include "Connect/MspParser.hpp"
#include "Connect/MspProcessor.hpp"
#include "msp/msp_protocol.h"
#include <platform.h>
#include <Gps.hpp>

using namespace fakeit;
using namespace Espfc;
using namespace Espfc::Connect;

class MspTestSerial: public Device::SerialDevice
{
public:
  void begin(const SerialDeviceConfig& conf) override {}
  void updateBaudRate(int baud) override {}
  int available() override { return 0; }
  int read() override { return -1; }
  size_t readMany(uint8_t * c, size_t l) override { return 0; }
  int peek() override { return -1; }
  void flush() override {}
  size_t write(uint8_t c) override { return 1; }
  size_t write(const uint8_t * c, size_t l) override { return l; }
  int availableForWrite() override { return 0; }
  bool isTxFifoEmpty() override { return true; }
  bool isSoft() const override { return false; }
  operator bool() const override { return true; }
};

/*void setUp(void)
{
  ArduinoFakeReset();
}*/

// void tearDown(void) {
// // clean stuff up here
// }


#define MSP_V2_FLAG 0

void test_msp_v1_parse_header()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'M' , '<' };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL(MSP_V1, msg.version);
}

void test_msp_v1_parse_no_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'M' , '<', 0, MSP_API_VERSION, 1 };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(0, msg.received);
  TEST_ASSERT_EQUAL_INT(0, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(1, msg.checksum);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v1_parse_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'M' , '<', 2, MSP_API_VERSION, 1, 2, 0 };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(2, msg.received);
  TEST_ASSERT_EQUAL_INT(2, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(0, msg.checksum);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v2_parse_header()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'X' , '<' };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL(MSP_V2, msg.version);
}

void test_msp_v2_parse_no_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'X' , '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 0, 0, 69 };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(0, msg.received);
  TEST_ASSERT_EQUAL_INT(0, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(69, msg.checksum2);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v2_parse_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'X' , '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 2, 0, 1, 2, 102 };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(2, msg.received);
  TEST_ASSERT_EQUAL_INT(2, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(102, msg.checksum2);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void process_msp(Model& model, uint16_t cmd, MspResponse& response, const uint8_t * payload = nullptr, size_t len = 0)
{
  MspMessage msg;
  MspTestSerial serial;
  MspProcessor processor(model);
  msg.cmd = cmd;
  if(payload && len) msg.append(payload, len);
  processor.processCommand(msg, response, serial);
}

void test_msp_boxids_use_betaflight_permanent_ids()
{
  Model model;
  MspResponse response;

  process_msp(model, MSP_BOXIDS, response);

  TEST_ASSERT_EQUAL_UINT8(9, response.len);
  TEST_ASSERT_EQUAL_UINT8(BOXARM, response.data[0]);
  TEST_ASSERT_EQUAL_UINT8(BOXAIRMODE, response.data[1]);
  TEST_ASSERT_EQUAL_UINT8(BOXANGLE, response.data[2]);
  TEST_ASSERT_EQUAL_UINT8(BOXGPSRESCUE, response.data[8]);
}

void test_msp_mode_range_maps_gps_rescue_box_to_poshold()
{
  Model model;
  MspResponse response;
  const uint8_t setModeRange[] = {
    0,                    // range index
    BOXGPSRESCUE,         // Betaflight permanent box id
    0,                    // AUX1
    20,                   // 1400us
    40,                   // 1900us
    0,                    // OR logic
    0,                    // link id
  };

  process_msp(model, MSP_SET_MODE_RANGE, response, setModeRange, sizeof(setModeRange));

  TEST_ASSERT_EQUAL_INT8(1, response.result);
  TEST_ASSERT_EQUAL_UINT8(MODE_POSHOLD, model.config.conditions[0].id);
  TEST_ASSERT_EQUAL_UINT8(AXIS_AUX_1, model.config.conditions[0].ch);
  TEST_ASSERT_EQUAL_INT16(1400, model.config.conditions[0].min);
  TEST_ASSERT_EQUAL_INT16(1900, model.config.conditions[0].max);

  MspResponse ranges;
  process_msp(model, MSP_MODE_RANGES, ranges);

  TEST_ASSERT_EQUAL_UINT8(BOXGPSRESCUE, ranges.data[0]);
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_msp_v1_parse_header);
  RUN_TEST(test_msp_v1_parse_no_payload);
  RUN_TEST(test_msp_v1_parse_payload);
  RUN_TEST(test_msp_v2_parse_header);
  RUN_TEST(test_msp_v2_parse_no_payload);
  RUN_TEST(test_msp_v2_parse_payload);
  RUN_TEST(test_msp_boxids_use_betaflight_permanent_ids);
  RUN_TEST(test_msp_mode_range_maps_gps_rescue_box_to_poshold);

  return UNITY_END();
}
