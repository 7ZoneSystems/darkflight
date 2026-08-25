#include <unity.h>
#include <ArduinoFake.h>
#include "Device/InputCRSF.h"
#include "Device/InputIBUS.hpp"
#include "msp/msp_protocol.h"
#include <Gps.hpp>

using namespace Espfc;
using namespace Espfc::Device;
using namespace Espfc::Rc;
using namespace fakeit;

void test_input_crsf_rc_valid()
{
  InputCRSF input;
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));
  uint8_t* frame_data = reinterpret_cast<uint8_t*>(&frame);

  When(Method(ArduinoFake(), micros)).Return(0);

  input.begin(nullptr, nullptr);

  const uint8_t data[] = {0xC8, 0x18, 0x16, 0xE0, 0x03, 0xDF, 0xD9, 0xC0, 0xF7, 0x8B, 0x5F, 0x94, 0xAF,
                          0x7C, 0xE5, 0x2B, 0x5F, 0xF9, 0xCA, 0x07, 0x00, 0x00, 0x4C, 0x7C, 0xE2, 0x23};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    input.parse(frame, data[i]);
  }

  for (size_t i = 0; i < sizeof(data); i++)
  {
    TEST_ASSERT_EQUAL_UINT8(data[i], frame_data[i]);
  }

  const uint8_t crc = Crsf::crc(frame);
  TEST_ASSERT_EQUAL_UINT8(0x23, crc);
  TEST_ASSERT_EQUAL_UINT8(0x23, frame.crc());

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.type);

  TEST_ASSERT_EQUAL_UINT16(1500u, input.get(0));
  TEST_ASSERT_EQUAL_UINT16(1500u, input.get(1));
  TEST_ASSERT_EQUAL_UINT16(1425u, input.get(2));
  TEST_ASSERT_EQUAL_UINT16(1500u, input.get(3));
  TEST_ASSERT_EQUAL_UINT16(1000u, input.get(4));
  TEST_ASSERT_EQUAL_UINT16(1000u, input.get(5));
}

void test_input_crsf_rc_valid_no_payload()
{
  InputCRSF input;
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));
  uint8_t* frame_data = reinterpret_cast<uint8_t*>(&frame);

  When(Method(ArduinoFake(), micros)).Return(0);

  input.begin(nullptr, nullptr);

  const uint8_t data[] = {0xC8, 0x02, 0x16, 0xD3};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    input.parse(frame, data[i]);
  }

  for (size_t i = 0; i < sizeof(data); i++)
  {
    TEST_ASSERT_EQUAL_UINT8(data[i], frame_data[i]);
  }

  const uint8_t crc = Crsf::crc(frame);
  TEST_ASSERT_EQUAL_UINT8(0xD3, crc);
  TEST_ASSERT_EQUAL_UINT8(0xD3, frame.crc());

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(0x02, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.type);
}

void test_input_crsf_rc_prefix()
{
  InputCRSF input;
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  When(Method(ArduinoFake(), micros)).Return(0);

  input.begin(nullptr, nullptr);

  // prefix with few random bytes
  const uint8_t data[] = {0xA1, 0x04, 0xC5, 0x09, 0xC8, 0x18, 0x16, 0xE0, 0x03, 0xDF, 0xD9, 0xC0, 0xF7, 0x8B, 0x5F,
                          0x94, 0xAF, 0x7C, 0xE5, 0x2B, 0x5F, 0xF9, 0xCA, 0x07, 0x00, 0x00, 0x4C, 0x7C, 0xE2, 0x23};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    input.parse(frame, data[i]);
  }

  const uint8_t crc = Crsf::crc(frame);
  TEST_ASSERT_EQUAL_UINT8(0x23, crc);

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.type);

  TEST_ASSERT_EQUAL_UINT16(1500, input.get(0));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(1));
}

void test_crsf_encode_rc()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  CrsfData data;
  data.chan0 = 992;
  data.chan1 = 992;
  data.chan2 = 172;
  data.chan3 = 992;
  data.chan4 = 992;
  data.chan5 = 992;
  data.chan6 = 992;
  data.chan7 = 992;
  data.chan8 = 992;
  data.chan9 = 992;
  data.chan10 = 992;
  data.chan11 = 992;
  data.chan12 = 992;
  data.chan13 = 992;
  data.chan14 = 992;
  data.chan15 = 992;

  Crsf::encodeRcData(frame, data);

  const uint8_t expected[] = {0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0x2B, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F,
                              0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0xDB};

  uint8_t* frame_data = reinterpret_cast<uint8_t*>(&frame);

  TEST_ASSERT_EQUAL_UINT8(expected[0], frame_data[0]); // addr
  TEST_ASSERT_EQUAL_UINT8(expected[1], frame_data[1]); // size
  TEST_ASSERT_EQUAL_UINT8(expected[2], frame_data[2]); // type
  TEST_ASSERT_EQUAL_UINT8(expected[3], frame_data[3]);
  TEST_ASSERT_EQUAL_UINT8(expected[4], frame_data[4]);
  TEST_ASSERT_EQUAL_UINT8(expected[5], frame_data[5]);
  TEST_ASSERT_EQUAL_UINT8(expected[6], frame_data[6]);
  TEST_ASSERT_EQUAL_UINT8(expected[7], frame_data[7]);
  TEST_ASSERT_EQUAL_UINT8(expected[8], frame_data[8]);
  TEST_ASSERT_EQUAL_UINT8(expected[9], frame_data[9]);
  TEST_ASSERT_EQUAL_UINT8(expected[10], frame_data[10]);
  TEST_ASSERT_EQUAL_UINT8(expected[11], frame_data[11]);
  TEST_ASSERT_EQUAL_UINT8(expected[12], frame_data[12]);
  TEST_ASSERT_EQUAL_UINT8(expected[13], frame_data[13]);
  TEST_ASSERT_EQUAL_UINT8(expected[14], frame_data[14]);
  TEST_ASSERT_EQUAL_UINT8(expected[15], frame_data[15]);
  TEST_ASSERT_EQUAL_UINT8(expected[16], frame_data[16]);
  TEST_ASSERT_EQUAL_UINT8(expected[17], frame_data[17]);
  TEST_ASSERT_EQUAL_UINT8(expected[18], frame_data[18]);
  TEST_ASSERT_EQUAL_UINT8(expected[19], frame_data[19]);
  TEST_ASSERT_EQUAL_UINT8(expected[20], frame_data[20]);
  TEST_ASSERT_EQUAL_UINT8(expected[21], frame_data[21]);
  TEST_ASSERT_EQUAL_UINT8(expected[22], frame_data[22]);
  TEST_ASSERT_EQUAL_UINT8(expected[23], frame_data[23]);
  TEST_ASSERT_EQUAL_UINT8(expected[24], frame_data[24]);
  TEST_ASSERT_EQUAL_UINT8(expected[25], frame_data[25]); // crc

  const uint8_t crc = Crsf::crc(frame);
  TEST_ASSERT_EQUAL_UINT8(0xdb, crc);

  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(0x18, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_RC_CHANNELS_PACKED, frame.type);
}

void test_crsf_decode_rc_struct()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  CrsfData data;
  data.chan0 = 992;
  data.chan1 = 992;
  data.chan2 = 172;
  data.chan3 = 992;
  data.chan4 = 992;
  data.chan5 = 992;
  data.chan6 = 992;
  data.chan7 = 992;
  data.chan8 = 992;
  data.chan9 = 992;
  data.chan10 = 992;
  data.chan11 = 992;
  data.chan12 = 992;
  data.chan13 = 992;
  data.chan14 = 992;
  data.chan15 = 992;

  Crsf::encodeRcData(frame, data);

  uint16_t channels[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  Crsf::decodeRcData(channels, (const CrsfData*)frame.payload);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[0]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[1]);
  TEST_ASSERT_EQUAL_UINT16(988, channels[2]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[3]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[4]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[5]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[6]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[7]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[8]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[9]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[10]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[11]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[12]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[13]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[14]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[15]);
}

void test_crsf_decode_rc_shift8()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  CrsfData data;
  data.chan0 = 992;
  data.chan1 = 992;
  data.chan2 = 172;
  data.chan3 = 992;
  data.chan4 = 992;
  data.chan5 = 992;
  data.chan6 = 992;
  data.chan7 = 992;
  data.chan8 = 992;
  data.chan9 = 992;
  data.chan10 = 992;
  data.chan11 = 992;
  data.chan12 = 992;
  data.chan13 = 992;
  data.chan14 = 992;
  data.chan15 = 992;

  Crsf::encodeRcData(frame, data);

  uint16_t channels[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  Crsf::decodeRcDataShift8(channels, (const CrsfData*)frame.payload);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[0]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[1]);
  TEST_ASSERT_EQUAL_UINT16(988, channels[2]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[3]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[4]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[5]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[6]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[7]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[8]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[9]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[10]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[11]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[12]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[13]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[14]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[15]);
}

/*void test_crsf_decode_rc_shift32()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  CrsfData data;
  data.chan0 = 992;
  data.chan1 = 992;
  data.chan2 = 191;
  data.chan3 = 992;
  data.chan4 = 992;
  data.chan5 = 992;
  data.chan6 = 992;
  data.chan7 = 992;
  data.chan8 = 992;
  data.chan9 = 992;
  data.chan10 = 992;
  data.chan11 = 992;
  data.chan12 = 992;
  data.chan13 = 992;
  data.chan14 = 992;
  data.chan15 = 992;

  Crsf::encodeRcData(frame, data);

  uint16_t channels[16] = { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0};

  Crsf::decodeRcDataShift32(channels, (const CrsfData*)frame.message.payload);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[0]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[1]);
  TEST_ASSERT_EQUAL_UINT16(1000, channels[2]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[3]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[4]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[5]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[6]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[7]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[8]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[9]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[10]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[11]);

  TEST_ASSERT_EQUAL_UINT16(1500, channels[12]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[13]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[14]);
  TEST_ASSERT_EQUAL_UINT16(1500, channels[15]);
}*/

void test_crsf_encode_tlm()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  frame.prepare(CRSF_FRAMETYPE_HEARTBEAT);
  frame.writeU8(0x01);
  frame.finalize();

  TEST_ASSERT_EQUAL_UINT8(0xC8, frame.addr);       // addr
  TEST_ASSERT_EQUAL_UINT8(0x03, frame.size);       // size
  TEST_ASSERT_EQUAL_UINT8(0x0B, frame.type);       // type: heartbeat
  TEST_ASSERT_EQUAL_UINT8(0x01, frame.payload[0]); // payload
  TEST_ASSERT_EQUAL_UINT8(0x90, frame.payload[1]); // crc

  const uint8_t crc = Crsf::crc(frame);
  TEST_ASSERT_EQUAL_UINT8(0x90, crc);
}

void test_crsf_encode_msp_v1()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  Connect::MspResponse resp;
  resp.version = Connect::MSP_V1;
  resp.cmd = MSP_API_VERSION;
  resp.result = 0;
  resp.writeU8(1);
  resp.writeU8(2);
  resp.writeU8(3);

  uint8_t buff[255];
  size_t size = resp.serialize(buff, sizeof(buff));
  const uint8_t* beg = buff + 3;        // skip msp header
  const uint8_t* end = buff + size - 1; // skip crc
  uint8_t seq = 0;

  beg = Crsf::encodeMspData(frame, CRSF_ADDRESS_RADIO_TRANSMITTER, 1, seq++, true, beg, end);
  TEST_ASSERT_TRUE(beg == end);

  // crsf headers
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(10, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_RESP, frame.type);

  // crsf ext headers
  TEST_ASSERT_EQUAL_UINT8(0xEA, frame.payload[0]); // radio-transmitter
  TEST_ASSERT_EQUAL_UINT8(0xC8, frame.payload[1]); // FC
  TEST_ASSERT_EQUAL_UINT8(0x30, frame.payload[2]); // status

  // ext msp v1 header
  TEST_ASSERT_EQUAL_UINT8(3, frame.payload[3]); // size
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[4]); // type // api_version(1)

  // ext msp payload
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[5]); // param1
  TEST_ASSERT_EQUAL_UINT8(2, frame.payload[6]); // param2
  TEST_ASSERT_EQUAL_UINT8(3, frame.payload[7]); // param3

  // crsf crc
  TEST_ASSERT_EQUAL_UINT8(0x6D, frame.crc());
}

void test_crsf_encode_msp_v1_fragmented()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  Connect::MspResponse resp;
  resp.version = Connect::MSP_V1;
  resp.cmd = MSP_API_VERSION;
  resp.result = 0;
  for (size_t i = 0; i < 64; i++)
  {
    resp.writeU8(i);
  }

  uint8_t buff[255];
  size_t size = resp.serialize(buff, sizeof(buff));
  const uint8_t* beg = buff + 3;        // skip msp header
  const uint8_t* end = buff + size - 1; // skip crc
  uint8_t seq = 0;

  beg = Crsf::encodeMspData(frame, CRSF_ADDRESS_RADIO_TRANSMITTER, 1, seq++, true, beg, end);
  TEST_ASSERT_FALSE(beg == end);

  // crsf headers
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(62, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_RESP, frame.type);

  // crsf ext headers
  TEST_ASSERT_EQUAL_UINT8(0xEA, frame.payload[0]); // radio-transmitter
  TEST_ASSERT_EQUAL_UINT8(0xC8, frame.payload[1]); // FC
  TEST_ASSERT_EQUAL_UINT8(0x30, frame.payload[2]); // status

  // ext msp v1 header
  TEST_ASSERT_EQUAL_UINT8(64, frame.payload[3]); // size
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[4]);  // type // api_version(1)

  // ext msp payload
  TEST_ASSERT_EQUAL_UINT8(0, frame.payload[5]); // param0
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[6]); // param1
  TEST_ASSERT_EQUAL_UINT8(2, frame.payload[7]); // param2

  // crsf crc
  TEST_ASSERT_EQUAL_UINT8(0xFE, frame.crc());

  beg = Crsf::encodeMspData(frame, CRSF_ADDRESS_RADIO_TRANSMITTER, 1, seq++, false, beg, end);
  TEST_ASSERT_TRUE(beg == end);

  // crsf headers
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(14, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_RESP, frame.type);

  // crsf ext headers
  TEST_ASSERT_EQUAL_UINT8(0xEA, frame.payload[0]); // radio-transmitter
  TEST_ASSERT_EQUAL_UINT8(0xC8, frame.payload[1]); // FC
  TEST_ASSERT_EQUAL_UINT8(0x21, frame.payload[2]); // status

  // ext msp payload
  TEST_ASSERT_EQUAL_UINT8(55, frame.payload[3]); // param55
  TEST_ASSERT_EQUAL_UINT8(56, frame.payload[4]); // param56
  TEST_ASSERT_EQUAL_UINT8(57, frame.payload[5]); // param57

  // crsf crc
  TEST_ASSERT_EQUAL_UINT8(0x73, frame.crc());
}

void test_crsf_encode_msp_v2()
{
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));

  Connect::MspResponse resp;
  resp.version = Connect::MSP_V2;
  resp.cmd = MSP_API_VERSION;
  resp.result = 0;
  resp.writeU8(1);
  resp.writeU8(2);
  resp.writeU8(3);

  uint8_t buff[255];
  size_t size = resp.serialize(buff, sizeof(buff));
  const uint8_t* beg = buff + 3;        // skip msp header
  const uint8_t* end = buff + size - 1; // skip crc
  uint8_t seq = 0xff;

  beg = Crsf::encodeMspData(frame, CRSF_ADDRESS_RADIO_TRANSMITTER, 2, seq, true, beg, end);
  TEST_ASSERT_TRUE(beg == end);

  // crsf headers
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(13, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_RESP, frame.type);

  // crsf ext headers
  TEST_ASSERT_EQUAL_UINT8(0xEA, frame.payload[0]); // radio-transmitter addr
  TEST_ASSERT_EQUAL_UINT8(0xC8, frame.payload[1]); // FC addr
  TEST_ASSERT_EQUAL_UINT8(0x5F, frame.payload[2]); // status flags

  // ext msp v2 header
  TEST_ASSERT_EQUAL_UINT8(0, frame.payload[3]); // flags
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[4]); // type: api_version(1) (lo)
  TEST_ASSERT_EQUAL_UINT8(0, frame.payload[5]); // type: api_version(1) (hi)
  TEST_ASSERT_EQUAL_UINT8(3, frame.payload[6]); // size (lo)
  TEST_ASSERT_EQUAL_UINT8(0, frame.payload[7]); // size (hi)

  // ext msp payload
  TEST_ASSERT_EQUAL_UINT8(1, frame.payload[8]);  // param1
  TEST_ASSERT_EQUAL_UINT8(2, frame.payload[9]);  // param2
  TEST_ASSERT_EQUAL_UINT8(3, frame.payload[10]); // param3

  // crsf crc
  TEST_ASSERT_EQUAL_UINT8(0x10, frame.crc());
}

void test_crsf_decode_msp_v1()
{
  const uint8_t data[] = {0xc8, 0x08, 0x7a, 0xc8, 0xea, 0x32, 0x00, 0x70, 0x70, 0x4b};
  CrsfMessage frame;
  std::copy_n(data, sizeof(data), (uint8_t*)&frame);

  Connect::MspMessage m;
  uint8_t origin = 0;

  Crsf::decodeMsp(frame, m, origin);

  // crsf headers
  TEST_ASSERT_EQUAL_UINT8(CRSF_ADDRESS_FLIGHT_CONTROLLER, frame.addr);
  TEST_ASSERT_EQUAL_UINT8(0x08, frame.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_REQ, frame.type);

  // crsf ext headers
  TEST_ASSERT_EQUAL_UINT8(0xC8, frame.payload[0]); // FC addr
  TEST_ASSERT_EQUAL_UINT8(0xEA, frame.payload[1]); // radio-transmitter addr (origin)
  TEST_ASSERT_EQUAL_UINT8(0x32, frame.payload[2]); // status flags

  // ext msp v1 header
  TEST_ASSERT_EQUAL_UINT8(0x00, frame.payload[3]); // size
  TEST_ASSERT_EQUAL_UINT8(0x70, frame.payload[4]); // type: msp_pid(0x70)

  // ext msp payload
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_UINT8(Connect::MSP_V1, m.version);
  TEST_ASSERT_EQUAL_UINT8(0, m.received);
  TEST_ASSERT_EQUAL_UINT8(0x70, m.cmd);

  // crsf crc
  TEST_ASSERT_EQUAL_UINT8(0x4B, frame.crc());

  // origin
  TEST_ASSERT_EQUAL_UINT8(0xEA, origin);
}

void test_csrf_decode_msp_v1_fragmented()
{
  // generate example msp message into buffer
  uint8_t buff[64] = {0};
  Connect::MspResponse resp;
  resp.version = Connect::MSP_V1;
  resp.cmd = MSP_API_VERSION;
  resp.result = 0;
  for (uint8_t i = 0; i < 32; i++)
  {
    resp.writeU8(i);
  }
  resp.serialize(buff, sizeof(buff));

  // we need only range from 3 to 37
  const size_t dataLen = 34;
  const uint8_t* dataPtr = buff + 3;                 // skip msp header ($M<)
  const uint8_t flags1 = (1 << 5) | (1 << 4) | 0x00; // v(1) + start(1) + sequence(0)
  const uint8_t flags2 = (1 << 5) | (0 << 4) | 0x01; // v(1) + start(0) + sequence(1)
  const uint8_t dst = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  const uint8_t src = CRSF_ADDRESS_RADIO_TRANSMITTER;

  // create first fragmented frame
  CrsfMessage frame1;
  frame1.prepare(CRSF_FRAMETYPE_MSP_REQ);
  frame1.writeU8(dst);
  frame1.writeU8(src);
  frame1.writeU8(flags1);
  for (size_t i = 0; i < 17; i++)
  {
    frame1.writeU8(dataPtr[i]);
  }
  frame1.finalize();
  TEST_ASSERT_EQUAL_UINT8(22, frame1.size);

  // create second fragmented frame
  CrsfMessage frame2;
  frame2.prepare(CRSF_FRAMETYPE_MSP_REQ);
  frame2.writeU8(dst);
  frame2.writeU8(src);
  frame2.writeU8(flags2);
  for (size_t i = 17; i < dataLen; i++)
  {
    frame2.writeU8(dataPtr[i]);
  }
  frame2.finalize();
  TEST_ASSERT_EQUAL_UINT8(22, frame2.size);

  // decode first frame
  Connect::MspMessage m;
  uint8_t origin = 0;

  Crsf::decodeMsp(frame1, m, origin);

  TEST_ASSERT_FALSE(m.isReady()); // not ready until we get the rest of the data
  TEST_ASSERT_TRUE(m.isCmd());

  // decode second frame
  Crsf::decodeMsp(frame2, m, origin);

  // assert that we receied same message that is valid and complete
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_TRUE(m.isCmd());
  TEST_ASSERT_EQUAL_UINT8(Connect::MSP_V1, m.version);
  TEST_ASSERT_EQUAL_UINT8(MSP_API_VERSION, m.cmd);
  TEST_ASSERT_EQUAL_UINT8(32, m.received);
  TEST_ASSERT_EQUAL_UINT8(32, m.expected);

  for (uint8_t i = 0; i < 32; i++)
  {
    TEST_ASSERT_EQUAL_UINT8(i, m.buffer[i]);
  }

  TEST_ASSERT_EQUAL_UINT8(src, origin);
}

// guard bytes placed directly after the message object to detect out of bounds writes
struct __attribute__((packed)) GuardedCrsfMessage
{
  CrsfMessage msg;
  uint8_t guard_a;
  uint8_t guard_b;
};

void test_crsf_encode_msp_v1_fragmented_no_overflow()
{
  GuardedCrsfMessage g;
  memset(&g, 0, sizeof(g));
  g.guard_a = 0xAA;
  g.guard_b = 0x55;

  Connect::MspResponse resp;
  resp.version = Connect::MSP_V1;
  resp.cmd = MSP_API_VERSION;
  resp.result = 0;
  for (size_t i = 0; i < 64; i++)
  {
    resp.writeU8(i);
  }

  uint8_t buff[255];
  size_t size = resp.serialize(buff, sizeof(buff));
  const uint8_t* beg = buff + 3;        // skip msp header
  const uint8_t* end = buff + size - 1; // skip crc

  beg = Crsf::encodeMspData(g.msg, CRSF_ADDRESS_RADIO_TRANSMITTER, 1, 0, true, beg, end);
  TEST_ASSERT_FALSE(beg == end);

  // maximum size frame: size counts type + payload + crc
  TEST_ASSERT_EQUAL_UINT8(62, g.msg.size);
  TEST_ASSERT_EQUAL_UINT8(0xFE, g.msg.crc());

  // adjacent memory must remain untouched (2-byte overflow was present here)
  TEST_ASSERT_EQUAL_UINT8(0xAA, g.guard_a);
  TEST_ASSERT_EQUAL_UINT8(0x55, g.guard_b);
}

void test_input_crsf_max_size_frame_parse_no_overflow()
{
  InputCRSF input;
  GuardedCrsfMessage g;
  memset(&g, 0, sizeof(g));
  g.guard_a = 0xAA;
  g.guard_b = 0x55;

  When(Method(ArduinoFake(), micros)).Return(0);
  input.begin(nullptr, nullptr);

  // build valid maximum-size MSP_REQ frame (size = 62, wire length = 64)
  CrsfMessage t;
  memset(&t, 0, sizeof(t));
  t.prepare(CRSF_FRAMETYPE_MSP_REQ);
  t.writeU8(CRSF_ADDRESS_FLIGHT_CONTROLLER); // dst
  t.writeU8(CRSF_ADDRESS_RADIO_TRANSMITTER); // origin
  t.writeU8((1 << 5) | (1 << 4) | 0);        // status: v1 + start + seq0
  t.writeU8(200);                            // msp v1 size (fragmented)
  t.writeU8(MSP_API_VERSION);                // msp v1 cmd
  while (t.size < CRSF_FRAME_SIZE_MAX - 2)   // pad to max payload
  {
    t.writeU8(0x42);
  }
  t.finalize();
  TEST_ASSERT_EQUAL_UINT8(62, t.size);

  const uint8_t* stream = reinterpret_cast<const uint8_t*>(&t);
  for (size_t i = 0; i < t.size + 2; i++) // whole wire frame incl addr and len
  {
    input.parse(g.msg, stream[i]);
  }

  // parser must not write past the message object while collecting frame data and crc
  TEST_ASSERT_EQUAL_UINT8(0xAA, g.guard_a);
  TEST_ASSERT_EQUAL_UINT8(0x55, g.guard_b);

  // frame must be intact after parsing
  TEST_ASSERT_EQUAL_UINT8(CRSF_SYNC_BYTE, g.msg.addr);
  TEST_ASSERT_EQUAL_UINT8(62, g.msg.size);
  TEST_ASSERT_EQUAL_UINT8(CRSF_FRAMETYPE_MSP_REQ, g.msg.type);
  TEST_ASSERT_EQUAL_UINT8(stream[63], g.msg.crc());
}

void test_crsf_decode_msp_short_start_frame_ignored()
{
  // start frame too short to contain full msp v2 header (size < 9) must be rejected
  CrsfMessage frame;
  memset(&frame, 0, sizeof(frame));
  frame.addr = CRSF_SYNC_BYTE;
  frame.type = CRSF_FRAMETYPE_MSP_REQ;
  frame.size = 6; // payload area = dst, origin, status + 1 byte only
  frame.payload[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame.payload[1] = CRSF_ADDRESS_RADIO_TRANSMITTER;
  frame.payload[2] = (2 << 5) | (1 << 4) | 0; // v2 + start + seq0

  Connect::MspMessage m;
  uint8_t origin = 0;

  int ret = Crsf::decodeMsp(frame, m, origin);

  TEST_ASSERT_EQUAL_INT(0, ret);
  TEST_ASSERT_FALSE(m.isReady());
  TEST_ASSERT_EQUAL_UINT8(0, m.received);
}

void test_crsf_decode_msp_short_continuation_frame_ignored()
{
  // start frame announcing long fragmented message
  CrsfMessage frame1;
  memset(&frame1, 0, sizeof(frame1));
  frame1.addr = CRSF_SYNC_BYTE;
  frame1.type = CRSF_FRAMETYPE_MSP_REQ;
  frame1.size = 17; // ext header + msp v1 header + 10 bytes of payload
  frame1.payload[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame1.payload[1] = CRSF_ADDRESS_RADIO_TRANSMITTER;
  frame1.payload[2] = (1 << 5) | (1 << 4) | 0; // v1 + start + seq0
  frame1.payload[3] = 200;                     // msp size (fragmented)
  frame1.payload[4] = MSP_API_VERSION;         // msp cmd
  for (size_t i = 0; i < 10; i++)
  {
    frame1.payload[5 + i] = i;
  }

  Connect::MspMessage m;
  uint8_t origin = 0;

  int ret = Crsf::decodeMsp(frame1, m, origin);
  TEST_ASSERT_EQUAL_INT(0, ret);
  TEST_ASSERT_EQUAL_UINT8(10, m.received);
  TEST_ASSERT_EQUAL_UINT16(200, m.expected);

  // malformed continuation frame with size below extended header length (size < 5)
  // previously caused integer underflow and unbounded append (memory corruption)
  CrsfMessage frame2;
  memset(&frame2, 0, sizeof(frame2));
  frame2.addr = CRSF_SYNC_BYTE;
  frame2.type = CRSF_FRAMETYPE_MSP_REQ;
  frame2.size = 4;
  frame2.payload[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame2.payload[1] = CRSF_ADDRESS_RADIO_TRANSMITTER;
  frame2.payload[2] = (1 << 5) | (0 << 4) | 1; // v1 + no start + seq1

  ret = Crsf::decodeMsp(frame2, m, origin);

  TEST_ASSERT_EQUAL_INT(0, ret);
  TEST_ASSERT_FALSE(m.isReady());
  TEST_ASSERT_EQUAL_UINT8(10, m.received); // unchanged, nothing appended
}

void test_crsf_decode_msp_resets_read_on_start()
{
  Connect::MspMessage m;
  uint8_t origin = 0;

  // first complete command with payload
  CrsfMessage frame1;
  memset(&frame1, 0, sizeof(frame1));
  frame1.addr = CRSF_SYNC_BYTE;
  frame1.type = CRSF_FRAMETYPE_MSP_REQ;
  frame1.size = 12; // ext header + msp v1 header + 4 bytes of payload
  frame1.payload[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame1.payload[1] = CRSF_ADDRESS_RADIO_TRANSMITTER;
  frame1.payload[2] = (1 << 5) | (1 << 4) | 0; // v1 + start + seq0
  frame1.payload[3] = 4;                       // msp size
  frame1.payload[4] = MSP_API_VERSION;         // msp cmd
  for (size_t i = 0; i < 4; i++)
  {
    frame1.payload[5 + i] = 10 + i;
  }

  int ret = Crsf::decodeMsp(frame1, m, origin);
  TEST_ASSERT_EQUAL_INT(1, ret);
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_UINT8(4, m.received);

  // simulate consumption by msp processor
  m.read = m.received;
  TEST_ASSERT_EQUAL_UINT8(4, m.read);

  // second command must be decoded from clean state, read offset included,
  // otherwise processor would consume stale bytes from previous message
  CrsfMessage frame2;
  memset(&frame2, 0, sizeof(frame2));
  frame2.addr = CRSF_SYNC_BYTE;
  frame2.type = CRSF_FRAMETYPE_MSP_REQ;
  frame2.size = 11; // ext header + msp v1 header + 3 bytes of payload
  frame2.payload[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  frame2.payload[1] = CRSF_ADDRESS_RADIO_TRANSMITTER;
  frame2.payload[2] = (1 << 5) | (1 << 4) | 1; // v1 + start + seq1
  frame2.payload[3] = 3;                       // msp size
  frame2.payload[4] = MSP_FC_VARIANT;          // msp cmd
  for (size_t i = 0; i < 3; i++)
  {
    frame2.payload[5 + i] = 20 + i;
  }

  ret = Crsf::decodeMsp(frame2, m, origin);
  TEST_ASSERT_EQUAL_INT(1, ret);
  TEST_ASSERT_TRUE(m.isReady());
  TEST_ASSERT_EQUAL_UINT8(3, m.received);
  TEST_ASSERT_EQUAL_UINT16(MSP_FC_VARIANT, m.cmd);
  TEST_ASSERT_EQUAL_UINT8(0, m.read); // must point at message start again
}

void test_input_ibus_rc_valid()
{
  InputIBUS input;
  InputIBUS::IBusData frame;
  memset(&frame, 0, sizeof(frame));
  uint8_t* frame_data = reinterpret_cast<uint8_t*>(&frame);

  When(Method(ArduinoFake(), micros)).Return(0);

  input.begin(nullptr);

  // const uint8_t data[] = {
  //   0x20, 0x40,  // preambule (len, cmd)
  //   0xDC, 0x05,  0xDC, 0x05,  0xBE, 0x05,  0xDC, 0x05, // channel 1-4
  //   0xD0, 0x07,  0xD0, 0x07,  0xDC, 0x05,  0xDC, 0x05, // channel 5-8
  //   0xDC, 0x05,  0xDC, 0x05,  0xDC, 0x05,  0xDC, 0x05, // channel 9-12
  //   0xDC, 0x05,  0xDC, 0x05,  // channel 13-14
  //   0x83, 0xF3  // checksum
  // };

  const uint8_t data[] = {
      0x20, 0x40, 0xDB, 0x05, 0xDC, 0x05, 0x54, 0x05, 0xDC, 0x05, 0xE8, 0x03, 0xD0, 0x07, 0xD2, 0x05,
      0xE8, 0x03, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDA, 0xF3,
  };
  for (size_t i = 0; i < sizeof(data); i++)
  {
    input.parse(frame, data[i]);
  }

  for (size_t i = 0; i < sizeof(data); i++)
  {
    TEST_ASSERT_EQUAL_HEX8(data[i], frame_data[i]);
  }

  TEST_ASSERT_EQUAL_HEX16(0xF3DA, frame.checksum);

  TEST_ASSERT_EQUAL_HEX8(0x20, frame.len);
  TEST_ASSERT_EQUAL_HEX8(0x40, frame.cmd);

  TEST_ASSERT_EQUAL_UINT16(1499, frame.ch[0]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[1]);
  TEST_ASSERT_EQUAL_UINT16(1364, frame.ch[2]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[3]);
  TEST_ASSERT_EQUAL_UINT16(1000, frame.ch[4]);
  TEST_ASSERT_EQUAL_UINT16(2000, frame.ch[5]);
  TEST_ASSERT_EQUAL_UINT16(1490, frame.ch[6]);
  TEST_ASSERT_EQUAL_UINT16(1000, frame.ch[7]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[8]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[9]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[10]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[11]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[12]);
  TEST_ASSERT_EQUAL_UINT16(1500, frame.ch[13]);

  TEST_ASSERT_EQUAL_UINT16(1499, input.get(0));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(1));
  TEST_ASSERT_EQUAL_UINT16(1364, input.get(2));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(3));
  TEST_ASSERT_EQUAL_UINT16(1000, input.get(4));
  TEST_ASSERT_EQUAL_UINT16(2000, input.get(5));
  TEST_ASSERT_EQUAL_UINT16(1490, input.get(6));
  TEST_ASSERT_EQUAL_UINT16(1000, input.get(7));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(8));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(9));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(10));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(11));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(12));
  TEST_ASSERT_EQUAL_UINT16(1500, input.get(13));
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_input_crsf_rc_valid);
  RUN_TEST(test_input_crsf_rc_valid_no_payload);
  RUN_TEST(test_input_crsf_rc_prefix);
  RUN_TEST(test_crsf_encode_rc);
  RUN_TEST(test_crsf_decode_rc_struct);
  RUN_TEST(test_crsf_decode_rc_shift8);
  // RUN_TEST(test_crsf_decode_rc_shift32);
  RUN_TEST(test_crsf_encode_tlm);
  RUN_TEST(test_crsf_encode_msp_v1);
  RUN_TEST(test_crsf_encode_msp_v1_fragmented);
  RUN_TEST(test_crsf_encode_msp_v1_fragmented_no_overflow);
  RUN_TEST(test_input_crsf_max_size_frame_parse_no_overflow);
  RUN_TEST(test_crsf_decode_msp_short_start_frame_ignored);
  RUN_TEST(test_crsf_decode_msp_short_continuation_frame_ignored);
  RUN_TEST(test_crsf_decode_msp_resets_read_on_start);
  RUN_TEST(test_crsf_encode_msp_v2);
  RUN_TEST(test_crsf_decode_msp_v1);
  RUN_TEST(test_csrf_decode_msp_v1_fragmented);
  RUN_TEST(test_input_ibus_rc_valid);

  return UNITY_END();
}