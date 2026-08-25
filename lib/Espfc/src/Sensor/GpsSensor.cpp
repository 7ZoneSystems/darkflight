#include "Sensor/GpsSensor.hpp"
#include <Arduino.h>
#include <Gps.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <tuple>

namespace Espfc::Sensor {

static constexpr std::array<int, 6> BAUDS{
    9600, 115200, 230400, 57600, 38400, 19200,
};

static constexpr std::array<uint16_t, 6> NMEA_MSG_OFF{
    Gps::NMEA_MSG_GGA, Gps::NMEA_MSG_GLL, Gps::NMEA_MSG_GSA, Gps::NMEA_MSG_GSV, Gps::NMEA_MSG_RMC, Gps::NMEA_MSG_VTG,
};

// legacy-protocol modules that already speak NAV-PVT/NAV-SAT (u-blox 6 and unknown)
static constexpr std::array<std::tuple<uint16_t, uint8_t>, 2> UBX_LEGACY_MODERN_MSG_ON{{
    {Gps::UBX_NAV_PVT, 1u},
    {Gps::UBX_NAV_SAT, 10u},
}};

static constexpr std::array<std::tuple<uint16_t, uint8_t>, 4> UBX_LEGACY_MSG_ON{
    std::make_tuple(Gps::UBX_NAV_POSLLH, 1u),
    std::make_tuple(Gps::UBX_NAV_VELNED, 1u),
    std::make_tuple(Gps::UBX_NAV_SOL, 1u),
    std::make_tuple(Gps::UBX_NAV_SVINFO, 1u),
};

GpsSensor::GpsSensor(Model& model): _model(model) {}

int GpsSensor::begin(Device::SerialDevice* port, int baud)
{
  _port = port;
  _targetBaud = _currentBaud = baud;
  _timer.setRate(50);

  _state = DETECT_BAUD;
  _timeout = micros() + DETECT_TIMEOUT;
  _counter = 0;
  setBaud(_targetBaud);

  return 1;
}

int GpsSensor::update()
{
  if (!_port) return 0;

  if (!_timer.check()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_GPS_READ);

  bool updated = false;
  uint8_t buff[32];
  size_t read = 0;
  while ((read = _port->readMany(buff, sizeof(buff))))
  {
    for (size_t i = 0; i < read; i++)
    {
      updated |= processUbx(buff[i]);
      processNmea(buff[i]);
    }
  }

  if (!updated) handle();

  return 1;
}

bool GpsSensor::processUbx(uint8_t c)
{
  _ubxParser.parse(c, _ubxMsg);
  if (!_ubxMsg.isReady()) return false;

  onMessage();

  handle();
  _ubxMsg = Gps::UbxMessage();

  return true;
}

void GpsSensor::processNmea(uint8_t c)
{
  _nmeaParser.parse(c, _nmeaMsg);
  if (!_nmeaMsg.isReady()) return;

  //$GNTXT,01,01,01,More than 100 frame errors, UART RX was disabled*70
  static const char* msg = "GNTXT,01,01,01,More than 100 frame errors";

  if (!_model.state.gps.frameError && std::strncmp(_nmeaMsg.payload, msg, std::strlen(msg)) == 0)
  {
    _model.state.gps.frameError = true;
    if (!_model.isModeActive(MODE_ARMED)) _model.logger.err().logln("GPS RX Frame Err");
  }

  if (_model.config.gps.provider == 0)
  {
    handleNmeaSentence();
  }

  onMessage();

  _nmeaMsg = Gps::NmeaMessage();
}

void GpsSensor::onMessage()
{
  if (_state == DETECT_BAUD)
  {
    // NMEA-only receivers never speak UBX; skip version/config handshake.
    _state = _model.config.gps.provider == 0 ? RECEIVE : GET_VERSION;
    _model.logger.info().log("GPS DET").logln(_currentBaud);
  }
}

void GpsSensor::handle()
{
  switch (_state)
  {
    case DETECT_BAUD:
      detectBaud();
      break;

    case GET_VERSION:
      readVersion();
      break;

    case CONFIGURE_BAUD:
      configureBaud();
      break;

    case DISABLE_NMEA:
      disableNmea();
      break;

    case ENABLE_UBX:
      enableUbx();
      break;

    case ENABLE_NAV5:
      enableNav5();
      break;

    case ENABLE_SBAS:
      enableSbas();
      break;

    case DETECT_GPS_L5:
      detectGpsL5();
      break;

    case CONFIGURE_GNSS:
      configureGnss();
      break;

    case CONFIGURE_NAV_RATE:
      configureRate();
      break;

    case ERROR:
      handleError();
      break;

    case RECEIVE:
    case WAIT:
    default:
      handleReceive();
      break;
  }
}

void GpsSensor::handleReceive()
{
  if (_state == RECEIVE)
  {
    _model.state.gps.present = true;
  }

  if (_ubxMsg.isReady())
  {
    if (_ubxMsg.isAck())
    {
      _state = _ackState;
    }
    else if (_ubxMsg.isNak())
    {
      _state = _timeoutState;
      _model.logger.err().log("GPS NAK").loghex(_ubxMsg.payload[0]).loghex(_ubxMsg.payload[1]).endl();
    }
    else if (_ubxMsg.isResponse(Gps::UBX_CFG_VALGET))
    {
      handleCfgValGet();
      _state = _ackState;
      _counter = 0;
    }
    else if (_ubxMsg.isResponse(Gps::UbxMonVer::ID))
    {
      handleVersion();
      _state = _ackState;
      _counter = 0;
    }
    else if (_ubxMsg.isResponse(Gps::UbxNavPvt92::ID))
    {
      handleNavPvt();
    }
    else if (_ubxMsg.isResponse(Gps::UbxNavPosLlh28::ID) && _ubxMsg.length >= sizeof(Gps::UbxNavPosLlh28))
    {
      handleNavPosLlh();
    }
    else if (_ubxMsg.isResponse(Gps::UbxNavVelned36::ID) && _ubxMsg.length >= sizeof(Gps::UbxNavVelned36))
    {
      handleNavVelned();
    }
    else if (_ubxMsg.isResponse(Gps::UbxNavSol52::ID) && _ubxMsg.length >= sizeof(Gps::UbxNavSol52))
    {
      handleNavSol();
    }
    else if (_ubxMsg.isResponse(Gps::UbxNavSvInfo::ID) && _ubxMsg.length >= sizeof(Gps::UbxNavSvInfo))
    {
      handleNavSvInfo();
    }
    else if (_ubxMsg.isResponse(Gps::UbxNavSat::ID))
    {
      handleNavSat();
    }
  }
  else if (_state == WAIT && micros() > _timeout)
  {
    // timeout
    _state = _timeoutState;
    _model.state.gps.present = false;
    _model.logger.err().logln("GPS TOUT");
  }
}

void GpsSensor::detectBaud()
{
  if (micros() > _timeout)
  {
    // on timeout check next baud
    if (_counter < BAUDS.size())
    {
      setBaud(BAUDS[_counter]);
      _counter++;
    }
    else
    {
      _state = ERROR; // detection falied, give up
      // _state = DETECT_BAUD; // restart detection if all baud rates failed
      _counter = 0;
      setBaud(_targetBaud);
    }
    _timeout = micros() + DETECT_TIMEOUT;
  }
}

void GpsSensor::readVersion()
{
  send(Gps::UbxMonVer{}, _model.config.gps.autoConfig ? CONFIGURE_BAUD : RECEIVE); // version handled in WAIT/RECEIVE
  _timeout = micros() + 3 * TIMEOUT;
}

void GpsSensor::configureBaud()
{
  if (!_model.config.gps.autoBaud)
  {
    setState(DISABLE_NMEA);
    return;
  }

  if (isLegacyProto())
  {
    send(
        Gps::UbxCfgPrt20{
            .portId = 1,
            .resered1 = 0,
            .txReady = 0,
            .mode = 0x08c0,                    // 8N1
            .baudRate = (uint32_t)_targetBaud, // baud
            .inProtoMask = 0x07,
            .outProtoMask = 0x07,
            .flags = 0,
            .resered2 = 0,
        },
        DISABLE_NMEA, DISABLE_NMEA); // we may not be able to receive ACK for this message
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01}); // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_UART1_BAUDRATE, uint32_t>(_targetBaud));
    send(req, DISABLE_NMEA, DISABLE_NMEA);
  }
  delay(30); // wait until transmission complete at 9600bps in worst case
  setBaud(_targetBaud);
  delay(5);
}

void GpsSensor::disableNmea()
{
  if (isLegacyProto())
  {
    const Gps::UbxCfgMsg3 m{
        .msgId = NMEA_MSG_OFF[_counter],
        .rate = 0,
    };
    _counter++;
    if (_counter < NMEA_MSG_OFF.size())
    {
      send(m, _state);
    }
    else
    {
      _counter = 0;
      send(m, ENABLE_UBX);
      _model.logger.info().logln("GPS NMEA OFF");
    }
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01}); // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GGA_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GLL_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GSA_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GSV_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_RMC_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_VTG_UART1, bool>(0));
    send(req, ENABLE_UBX);
    _model.logger.info().logln("GPS NMEA* OFF");
  }
}

void GpsSensor::enableUbx()
{
  if (isLegacyProto())
  {
    if (_model.state.gps.support.version == GPS_M6 || _model.state.gps.support.version == GPS_UNKNOWN)
    {
      const Gps::UbxCfgMsg3 m{
          .msgId = std::get<0>(UBX_LEGACY_MODERN_MSG_ON[_counter]),
          .rate = std::get<1>(UBX_LEGACY_MODERN_MSG_ON[_counter]),
      };
      _counter++;
      if (_counter < UBX_LEGACY_MODERN_MSG_ON.size())
      {
        send(m, _state);
      }
      else
      {
        send(m, ENABLE_NAV5);
        _counter = 0;
        _timeout = micros() + 10 * TIMEOUT;
        _model.logger.info().logln("GPS UBX ON");
      }
    }
    else
    {
      const Gps::UbxCfgMsg3 m{
          .msgId = std::get<0>(UBX_LEGACY_MSG_ON[_counter]),
          .rate = std::get<1>(UBX_LEGACY_MSG_ON[_counter]),
      };
      _counter++;
      if (_counter < UBX_LEGACY_MSG_ON.size())
      {
        send(m, _state);
      }
      else
      {
        send(m, ENABLE_NAV5);
        _counter = 0;
        _timeout = micros() + 10 * TIMEOUT;
        _model.logger.info().logln("GPS UBX ON");
      }
    }
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01}); // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_UBX_NAV_PVT_UART1, uint8_t>(1));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_UBX_NAV_SAT_UART1, uint8_t>(10));
    send(req, ENABLE_NAV5); // if supported we get ACK, then go to get_version, else try legacy disable_nmea commands
    _model.logger.info().logln("GPS UBX* ON");
  }
}

void GpsSensor::enableNav5()
{
  if (isLegacyProto())
  {
    send(
        Gps::UbxCfgNav5{
            .mask = {.value = 0xffff}, // all
            .dynModel = 8,             // airborne
            .fixMode = 3,
            .fixedAlt = 0,
            .fixedAltVar = 10000,
            .minElev = 5,
            .drLimit = 0,
            .pDOP = 250,
            .tDOP = 250,
            .pAcc = 100,
            .tAcc = 300,
            .staticHoldThresh = 0,
            .dgnssTimeout = 60,
            .cnoThreshNumSVs = 0,
            .cnoThresh = 0,
            .reserved0 = {0, 0},
            .staticHoldMaxDist = 200,
            .utcStandard = 0,
            .reserved1 = {0, 0, 0, 0, 0},
        },
        ENABLE_SBAS);
    _model.logger.info().logln("GPS NAV5");
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});       // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_NAVSPG_DYNMODEL, uint8_t>(8)); // airborne
    send(req, ENABLE_SBAS);
    _model.logger.info().logln("GPS NAVSPG*");
  }
}

void GpsSensor::enableSbas()
{
  const bool useSbas = _model.config.gps.sbasMode == 0 ? _model.config.gps.enableSBAS : _model.config.gps.sbasMode > 1;
  if (_model.state.gps.support.sbas && useSbas)
  {
    if (isLegacyProto())
    {
      send(
          Gps::UbxCfgSbas8{
              .mode = 1,
              .usage = 1,
              .maxSbas = 3,
              .scanmode2 = 0,
              .scanmode1 = 0,
          },
          DETECT_GPS_L5);
      _model.logger.info().logln("GPS SBAS");
    }
    else
    {
      Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
      req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});         // RAM only
      req.write(Gps::UbxCfgValsetItem<Gps::CFG_SBAS_PRNSCANMASK, uint64_t>(0)); // all
      send(req, DETECT_GPS_L5);
      _model.logger.info().logln("GPS SBAS*");
    }
  }
  else
  {
    setState(DETECT_GPS_L5);
  }
}

void GpsSensor::detectGpsL5()
{
  if (_model.state.gps.support.version == GPS_M6)
  {
    setState(CONFIGURE_GNSS);
    return;
  }
  Gps::UbxRequest req(Gps::UBX_CFG_VALGET);
  req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01}); // RAM only
  req.write(Gps::CFG_SIGNAL_GPS_L5);
  send(req, CONFIGURE_GNSS, CONFIGURE_GNSS); // if supported we get ACK with value, else timeout and continue with GNSS
                                             // configuration without L5 support
}

void GpsSensor::configureRate()
{
  uint16_t mRate = 200;
  if (_currentBaud > 100000) mRate = 100;
  if (_model.state.gps.support.version == GPS_M10 && _currentBaud > 200000) mRate = 40; // (proto<24 => >50ms)
  const uint16_t nRate = 1;

  if (isLegacyProto())
  {
    const Gps::UbxCfgRate6 m{
        .measRate = mRate,
        .navRate = nRate,
        .timeRef = 0, // utc
    };
    send(m, RECEIVE);
    _model.logger.info().log("GPS NAVRATE").log(mRate).log(nRate).logln(1000 / mRate);
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01}); // RAM only
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_RATE_MEAS, uint16_t>(mRate));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_RATE_NAV, uint16_t>(nRate));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_RATE_TIMEREF, uint8_t>(0)); // utc
    send(req, RECEIVE);
    _model.logger.info().log("GPS NAVRATE*").log(mRate).log(nRate).logln(1000 / mRate);
  }
}

void GpsSensor::setBaud(int baud)
{
  if (baud != _currentBaud)
  {
    _port->updateBaudRate(baud);
    _currentBaud = baud;
    _model.logger.info().log("GPS BAUD").logln(baud);
  }
}

void GpsSensor::setState(State state, State ackState, State timeoutState)
{
  setState(state);
  _ackState = ackState;
  _timeoutState = timeoutState;
}

void GpsSensor::setState(State state)
{
  _state = state;
  _timeout = micros() + TIMEOUT;
}

void GpsSensor::handleError()
{
  if (_counter == 0)
  {
    _model.logger.err().logln("GPS ERROR");
    _counter++;
  }
  _model.state.gps.present = false;
}

void GpsSensor::configureGnss()
{
  const bool useDualBand = _model.config.gps.enableDualBand && _model.state.gps.support.gpsL5;
  bool enableGPS = _model.config.gps.enableGPS;
  bool enableGLO = _model.config.gps.enableGLONASS;
  bool enableGAL = _model.config.gps.enableGalileo;
  bool enableBDS = _model.config.gps.enableBeiDou;
  bool enableQZSS = _model.config.gps.enableQZSS;
  bool enableSBAS = _model.config.gps.enableSBAS;

  const auto& support = _model.state.gps.support;
  switch (_model.config.gps.gnssMode)
  {
    case 1:
      enableGPS = support.gps;
      enableGLO = enableGAL = enableBDS = enableQZSS = false;
      break;
    case 2:
      enableGPS = support.gps;
      enableGLO = support.glonass;
      enableGAL = enableBDS = enableQZSS = false;
      break;
    case 3:
      enableGPS = support.gps;
      enableGAL = support.galileo;
      enableGLO = enableBDS = enableQZSS = false;
      break;
    case 4:
      enableGPS = support.gps;
      enableBDS = support.beidou;
      enableGLO = enableGAL = enableQZSS = false;
      break;
    case 5:
      enableGPS = support.gps;
      enableGLO = support.glonass;
      enableGAL = support.galileo;
      enableBDS = support.beidou;
      enableQZSS = support.qzss;
      break;
  }

  size_t written = 0;
  if (isLegacyProto())
  {
    // const Gps::UbxCfgGnss7 gnss{
    //   .msgVer = 0,
    //   .numTrkChHw = 0,
    //   .numTrkChUse = 0xFF,
    //   .numConfigBlocks = 7,
    //   .blocks = {
    //     // GPS: L1C/A or L1+L5
    //     { 0x00, 0x08, 0x10, 0x00, (uint8_t)(enableGPS  ? 0x01 : 0x00), 0x00, (uint8_t)(useDualBand ? 0x20 : 0x01),
    //     0x01 },
    //     // SBAS: L1C/A
    //     { 0x01, 0x01, 0x03, 0x00, (uint8_t)(enableSBAS ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
    //     // Galileo: E1 or E1+E5a
    //     { 0x02, 0x04, 0x08, 0x00, (uint8_t)(enableGAL  ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
    //     // BeiDou: B1I or B1I+B2a
    //     { 0x03, 0x08, 0x10, 0x00, (uint8_t)(enableBDS  ? 0x01 : 0x00), 0x00, (uint8_t)(false && useDualBand ? 0x80 :
    //     0x01), 0x01 },
    //     // IMES: disabled
    //     { 0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x01, 0x01 },
    //     // QZSS: L1C/A or L1+L5
    //     { 0x05, 0x00, 0x03, 0x00, (uint8_t)(enableQZSS ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
    //     // GLONASS: L1
    //     { 0x06, 0x08, 0x0E, 0x00, (uint8_t)(enableGLO  ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
    //   },
    // };
    // written = sizeof(gnss);
    // send(gnss, CONFIGURE_NAV_RATE);
    Gps::UbxRequest req{Gps::UBX_CFG_GNSS};
    uint8_t numBlocks = _model.state.gps.support.gps + _model.state.gps.support.sbas +
                        _model.state.gps.support.galileo + _model.state.gps.support.beidou +
                        _model.state.gps.support.qzss + _model.state.gps.support.glonass +
                        _model.state.gps.support.imes;

    if (numBlocks == 0)
    {
      _model.logger.info().logln("GPS GNSS LEGACY DEFAULT");
      setState(CONFIGURE_NAV_RATE);
      return;
    }

    written += req.write(
        Gps::UbxCfgGnssHeader{.msgVer = 0, .numTrkChHw = 32, .numTrkChUse = 0xff, .numConfigBlocks = numBlocks});
    if (_model.state.gps.support.gps)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 0,
                                                .resTrkCh = 8,
                                                .maxTrkCh = 16,
                                                .flagsEnable = enableGPS,
                                                .sigCfgMask = (uint8_t)(useDualBand ? 0x20 : 0x01),
                                                .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.sbas)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 1, .resTrkCh = 1, .maxTrkCh = 3, .flagsEnable = enableSBAS, .sigCfgMask = 0x01, .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.galileo)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 2, .resTrkCh = 4, .maxTrkCh = 8, .flagsEnable = enableGAL, .sigCfgMask = 0x01, .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.beidou)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 3,
                                                .resTrkCh = 8,
                                                .maxTrkCh = 16,
                                                .flagsEnable = enableBDS,
                                                .sigCfgMask = (uint8_t)(false ? 0x80 : 0x01),
                                                .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.imes)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 4, .resTrkCh = 0, .maxTrkCh = 8, .flagsEnable = 0, .sigCfgMask = 0x01, .flagsHigh = 0x03});
    }
    if (_model.state.gps.support.qzss)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 5, .resTrkCh = 0, .maxTrkCh = 3, .flagsEnable = enableQZSS, .sigCfgMask = 0x01, .flagsHigh = 0x05});
    }
    if (_model.state.gps.support.glonass)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 6, .resTrkCh = 8, .maxTrkCh = 14, .flagsEnable = enableGLO, .sigCfgMask = 0x01, .flagsHigh = 0x01});
    }
    send(req, CONFIGURE_NAV_RATE);
  }
  else
  {
    Gps::UbxRequest req{Gps::UBX_CFG_VALSET};
    written += req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01}); // RAM only
    if (_model.state.gps.support.gps)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GPS_ENA, bool>(enableGPS));
    }
    if (_model.state.gps.support.sbas)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_SBAS_ENA, bool>(enableSBAS));
    }
    if (_model.state.gps.support.galileo)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GAL_ENA, bool>(enableGAL));
    }
    if (_model.state.gps.support.qzss)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_QZSS_ENA, bool>(enableQZSS));
    }
    if (_model.state.gps.support.glonass)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GLO_ENA, bool>(enableGLO));
    }
    if (_model.state.gps.support.beidou)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_BDS_ENA, bool>(enableBDS));
    }
    if (useDualBand && _model.state.gps.support.gps)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GPS_L5, bool>(useDualBand));
    }
    // there is no guarantion that gps and baidou are supported together.
    // if (useDualBand && _model.state.gps.support.beidou)
    // {
    //   written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_BDS_B2A, bool>(useDualBand));
    // }
    send(req, CONFIGURE_NAV_RATE);
  }

  _model.logger.info().log("GPS GNSS");
  if (isLegacyProto()) _model.logger.log("LEGACY");
  if (enableGPS) _model.logger.log("GPS");
  if (useDualBand) _model.logger.log("L1+L5");
  if (enableGLO) _model.logger.log("GLO");
  if (enableGAL) _model.logger.log("GAL");
  if (enableBDS) _model.logger.log("BDS");
  if (enableSBAS) _model.logger.log("SBAS");
  if (enableQZSS) _model.logger.log("QZSS");
  _model.logger.logln(written);
}

void GpsSensor::calculateHomeVector() const
{
  if (!_model.state.gps.isHomeValid())
  {
    _model.state.gps.distanceToHome = 0;
    _model.state.gps.directionToHome = 0;
    return;
  }

  const int32_t lat1 = _model.state.gps.location.home.lat;
  const int32_t lon1 = _model.state.gps.location.home.lon;
  const int32_t lat2 = _model.state.gps.location.raw.lat;
  const int32_t lon2 = _model.state.gps.location.raw.lon;

  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(lat1, lon1, lat2, lon2);

  _model.state.gps.distanceToHome = distance;
  _model.state.gps.directionToHome = bearing;
}

void GpsSensor::handleCfgValGet() const
{
  const uint32_t key = *(reinterpret_cast<const uint32_t*>(_ubxMsg.payload) + sizeof(Gps::UbxCfgValsetHeader));
  if (key == Gps::CFG_SIGNAL_GPS_L5)
  {
    _model.state.gps.support.gpsL5 = true;
    _model.logger.info().logln("GPS DET L5");
  }
}

void GpsSensor::handleNavPvt() const
{
  const auto& m = *_ubxMsg.getAs<Gps::UbxNavPvt92>();

  _model.state.gps.fix = m.fixType == 3 && m.flags.gnssFixOk;
  _model.state.gps.fixType = m.fixType;
  _model.state.gps.numSats = m.numSV;

  _model.state.gps.accuracy.pDop = m.pDOP;
  _model.state.gps.accuracy.horizontal = m.hAcc; // mm
  _model.state.gps.accuracy.vertical = m.vAcc;   // mm
  _model.state.gps.accuracy.heading = m.headAcc; // deg * 1e5

  _model.state.gps.location.raw.lat = m.lat;
  _model.state.gps.location.raw.lon = m.lon;
  _model.state.gps.location.raw.height = m.hSML;

  _model.state.gps.velocity.raw.groundSpeed = m.gSpeed * 10;
  _model.state.gps.velocity.raw.heading = m.headMot;

  // NAV-PVT velocities (velN/velE/velD/gSpeed) and sAcc are cm/s per the
  // u-blox interface manual; the shared GPS state uses mm/s like NAV-VELNED.
  _model.state.gps.velocity.raw.north = m.velN * 10;
  _model.state.gps.velocity.raw.east = m.velE * 10;
  _model.state.gps.velocity.raw.down = m.velD * 10;
  _model.state.gps.velocity.raw.speed3d =
      lrintf(std::hypot(static_cast<float>(_model.state.gps.velocity.raw.groundSpeed),
                        static_cast<float>(_model.state.gps.velocity.raw.down)));
  _model.state.gps.accuracy.speed = m.sAcc * 10;

  if (m.valid.validDate && m.valid.validTime)
  {
    _model.state.gps.dateTime.year = m.year;
    _model.state.gps.dateTime.month = m.month;
    _model.state.gps.dateTime.day = m.day;
    _model.state.gps.dateTime.hour = m.hour;
    _model.state.gps.dateTime.minute = m.min;
    _model.state.gps.dateTime.second = m.sec;
    int32_t msec = m.nano / 1000000;
    if (msec < 0)
    {
      msec += 1000;
    }
    _model.state.gps.dateTime.msec = msec;
  }

  uint32_t now = micros();
  _model.state.gps.interval = now - _model.state.gps.lastMsgTs;
  _model.state.gps.lastMsgTs = now;

  calculateHomeVector();
}

void GpsSensor::handleNavSat() const
{
  if (_ubxMsg.length < sizeof(Gps::UbxNavSat)) return; // truncated header
  const auto& m = *_ubxMsg.getAs<Gps::UbxNavSat>();
  // only trust satellite entries actually delivered in this message
  const size_t available = (_ubxMsg.length - sizeof(Gps::UbxNavSat)) / sizeof(m.sats[0]);
  _model.state.gps.numCh = std::min<size_t>({m.numSvs, SAT_MAX, available});
  for (uint8_t i = 0; i < SAT_MAX; i++)
  {
    if (i < _model.state.gps.numCh)
    {
      _model.state.gps.svinfo[i].id = m.sats[i].svId;
      _model.state.gps.svinfo[i].gnssId = m.sats[i].gnssId;
      _model.state.gps.svinfo[i].cno = m.sats[i].cno;
      _model.state.gps.svinfo[i].quality.value = m.sats[i].flags.value;
    }
    else
    {
      _model.state.gps.svinfo[i] = GpsSatelite{};
    }
  }
}

void GpsSensor::handleNavPosLlh() const
{
  const auto& m = *_ubxMsg.getAs<Gps::UbxNavPosLlh28>();
  _model.state.gps.location.raw.lat = m.lat;
  _model.state.gps.location.raw.lon = m.lon;
  _model.state.gps.location.raw.height = m.height;
  _model.state.gps.accuracy.horizontal = m.hAcc;
  _model.state.gps.accuracy.vertical = m.vAcc;
  calculateHomeVector();
}

void GpsSensor::handleNavVelned() const
{
  const auto& m = *_ubxMsg.getAs<Gps::UbxNavVelned36>();
  // NAV-VELNED uses cm/s; the common GPS state uses mm/s like NAV-PVT.
  _model.state.gps.velocity.raw.north = m.velN * 10;
  _model.state.gps.velocity.raw.east = m.velE * 10;
  _model.state.gps.velocity.raw.down = m.velD * 10;
  _model.state.gps.velocity.raw.speed3d = m.speed * 10;
  _model.state.gps.velocity.raw.groundSpeed = m.gSpeed * 10;
  _model.state.gps.velocity.raw.heading = m.heading;
  _model.state.gps.accuracy.speed = m.sAcc * 10;
  _model.state.gps.accuracy.heading = m.cAcc;

  const uint32_t now = micros();
  _model.state.gps.interval = now - _model.state.gps.lastMsgTs;
  _model.state.gps.lastMsgTs = now;
  calculateHomeVector();
}

void GpsSensor::handleNavSol() const
{
  const auto& m = *_ubxMsg.getAs<Gps::UbxNavSol52>();
  _model.state.gps.fixType = m.gpsFix;
  _model.state.gps.fix = m.gpsFix == 3 && (m.flags & 0x01);
  _model.state.gps.numSats = m.numSV;
  _model.state.gps.accuracy.pDop = m.pDOP;
  _model.state.gps.accuracy.horizontal = m.pAcc * 10;
  _model.state.gps.accuracy.speed = m.sAcc * 10;
}

void GpsSensor::handleNavSvInfo() const
{
  const auto& m = *_ubxMsg.getAs<Gps::UbxNavSvInfo>();
  const size_t available = (_ubxMsg.length - sizeof(Gps::UbxNavSvInfo)) / sizeof(m.sats[0]);
  _model.state.gps.numCh = static_cast<uint8_t>(std::min<size_t>(std::min<size_t>(m.numCh, SAT_MAX), available));
  for (size_t i = 0; i < SAT_MAX; i++)
  {
    if (i < _model.state.gps.numCh)
    {
      const auto& sat = m.sats[i];
      _model.state.gps.svinfo[i].id = sat.svid;
      _model.state.gps.svinfo[i].gnssId = 0; // u-blox 6 SVINFO is GPS-only.
      _model.state.gps.svinfo[i].cno = sat.cno;
      _model.state.gps.svinfo[i].quality.value = sat.quality;
    }
    else
    {
      _model.state.gps.svinfo[i] = GpsSatelite{};
    }
  }
}

static int32_t nmeaCoordinateToRaw(const char* field, char hemisphere)
{
  // ddmm.mmmm (lat) / dddmm.mmmm (lon): degrees are all digits except the
  // last two whole-minute digits before the decimal point.
  const char* dot = std::strchr(field, '.');
  if (!dot || dot - field < 2) return 0;

  char degBuf[4] = {0};
  const size_t degLen = (dot - field) - 2;
  std::memcpy(degBuf, field, std::min(degLen, sizeof(degBuf) - 1));

  const double degrees = std::atoi(degBuf);
  const double minutes = std::atof(field + degLen);
  double value = degrees + minutes / 60.0;
  if (hemisphere == 'S' || hemisphere == 'W') value = -value;

  return (int32_t)lrint(value * 1e7);
}

void GpsSensor::handleNmeaSentence()
{
  char* payload = _nmeaMsg.payload;
  char* star = std::strchr(payload, '*');
  if (star) *star = '\0'; // drop the checksum suffix before tokenizing

  static constexpr size_t MAX_FIELDS = 20;
  char* fields[MAX_FIELDS];
  size_t count = 0;

  char* tok = payload;
  while (tok && count < MAX_FIELDS)
  {
    fields[count++] = tok;
    char* comma = std::strchr(tok, ',');
    if (!comma) break;
    *comma = '\0';
    tok = comma + 1;
  }

  if (count == 0 || std::strlen(fields[0]) < 5) return;
  const char* type = fields[0] + 2; // skip 2-char talker id (GP/GN/GL/GA/GB)

  if (std::strncmp(type, "GGA", 3) == 0)
    handleNmeaGga(fields, count);
  else if (std::strncmp(type, "RMC", 3) == 0)
    handleNmeaRmc(fields, count);
  else if (std::strncmp(type, "GSA", 3) == 0)
    handleNmeaGsa(fields, count);
  else if (std::strncmp(type, "GSV", 3) == 0)
    handleNmeaGsv(fields, count);
}

void GpsSensor::handleNmeaGga(char** f, size_t n) const
{
  if (n < 10) return;

  const int fixQuality = std::atoi(f[6]); // 0=none,1=GPS,2=DGPS
  _model.state.gps.fix = fixQuality > 0;
  _model.state.gps.numSats = (uint8_t)std::atoi(f[7]);

  if (fixQuality > 0 && f[2][0] && f[4][0])
  {
    _model.state.gps.location.raw.lat = nmeaCoordinateToRaw(f[2], f[3][0]);
    _model.state.gps.location.raw.lon = nmeaCoordinateToRaw(f[4], f[5][0]);
    _model.state.gps.location.raw.height = lrintf(std::atof(f[9]) * 1000.0f); // m -> mm

    const uint32_t now = micros();
    _model.state.gps.interval = now - _model.state.gps.lastMsgTs;
    _model.state.gps.lastMsgTs = now;
    calculateHomeVector();
  }

  // NMEA carries no direct accuracy figure; approximate horizontal accuracy
  // from HDOP (~5m per HDOP unit, typical consumer-GPS UERE). Needs field
  // verification against your module before relying on it for PosHold gating.
  const float hdop = std::atof(f[8]);
  _model.state.gps.accuracy.horizontal = (uint32_t)(hdop * 5000.0f);
}

void GpsSensor::handleNmeaRmc(char** f, size_t n) const
{
  if (n < 10) return;

  if (f[2][0] == 'A') // 'A' = valid fix, 'V' = warning/invalid
  {
    const float speedMs = std::atof(f[7]) * 0.514444f; // knots -> m/s
    const float courseDeg = std::atof(f[8]);
    const float speedMmS = speedMs * 1000.0f;

    _model.state.gps.velocity.raw.groundSpeed = lrintf(speedMmS);
    _model.state.gps.velocity.raw.speed3d = lrintf(speedMmS);
    _model.state.gps.velocity.raw.heading = lrintf(courseDeg * 1e5f);
    // No native N/E velocity in NMEA; derive from ground speed + course.
    // This is a trig decomposition of a directly-measured speed, not a
    // position-delta estimate, so it stays consistent with PosHold's
    // "velocity is a direct measurement" assumption.
    _model.state.gps.velocity.raw.north = lrintf(speedMmS * cosf(Utils::toRad(courseDeg)));
    _model.state.gps.velocity.raw.east = lrintf(speedMmS * sinf(Utils::toRad(courseDeg)));
  }

  if (std::strlen(f[9]) >= 6)
  {
    _model.state.gps.dateTime.day = (f[9][0] - '0') * 10 + (f[9][1] - '0');
    _model.state.gps.dateTime.month = (f[9][2] - '0') * 10 + (f[9][3] - '0');
    _model.state.gps.dateTime.year = 2000 + (f[9][4] - '0') * 10 + (f[9][5] - '0');
  }
}

void GpsSensor::handleNmeaGsa(char** f, size_t n) const
{
  if (n < 18) return;

  const int fixType = std::atoi(f[2]); // 1=no fix, 2=2D, 3=3D
  if (fixType > 0) _model.state.gps.fixType = (uint8_t)fixType;
  _model.state.gps.accuracy.pDop = (uint32_t)(std::atof(f[15]) * 100.0f);
}

void GpsSensor::handleNmeaGsv(char** f, size_t n) const
{
  if (n < 4) return;

  const uint8_t msgNum = (uint8_t)std::atoi(f[2]);
  _model.state.gps.numCh = std::min<uint8_t>((uint8_t)std::atoi(f[3]), SAT_MAX);

  const size_t base = (msgNum - 1) * 4;
  for (size_t i = 0; i < 4; i++)
  {
    const size_t fi = 4 + i * 4;
    const size_t si = base + i;
    if (fi + 3 >= n || si >= SAT_MAX || si >= _model.state.gps.numCh) break;
    if (!f[fi][0]) continue;

    _model.state.gps.svinfo[si].id = (uint8_t)std::atoi(f[fi]);
    _model.state.gps.svinfo[si].gnssId = 0;
    _model.state.gps.svinfo[si].cno = (uint8_t)std::atoi(f[fi + 3]);
  }
}

void GpsSensor::handleVersion() const
{
  const char* payload = (const char*)_ubxMsg.payload;

  _model.logger.info().log("GPS VER").logln(payload);
  _model.logger.info().log("GPS VER").logln(payload + 30);

  if (std::strcmp(payload + 30, "00080000") == 0)
  {
    _model.state.gps.support.version = GPS_M8;
  }
  else if (std::strcmp(payload + 30, "00090000") == 0)
  {
    _model.state.gps.support.version = GPS_M9;
  }
  else if (std::strcmp(payload + 30, "00190000") == 0)
  {
    _model.state.gps.support.version = GPS_F9;
  }
  else if (std::strcmp(payload + 30, "000A0000") == 0)
  {
    _model.state.gps.support.version = GPS_M10;
  }
  else if (_model.state.gps.support.protVerMajor == 0)
  {
    // u-blox 6 does not expose the protocol version extension used by M8+.
    _model.state.gps.support.version = GPS_M6;
  }

  if (_ubxMsg.length >= 70)
  {
    checkSupport(payload + 40);
    _model.logger.info().log("GPS EXT").logln(payload + 40);
  }
  if (_ubxMsg.length >= 100)
  {
    checkSupport(payload + 70);
    _model.logger.info().log("GPS EXT").logln(payload + 70);
  }
  if (_ubxMsg.length >= 130)
  {
    checkSupport(payload + 100);
    _model.logger.info().log("GPS EXT").logln(payload + 100);
  }
  if (_ubxMsg.length >= 160)
  {
    checkSupport(payload + 130);
    _model.logger.info().log("GPS EXT").logln(payload + 130);
  }
}

void GpsSensor::checkSupport(const char* payload) const
{
  if (std::strstr(payload, "GPS") != nullptr)
  {
    _model.state.gps.support.gps = true;
  }
  if (std::strstr(payload, "SBAS") != nullptr)
  {
    _model.state.gps.support.sbas = true;
  }
  if (std::strstr(payload, "GLO") != nullptr)
  {
    _model.state.gps.support.glonass = true;
  }
  if (std::strstr(payload, "GAL") != nullptr)
  {
    _model.state.gps.support.galileo = true;
  }
  if (std::strstr(payload, "BDS") != nullptr)
  {
    _model.state.gps.support.beidou = true;
  }
  if (std::strstr(payload, "QZSS") != nullptr)
  {
    _model.state.gps.support.qzss = true;
  }
  if (std::strstr(payload, "IMES") != nullptr)
  {
    _model.state.gps.support.imes = true;
  }
  const char* pv = std::strstr(payload, "PROTVER=");
  if (pv != nullptr)
  {
    _model.state.gps.support.protVerMajor = (uint8_t)std::atoi(pv + 8);
  }
}

} // namespace Espfc::Sensor
