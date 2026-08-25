#include "GpsEngine.h"

GpsEngine::GpsEngine() 
    : _serial(nullptr),
      _detectedBaud(0),
      _rxByteCount(0),
      _packetCount(0),
      _lastByteTimeMs(0),
      _ubxState(WAIT_SYNC1),
      _msgClass(0),
      _msgId(0),
      _payloadLen(0),
      _payloadCounter(0),
      _ckA(0), _ckB(0),
      _calcCkA(0), _calcCkB(0),
      _nmeaIdx(0),
      _inNmea(false)
{
    memset(&_data, 0, sizeof(GpsData));
    memset(_nmeaBuf, 0, sizeof(_nmeaBuf));
}

bool GpsEngine::begin(HardwareSerial& serialPort, uint32_t targetBaud) {
    _serial = &serialPort;
    Serial.println("\n[GPS] Starting Auto-Baud Detection for u-blox M10Q...");

    // Список распространенных заводских скоростей u-blox модулей
    // (u-blox M10 по умолчанию обычно 38400 или 115200)
    const uint32_t testBauds[] = { 38400, 115200, 9600, 460800, 57600 };
    bool foundBaud = false;

    for (uint32_t baud : testBauds) {
        Serial.printf("[GPS] Probing UART at %u baud (RX: %d, TX: %d)...\n", baud, PIN_GPS_RX, PIN_GPS_TX);
        _serial->end();
        _serial->begin(baud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
        delay(150);

        // Проверяем, идут ли данные
        uint32_t startCheck = millis();
        int bytesCount = 0;
        while (millis() - startCheck < 300) {
            if (_serial->available()) {
                bytesCount += _serial->available();
                while (_serial->available()) _serial->read();
            }
            delay(10);
        }

        if (bytesCount > 5) {
            Serial.printf("[GPS] SUCCESS: Detected active GPS data stream at %u baud! (Received %d bytes)\n", baud, bytesCount);
            _detectedBaud = baud;
            foundBaud = true;
            break;
        }
    }

    if (!foundBaud) {
        Serial.println("[GPS] WARNING: No incoming data detected on any baud rate.");
        Serial.println("[GPS] Defaulting to 115200 baud. Check RX/TX wiring!");
        _serial->end();
        _serial->begin(targetBaud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
        _detectedBaud = targetBaud;
    }

    // Отправляем конфигурационные команды на текущей скорости
    configureUblox();

    _ubxState = WAIT_SYNC1;
    _inNmea = false;
    _nmeaIdx = 0;
    return true;
}

void GpsEngine::configureUblox() {
    if (!_serial) return;
    Serial.println("[GPS] Sending u-blox M10 configuration (UBX-CFG-VALSET & UBX-CFG)...");

    // ------------------------------------------------------------------------
    // 1. УСТАНОВКА МОДЕЛИ "AUTOMOTIVE" И ЧАСТОТЫ 10 ГЦ (UBX Gen 8/9/10)
    // ------------------------------------------------------------------------
    // UBX-CFG-NAV5: Динамическая модель 4 (Automotive)
    uint8_t cfgNav5[36] = {0};
    cfgNav5[0] = 0x01; // Apply dynModel mask
    cfgNav5[1] = 0x00;
    cfgNav5[2] = 0x04; // 4 = Automotive (<4G)
    cfgNav5[3] = 0x03; // Auto 2D/3D
    _sendUbxCommand(0x06, 0x24, cfgNav5, sizeof(cfgNav5));
    delay(20);

    // UBX-CFG-RATE: 100 мс = 10 Гц
    uint16_t measRateMs = 1000 / GPS_UPDATE_RATE_HZ;
    uint8_t cfgRate[6] = {
        (uint8_t)(measRateMs & 0xFF),
        (uint8_t)((measRateMs >> 8) & 0xFF),
        0x01, 0x00, // navRate = 1
        0x01, 0x00  // timeRef = 1 (GPS)
    };
    _sendUbxCommand(0x06, 0x08, cfgRate, sizeof(cfgRate));
    delay(20);

    // Включение UBX-NAV-PVT с периодом 1 такт
    uint8_t enablePvt[3] = {0x01, 0x07, 0x01};
    _sendUbxCommand(0x06, 0x01, enablePvt, sizeof(enablePvt));
    delay(20);

    // ------------------------------------------------------------------------
    // 2. NATIVE U-BLOX M10 CONFIGURATION (UBX-CFG-VALSET)
    // ------------------------------------------------------------------------
    // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1 (Key: 0x20910007, U1)
    // CFG-NAVSPG-DYNMODEL = 4 (Key: 0x20110021, U1)
    // CFG-RATE-MEAS = 100ms (Key: 0x30210001, U2)
    uint8_t cfgValSet[] = {
        0x00,             // Version 0
        0x01,             // Layer 1 = RAM
        0x00, 0x00,       // Reserved

        // Key: CFG-MSGOUT-UBX_NAV_PVT_UART1 (0x20910007), Val = 1
        0x07, 0x00, 0x91, 0x20, 0x01,

        // Key: CFG-NAVSPG-DYNMODEL (0x20110021), Val = 4 (Automotive)
        0x21, 0x00, 0x11, 0x20, 0x04,

        // Key: CFG-RATE-MEAS (0x30210001), Val = 100ms (10Hz)
        0x01, 0x00, 0x21, 0x30, (uint8_t)(measRateMs & 0xFF), (uint8_t)((measRateMs >> 8) & 0xFF)
    };
    _sendUbxCommand(0x06, 0x8A, cfgValSet, sizeof(cfgValSet));
    delay(30);

    Serial.println("[GPS] Configuration sent successfully.");
}

bool GpsEngine::update() {
    if (!_serial) return false;
    
    bool newPvtReceived = false;

    while (_serial->available() > 0) {
        uint8_t b = _serial->read();
        _rxByteCount++;
        _lastByteTimeMs = millis();

        // --------------------------------------------------------------------
        // 1. ПАРСЕР NMEA (СТРОКИ С НАЧАЛОМ '$')
        // --------------------------------------------------------------------
        if (b == '$') {
            _inNmea = true;
            _nmeaIdx = 0;
            _nmeaBuf[_nmeaIdx++] = '$';
        } else if (_inNmea) {
            if (b == '\n' || b == '\r') {
                if (_nmeaIdx > 6) {
                    _nmeaBuf[_nmeaIdx] = '\0';
                    _processNmeaSentence(_nmeaBuf);
                    newPvtReceived = true;
                }
                _inNmea = false;
                _nmeaIdx = 0;
            } else if (_nmeaIdx < sizeof(_nmeaBuf) - 1) {
                _nmeaBuf[_nmeaIdx++] = (char)b;
            } else {
                _inNmea = false;
            }
        }

        // --------------------------------------------------------------------
        // 2. БИНАРНЫЙ ПАРСЕР UBX-NAV-PVT
        // --------------------------------------------------------------------
        switch (_ubxState) {
            case WAIT_SYNC1:
                if (b == 0xB5) {
                    _ubxState = WAIT_SYNC2;
                }
                break;

            case WAIT_SYNC2:
                if (b == 0x62) {
                    _ubxState = WAIT_CLASS;
                    _calcCkA = 0;
                    _calcCkB = 0;
                } else {
                    _ubxState = WAIT_SYNC1;
                }
                break;

            case WAIT_CLASS:
                _msgClass = b;
                _calculateChecksum(b, _calcCkA, _calcCkB);
                _ubxState = WAIT_ID;
                break;

            case WAIT_ID:
                _msgId = b;
                _calculateChecksum(b, _calcCkA, _calcCkB);
                _ubxState = WAIT_LEN_L;
                break;

            case WAIT_LEN_L:
                _payloadLen = b;
                _calculateChecksum(b, _calcCkA, _calcCkB);
                _ubxState = WAIT_LEN_H;
                break;

            case WAIT_LEN_H:
                _payloadLen |= ((uint16_t)b << 8);
                _calculateChecksum(b, _calcCkA, _calcCkB);
                _payloadCounter = 0;
                
                if (_payloadLen > sizeof(_payloadBuf)) {
                    _ubxState = WAIT_SYNC1;
                } else if (_payloadLen == 0) {
                    _ubxState = WAIT_CKA;
                } else {
                    _ubxState = PAYLOAD;
                }
                break;

            case PAYLOAD:
                _payloadBuf[_payloadCounter++] = b;
                _calculateChecksum(b, _calcCkA, _calcCkB);
                if (_payloadCounter >= _payloadLen) {
                    _ubxState = WAIT_CKA;
                }
                break;

            case WAIT_CKA:
                _ckA = b;
                _ubxState = WAIT_CKB;
                break;

            case WAIT_CKB:
                _ckB = b;
                if (_ckA == _calcCkA && _ckB == _calcCkB) {
                    if (_msgClass == 0x01 && _msgId == 0x07) {
                        _processUbxPayload();
                        _packetCount++;
                        newPvtReceived = true;
                    }
                }
                _ubxState = WAIT_SYNC1;
                break;
        }
    }

    return newPvtReceived;
}

void GpsEngine::_processUbxPayload() {
    if (_payloadLen < 84) return;

    // TOW (Time of Week ms)
    _data.towMs = (uint32_t)_payloadBuf[0] | ((uint32_t)_payloadBuf[1] << 8) |
                  ((uint32_t)_payloadBuf[2] << 16) | ((uint32_t)_payloadBuf[3] << 24);

    // FixType & Flags
    _data.fixType = _payloadBuf[20];
    uint8_t flags = _payloadBuf[21];
    bool gnssFixOk = (flags & 0x01) != 0;

    // Number of satellites
    _data.numSats = _payloadBuf[23];

    // Longitude & Latitude (1e-7 deg)
    int32_t rawLon = (int32_t)((uint32_t)_payloadBuf[24] | ((uint32_t)_payloadBuf[25] << 8) |
                               ((uint32_t)_payloadBuf[26] << 16) | ((uint32_t)_payloadBuf[27] << 24));
    _data.lon = (double)rawLon / 10000000.0;

    int32_t rawLat = (int32_t)((uint32_t)_payloadBuf[28] | ((uint32_t)_payloadBuf[29] << 8) |
                               ((uint32_t)_payloadBuf[30] << 16) | ((uint32_t)_payloadBuf[31] << 24));
    _data.lat = (double)rawLat / 10000000.0;

    // Height MSL (mm)
    int32_t rawHmsl = (int32_t)((uint32_t)_payloadBuf[36] | ((uint32_t)_payloadBuf[37] << 8) |
                                ((uint32_t)_payloadBuf[38] << 16) | ((uint32_t)_payloadBuf[39] << 24));
    _data.altMSL = (float)rawHmsl / 1000.0f;

    // Horizontal & Vertical Accuracy (mm)
    uint32_t rawHacc = (uint32_t)_payloadBuf[40] | ((uint32_t)_payloadBuf[41] << 8) |
                       ((uint32_t)_payloadBuf[42] << 16) | ((uint32_t)_payloadBuf[43] << 24);
    _data.hAccM = (float)rawHacc / 1000.0f;

    uint32_t rawVacc = (uint32_t)_payloadBuf[44] | ((uint32_t)_payloadBuf[45] << 8) |
                       ((uint32_t)_payloadBuf[46] << 16) | ((uint32_t)_payloadBuf[47] << 24);
    _data.vAccM = (float)rawVacc / 1000.0f;

    // Ground Speed (Doppler 3D velocity mm/s)
    int32_t rawGspeed = (int32_t)((uint32_t)_payloadBuf[60] | ((uint32_t)_payloadBuf[61] << 8) |
                                  ((uint32_t)_payloadBuf[62] << 16) | ((uint32_t)_payloadBuf[63] << 24));
    _data.speedMs = (float)rawGspeed / 1000.0f;
    _data.speedKmh = _data.speedMs * 3.6f;

    // Heading of motion (1e-5 deg)
    int32_t rawHead = (int32_t)((uint32_t)_payloadBuf[64] | ((uint32_t)_payloadBuf[65] << 8) |
                                ((uint32_t)_payloadBuf[66] << 16) | ((uint32_t)_payloadBuf[67] << 24));
    _data.headingDeg = (float)rawHead / 100000.0f;

    // Speed Accuracy (mm/s)
    uint32_t rawSacc = (uint32_t)_payloadBuf[68] | ((uint32_t)_payloadBuf[69] << 8) |
                       ((uint32_t)_payloadBuf[70] << 16) | ((uint32_t)_payloadBuf[71] << 24);
    _data.sAccKmh = ((float)rawSacc / 1000.0f) * 3.6f;

    // pDOP
    if (_payloadLen >= 78) {
        uint16_t rawPdop = (uint16_t)_payloadBuf[76] | ((uint16_t)_payloadBuf[77] << 8);
        _data.pDOP = (float)rawPdop * 0.01f;
    }

    _data.validFix = gnssFixOk && (_data.fixType == 3 || _data.fixType == 4);
    _data.lastUpdateMs = millis();
}

void GpsEngine::_processNmeaSentence(const char* sentence) {
    // Резервный парсер строк NMEA: $GNRMC, $GNGGA, $GNGSA, $GPGSV
    if (strstr(sentence, "RMC")) {
        // $GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
        char status = 'V';
        float rawLat = 0, rawLon = 0, knots = 0;
        char latDir = 'N', lonDir = 'E';
        
        // Быстрый разбор RMC
        char temp[120];
        strncpy(temp, sentence, sizeof(temp));
        char* token = strtok(temp, ",");
        int field = 0;
        while (token != NULL) {
            field++;
            if (field == 3) status = token[0];
            else if (field == 4) rawLat = atof(token);
            else if (field == 5) latDir = token[0];
            else if (field == 6) rawLon = atof(token);
            else if (field == 7) lonDir = token[0];
            else if (field == 8) knots = atof(token);
            token = strtok(NULL, ",");
        }

        if (status == 'A') {
            // Перевод DDMM.MMMM в градусы
            int latDeg = (int)(rawLat / 100);
            float latMin = rawLat - (latDeg * 100);
            _data.lat = latDeg + (latMin / 60.0f);
            if (latDir == 'S') _data.lat = -_data.lat;

            int lonDeg = (int)(rawLon / 100);
            float lonMin = rawLon - (lonDeg * 100);
            _data.lon = lonDeg + (lonMin / 60.0f);
            if (lonDir == 'W') _data.lon = -_data.lon;

            _data.speedKmh = knots * 1.852f;
            _data.speedMs = _data.speedKmh / 3.6f;
            _data.validFix = true;
            _data.lastUpdateMs = millis();
        }
    } else if (strstr(sentence, "GGA")) {
        // $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        char temp[120];
        strncpy(temp, sentence, sizeof(temp));
        char* token = strtok(temp, ",");
        int field = 0;
        while (token != NULL) {
            field++;
            if (field == 7) _data.fixType = atoi(token);
            else if (field == 8) _data.numSats = atoi(token);
            else if (field == 9) _data.hAccM = atof(token);
            else if (field == 10) _data.altMSL = atof(token);
            token = strtok(NULL, ",");
        }
        if (_data.fixType >= 1) _data.validFix = true;
        _data.lastUpdateMs = millis();
    }
}

bool GpsEngine::isReadyForRace() const {
    return _data.validFix && (_data.numSats >= 6) && ((millis() - _data.lastUpdateMs) < 500);
}

void GpsEngine::_sendUbxCommand(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    if (!_serial) return;

    uint8_t cka = 0, ckb = 0;
    
    _serial->write(0xB5);
    _serial->write(0x62);

    _serial->write(cls);
    _calculateChecksum(cls, cka, ckb);

    _serial->write(id);
    _calculateChecksum(id, cka, ckb);

    _serial->write((uint8_t)(len & 0xFF));
    _calculateChecksum((uint8_t)(len & 0xFF), cka, ckb);

    _serial->write((uint8_t)((len >> 8) & 0xFF));
    _calculateChecksum((uint8_t)((len >> 8) & 0xFF), cka, ckb);

    for (uint16_t i = 0; i < len; i++) {
        _serial->write(payload[i]);
        _calculateChecksum(payload[i], cka, ckb);
    }

    _serial->write(cka);
    _serial->write(ckb);
}

void GpsEngine::_calculateChecksum(uint8_t byte, uint8_t& cka, uint8_t& ckb) {
    cka += byte;
    ckb += cka;
}
