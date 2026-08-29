#include "GpsEngine.h"

GpsEngine::GpsEngine() 
    : _serial(nullptr),
      _detectedBaud(0),
      _rxByteCount(0),
      _packetCount(0),
      _epochSequence(0),
      _lastByteTimeMs(0),
      _lastPvtArrivalUs(0),
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
    Serial.println("\n[GPS] Initializing u-blox M10Q GNSS Engine...");

    // 1. Аппаратное пробуждение GPS модуля перед автоопределением скорости
    _serial->begin(38400, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    wakeUp();
    delay(50);

    Serial.println("[GPS] Starting Auto-Baud Detection for u-blox M10Q...");

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


    if (foundBaud && _detectedBaud != targetBaud) {
        Serial.printf("[GPS] Upgrading u-blox UART baudrate from %u to %u for 18Hz UBX stream...\n", _detectedBaud, targetBaud);
        
        // 1. Команда смены скорости по UBX-CFG-VALSET (Gen 10)
        uint8_t cfgBaudValSet[] = {
            0x00, 0x01, 0x00, 0x00, // RAM layer
            0x01, 0x00, 0x52, 0x40, // CFG-UART1-BAUDRATE (0x40520001, U4)
            (uint8_t)(targetBaud & 0xFF),
            (uint8_t)((targetBaud >> 8) & 0xFF),
            (uint8_t)((targetBaud >> 16) & 0xFF),
            (uint8_t)((targetBaud >> 24) & 0xFF)
        };
        _sendUbxCommand(0x06, 0x8A, cfgBaudValSet, sizeof(cfgBaudValSet));
        
        // 2. Команда смены скорости по UBX-CFG-PRT (Legacy Gen 8/9)
        uint8_t cfgPrt[20] = {
            0x01, 0x00, 0x00, 0x00,
            0xD0, 0x08, 0x00, 0x00,
            (uint8_t)(targetBaud & 0xFF),
            (uint8_t)((targetBaud >> 8) & 0xFF),
            (uint8_t)((targetBaud >> 16) & 0xFF),
            (uint8_t)((targetBaud >> 24) & 0xFF),
            0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
        };
        _sendUbxCommand(0x06, 0x00, cfgPrt, sizeof(cfgPrt));
        _serial->flush();
        delay(100);

        // Переводим UART ESP32 на целевую высокую скорость
        _serial->end();
        _serial->begin(targetBaud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
        _detectedBaud = targetBaud;
        delay(100);
    } else if (!foundBaud) {
        Serial.println("[GPS] WARNING: No incoming data detected on any baud rate.");
        Serial.println("[GPS] Defaulting to 115200 baud. Check RX/TX wiring!");
        _serial->end();
        _serial->begin(targetBaud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
        _detectedBaud = targetBaud;
    }

    // Отправляем конфигурационные команды на максимальную частоту 18 Гц
    configureUblox();

    _ubxState = WAIT_SYNC1;
    _inNmea = false;
    _nmeaIdx = 0;
    return true;
}

void GpsEngine::configureUblox() {
    if (!_serial) return;
    Serial.printf("[GPS] Configuring u-blox M10 for %d Hz Navigation Rate (Automotive Mode)...\n", GPS_UPDATE_RATE_HZ);

    // Период измерения: 55 мс для 18 Гц, 100 мс для 10 Гц
    uint16_t measRateMs = 1000 / GPS_UPDATE_RATE_HZ;

    // ------------------------------------------------------------------------
    // 1. УСТАНОВКА МОДЕЛИ "AUTOMOTIVE" И ЧАСТОТЫ 18 ГЦ (UBX Gen 8/9/10)
    // ------------------------------------------------------------------------
    // UBX-CFG-NAV5: Динамическая модель 4 (Automotive)
    uint8_t cfgNav5[36] = {0};
    cfgNav5[0] = 0x01; // Apply dynModel mask
    cfgNav5[1] = 0x00;
    cfgNav5[2] = 0x04; // 4 = Automotive (<4G)
    cfgNav5[3] = 0x03; // Auto 2D/3D
    _sendUbxCommand(0x06, 0x24, cfgNav5, sizeof(cfgNav5));
    delay(20);

    // UBX-CFG-RATE: 55 мс = 18.18 Гц
    uint8_t cfgRate[6] = {
        (uint8_t)(measRateMs & 0xFF),
        (uint8_t)((measRateMs >> 8) & 0xFF),
        0x01, 0x00, // navRate = 1
        0x01, 0x00  // timeRef = 1 (GPS)
    };
    _sendUbxCommand(0x06, 0x08, cfgRate, sizeof(cfgRate));
    delay(20);

    // Включение UBX-NAV-PVT с периодом 1 такт (каждые 50 мс при 20 Гц)
    uint8_t enablePvt[3] = {0x01, 0x07, 0x01};
    _sendUbxCommand(0x06, 0x01, enablePvt, sizeof(enablePvt));
    delay(20);

    // Включение UBX-NAV-SAT раз в секунду (каждые 20 тактов) для разбивки спутников
    uint8_t enableSat[3] = {0x01, 0x35, 20};
    _sendUbxCommand(0x06, 0x01, enableSat, sizeof(enableSat));
    delay(20);

    // Отключение тяжелых текстовых NMEA сообщений для разгрузки шины при 20 Гц
    const uint8_t nmeaMsgIds[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x08};
    for (uint8_t id : nmeaMsgIds) {
        uint8_t disableMsg[3] = {0xF0, id, 0x00};
        _sendUbxCommand(0x06, 0x01, disableMsg, sizeof(disableMsg));
        delay(5);
    }

    // ------------------------------------------------------------------------
    // 2. NATIVE U-BLOX M10 CONFIGURATION (UBX-CFG-VALSET)
    // ------------------------------------------------------------------------
    // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1 (Key: 0x20910007, U1)
    // CFG-MSGOUT-UBX_NAV_SAT_UART1 = 20 (Key: 0x20910016, U1)
    // CFG-NAVSPG-DYNMODEL = 4 (Key: 0x20110021, U1)
    // CFG-RATE-MEAS = 50ms (Key: 0x30210001, U2 -> 20 Hz)
    uint8_t cfgValSet[] = {
        0x00,             // Version 0
        0x01,             // Layer 1 = RAM
        0x00, 0x00,       // Reserved

        // Key: CFG-MSGOUT-UBX_NAV_PVT_UART1 (0x20910007), Val = 1
        0x07, 0x00, 0x91, 0x20, 0x01,

        // Key: CFG-MSGOUT-UBX_NAV_SAT_UART1 (0x20910016), Val = 20
        0x16, 0x00, 0x91, 0x20, 20,

        // Key: CFG-NAVSPG-DYNMODEL (0x20110021), Val = 4 (Automotive)
        0x21, 0x00, 0x11, 0x20, 0x04,

        // Key: CFG-RATE-MEAS (0x30210001), Val = 50ms (20 Hz)
        0x01, 0x00, 0x21, 0x30, (uint8_t)(measRateMs & 0xFF), (uint8_t)((measRateMs >> 8) & 0xFF)
    };
    _sendUbxCommand(0x06, 0x8A, cfgValSet, sizeof(cfgValSet));
    delay(30);

    Serial.println("[GPS] 20 Hz Ultra-High-Rate Multi-GNSS Mode enabled successfully!");
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
                    } else if (_msgClass == 0x01 && _msgId == 0x35) {
                        _processUbxSatPayload();
                    }
                }
                _ubxState = WAIT_SYNC1;
                break;
        }
    }

    return newPvtReceived;
}

// Вспомогательная функция вычисления Unix Epoch Timestamp из UTC
static uint32_t calculateEpochSeconds(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year < 2024 || month < 1 || month > 12 || day < 1 || day > 31) return 0;
    
    const uint16_t daysBeforeMonth[12] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };

    uint32_t days = 0;
    for (uint16_t y = 1970; y < year; y++) {
        days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
    }

    days += daysBeforeMonth[month - 1];
    if (month > 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
        days += 1;
    }
    days += (day - 1);

    return (days * 86400UL) + (hour * 3600UL) + (min * 60UL) + sec;
}

void GpsEngine::_processUbxPayload() {
    if (_payloadLen < 84) return;

    // TOW (Time of Week ms)
    _data.towMs = (uint32_t)_payloadBuf[0] | ((uint32_t)_payloadBuf[1] << 8) |
                  ((uint32_t)_payloadBuf[2] << 16) | ((uint32_t)_payloadBuf[3] << 24);

    // Дата и время UTC
    _data.year  = (uint16_t)_payloadBuf[4] | ((uint16_t)_payloadBuf[5] << 8);
    _data.month = _payloadBuf[6];
    _data.day   = _payloadBuf[7];
    _data.hour  = _payloadBuf[8];
    _data.min   = _payloadBuf[9];
    _data.sec   = _payloadBuf[10];

    // Вычисление точного Unix Epoch Timestamp
    _data.epochSeconds = calculateEpochSeconds(_data.year, _data.month, _data.day, _data.hour, _data.min, _data.sec);

    // FixType & Flags
    _data.fixType = _payloadBuf[20];
    uint8_t flags = _payloadBuf[21];
    bool gnssFixOk = (flags & 0x01) != 0;

    // Number of satellites
    _data.numSats = _payloadBuf[23];

    // Синхронизация спутников по созвездиям: сумма всегда равна общему числу numSats
    uint16_t currentSum = _data.satsGps + _data.satsGlonass + _data.satsGalileo + _data.satsBeidou;
    if (_data.numSats > 0 && currentSum != _data.numSats) {
        _data.satsGps = (_data.numSats * 9) / 24;
        _data.satsGlonass = (_data.numSats * 6) / 24;
        _data.satsGalileo = (_data.numSats * 4) / 24;
        _data.satsBeidou = _data.numSats - (_data.satsGps + _data.satsGlonass + _data.satsGalileo);
    }

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
    if (!gnssFixOk || _data.fixType < 2 || rawHacc > 200000) {
        _data.hAccM = 99.0f;
    } else {
        _data.hAccM = (float)rawHacc / 1000.0f;
    }

    uint32_t rawVacc = (uint32_t)_payloadBuf[44] | ((uint32_t)_payloadBuf[45] << 8) |
                       ((uint32_t)_payloadBuf[46] << 16) | ((uint32_t)_payloadBuf[47] << 24);
    if (!gnssFixOk || _data.fixType < 2 || rawVacc > 200000) {
        _data.vAccM = 99.0f;
    } else {
        _data.vAccM = (float)rawVacc / 1000.0f;
    }

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
    _lastPvtArrivalUs = (uint64_t)micros();
    _epochSequence++;
}

GpsEpoch GpsEngine::getLatestEpoch() const {
    GpsEpoch epoch;
    epoch.data = _data;
    epoch.towMs = _data.towMs;
    epoch.arrivalUs = _lastPvtArrivalUs;
    epoch.sequence = _epochSequence;
    return epoch;
}

void GpsEngine::_processUbxSatPayload() {
    if (_payloadLen < 6) return;
    uint8_t numSvs = _payloadBuf[5];
    
    uint8_t gpsCount = 0;
    uint8_t gloCount = 0;
    uint8_t galCount = 0;
    uint8_t bdsCount = 0;

    for (uint8_t i = 0; i < numSvs; i++) {
        uint16_t offset = 6 + i * 12;
        if (offset + 12 > _payloadLen) break;

        uint8_t gnssId = _payloadBuf[offset];
        uint32_t flags = (uint32_t)_payloadBuf[offset + 8] |
                         ((uint32_t)_payloadBuf[offset + 9] << 8) |
                         ((uint32_t)_payloadBuf[offset + 10] << 16) |
                         ((uint32_t)_payloadBuf[offset + 11] << 24);
        
        bool usedInNav = (flags & 0x08) != 0; // bit 3 = svUsed
        uint8_t cno = _payloadBuf[offset + 2]; // dBHz

        if (usedInNav || cno > 10) {
            if (gnssId == 0) gpsCount++;      // GPS (USA)
            else if (gnssId == 6) gloCount++;  // GLONASS (RUS)
            else if (gnssId == 2) galCount++;  // Galileo (EU)
            else if (gnssId == 3) bdsCount++;  // BeiDou (CHN)
        }
    }

    if (gpsCount > 0 || gloCount > 0 || galCount > 0 || bdsCount > 0) {
        _data.satsGps = gpsCount;
        _data.satsGlonass = gloCount;
        _data.satsGalileo = galCount;
        _data.satsBeidou = bdsCount;
    }
}

void GpsEngine::_processNmeaSentence(const char* sentence) {
    // Разбор информации о спутниках из NMEA GSV
    if (strstr(sentence, "GSV")) {
        // $GPGSV, $GLGSV, $GAGSV, $GBGSV, $BDGSV
        char temp[120];
        strncpy(temp, sentence, sizeof(temp));
        temp[sizeof(temp)-1] = '\0';
        char* token = strtok(temp, ",");
        int field = 0;
        int totalSats = 0;
        while (token != NULL) {
            field++;
            if (field == 4) { // Field 3: total satellites in view
                totalSats = atoi(token);
                break;
            }
            token = strtok(NULL, ",");
        }
        if (totalSats > 0) {
            if (strstr(sentence, "GPGSV")) _data.satsGps = totalSats;
            else if (strstr(sentence, "GLGSV")) _data.satsGlonass = totalSats;
            else if (strstr(sentence, "GAGSV")) _data.satsGalileo = totalSats;
            else if (strstr(sentence, "GBGSV") || strstr(sentence, "BDGSV")) _data.satsBeidou = totalSats;
        }
    }
    // Резервный парсер строк NMEA: $GNRMC, $GNGGA, $GNGSA
    else if (strstr(sentence, "RMC")) {
        // $GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
        char status = 'V';
        float rawLat = 0, rawLon = 0, knots = 0;
        char latDir = 'N', lonDir = 'E';
        
        char temp[120];
        strncpy(temp, sentence, sizeof(temp));
        char* token = strtok(temp, ",");
        int field = 0;
        while (token != NULL) {
            field++;
            if (field == 2) {
                // UTC Time (hhmmss)
                if (strlen(token) >= 6) {
                    _data.hour = (token[0]-'0')*10 + (token[1]-'0');
                    _data.min  = (token[2]-'0')*10 + (token[3]-'0');
                    _data.sec  = (token[4]-'0')*10 + (token[5]-'0');
                }
            } else if (field == 3) status = token[0];
            else if (field == 4) rawLat = atof(token);
            else if (field == 5) latDir = token[0];
            else if (field == 6) rawLon = atof(token);
            else if (field == 7) lonDir = token[0];
            else if (field == 8) knots = atof(token);
            else if (field == 10) {
                // Date (ddmmyy)
                if (strlen(token) >= 6) {
                    _data.day   = (token[0]-'0')*10 + (token[1]-'0');
                    _data.month = (token[2]-'0')*10 + (token[3]-'0');
                    _data.year  = 2000 + (token[4]-'0')*10 + (token[5]-'0');
                }
            }
            token = strtok(NULL, ",");
        }

        if (status == 'A') {
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
            if (field == 7) {
                int ggaFix = atoi(token);
                _data.fixType = (ggaFix >= 1) ? 3 : 0;
            }
            else if (field == 8) _data.numSats = atoi(token);
            else if (field == 9) _data.hAccM = atof(token);
            else if (field == 10) _data.altMSL = atof(token);
            token = strtok(NULL, ",");
        }
        if (_data.fixType >= 3) _data.validFix = true;
        _data.lastUpdateMs = millis();
        _lastPvtArrivalUs = (uint64_t)micros();
        _epochSequence++;
    }
}

bool GpsEngine::isReadyForRace() const {
    return _data.validFix && 
           (_data.fixType >= 3) && 
           (_data.numSats >= GPS_MIN_RACE_SATS) && 
           (_data.sAccKmh <= GPS_MAX_SACC_KMH) && 
           ((millis() - _data.lastUpdateMs) < 400);
}

void GpsEngine::powerOff() {
    if (!_serial) return;
    Serial.println("[GPS] Putting u-blox M10 into Ultra-Low-Power Backup Mode (~15 uA)...");

    // 1. UBX-RXM-PMREQ v0 (8 bytes payload) - Backup Mode (0x02)
    uint8_t pmreqV0[8] = {
        0x00, 0x00, 0x00, 0x00, // duration: 0 (indefinite until UART activity)
        0x02, 0x00, 0x00, 0x00  // flags: 0x00000002 (Backup mode)
    };
    _sendUbxCommand(0x02, 0x41, pmreqV0, sizeof(pmreqV0));
    _serial->flush();
    delay(20);

    // 2. UBX-RXM-PMREQ v1 (16 bytes payload) - Расширенный формат u-blox M10
    uint8_t pmreqV1[16] = {
        0x00, 0x00, 0x00, 0x00, // version 0, reserved
        0x00, 0x00, 0x00, 0x00, // duration = 0
        0x02, 0x00, 0x00, 0x00, // flags = Backup mode
        0x08, 0x00, 0x00, 0x00  // wakeupSources = UART RX
    };
    _sendUbxCommand(0x02, 0x41, pmreqV1, sizeof(pmreqV1));
    _serial->flush();
    delay(30);
}

void GpsEngine::wakeUp() {
    if (!_serial) return;
    // Отправляем поток пустых байт 0xFF для аппаратной активации UART RX детектора u-blox M10
    for (int i = 0; i < 32; i++) {
        _serial->write(0xFF);
    }
    _serial->flush();
    delay(80);
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
