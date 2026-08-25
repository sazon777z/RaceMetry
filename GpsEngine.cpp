#include "GpsEngine.h"

GpsEngine::GpsEngine() 
    : _serial(nullptr),
      _state(WAIT_SYNC1),
      _msgClass(0),
      _msgId(0),
      _payloadLen(0),
      _payloadCounter(0),
      _ckA(0), _ckB(0),
      _calcCkA(0), _calcCkB(0)
{
    memset(&_data, 0, sizeof(GpsData));
}

bool GpsEngine::begin(HardwareSerial& serialPort, uint32_t targetBaud) {
    _serial = &serialPort;
    
    // Инициализируем UART на стандартной скорости GPS (9600) для отправки переключения
    _serial->begin(GPS_BAUDRATE_INITIAL, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    delay(100);

    // Посылаем команду смены битрейта на скоростной (targetBaud)
    // UBX-CFG-PRT для UART1: portID=1, mode=0x08D0 (8N1), baudRate=targetBaud, inProto=0x0007, outProto=0x0001
    uint8_t cfgPrtPayload[20] = {
        0x01, 0x00, 0x00, 0x00,             // Port 1 (UART1), reserved
        0xD0, 0x08, 0x00, 0x00,             // 8N1 (0x08D0)
        (uint8_t)(targetBaud & 0xFF),        // Baud rate byte 0
        (uint8_t)((targetBaud >> 8) & 0xFF), // Baud rate byte 1
        (uint8_t)((targetBaud >> 16) & 0xFF),// Baud rate byte 2
        (uint8_t)((targetBaud >> 24) & 0xFF),// Baud rate byte 3
        0x07, 0x00,                         // InProtoMask (UBX + NMEA + RTCM)
        0x01, 0x00,                         // OutProtoMask (UBX only)
        0x00, 0x00,                         // Flags
        0x00, 0x00                          // Reserved
    };
    _sendUbxCommand(0x06, 0x00, cfgPrtPayload, sizeof(cfgPrtPayload));
    _serial->flush();
    delay(100);

    // Переинициализируем UART ESP32 на целевой скоростной битрейт
    _serial->end();
    _serial->begin(targetBaud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    delay(100);

    // Отправляем остальную конфигурацию (частота, динамическая модель, фильтры)
    configureUblox();

    _state = WAIT_SYNC1;
    return true;
}

void GpsEngine::configureUblox() {
    if (!_serial) return;

    // 1. Установка модели динамики "Automotive" (UBX-CFG-NAV5)
    // Модель 4 оптимизирует Калман-фильтр GPS под перегрузки и ускорения автомобиля
    uint8_t cfgNav5[36] = {0};
    cfgNav5[0] = 0x01; // Apply dynamic model mask
    cfgNav5[1] = 0x00;
    cfgNav5[2] = 0x04; // 4 = Automotive model (< 4G acceleration)
    cfgNav5[3] = 0x03; // Fix mode 3 = Auto 2D/3D
    _sendUbxCommand(0x06, 0x024, cfgNav5, sizeof(cfgNav5));
    delay(30);

    // 2. Установка частоты обновления (UBX-CFG-RATE)
    // 100 мс = 10 Гц, 55 мс = 18 Гц
    uint16_t measRateMs = 1000 / GPS_UPDATE_RATE_HZ;
    uint8_t cfgRate[6] = {
        (uint8_t)(measRateMs & 0xFF),
        (uint8_t)((measRateMs >> 8) & 0xFF),
        0x01, 0x00, // navRate = 1 cycle
        0x01, 0x00  // timeRef = 1 (GPS time)
    };
    _sendUbxCommand(0x06, 0x08, cfgRate, sizeof(cfgRate));
    delay(30);

    // 3. Отключение тяжелых текстовых NMEA сообщений (UBX-CFG-MSG)
    const uint8_t nmeaMsgIds[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x08};
    for (uint8_t id : nmeaMsgIds) {
        uint8_t disableMsg[3] = {0xF0, id, 0x00}; // Class 0xF0 (NMEA), ID, rate=0
        _sendUbxCommand(0x06, 0x01, disableMsg, sizeof(disableMsg));
        delay(10);
    }

    // 4. Включение бинарного навигационного пакета UBX-NAV-PVT с периодом 1 такт
    uint8_t enablePvt[3] = {0x01, 0x07, 0x01}; // Class 0x01 (NAV), ID 0x07 (PVT), rate=1
    _sendUbxCommand(0x06, 0x01, enablePvt, sizeof(enablePvt));
    delay(30);
}

bool GpsEngine::update() {
    if (!_serial) return false;
    
    bool newPvtReceived = false;

    while (_serial->available() > 0) {
        uint8_t b = _serial->read();

        switch (_state) {
            case WAIT_SYNC1:
                if (b == 0xB5) {
                    _state = WAIT_SYNC2;
                }
                break;

            case WAIT_SYNC2:
                if (b == 0x62) {
                    _state = WAIT_CLASS;
                    _calcCkA = 0;
                    _calcCkB = 0;
                } else {
                    _state = WAIT_SYNC1;
                }
                break;

            case WAIT_CLASS:
                _msgClass = b;
                _calculateChecksum(b, _calcCkA, _calcCkB);
                _state = WAIT_ID;
                break;

            case WAIT_ID:
                _msgId = b;
                _calculateChecksum(b, _calcCkA, _calcCkB);
                _state = WAIT_LEN_L;
                break;

            case WAIT_LEN_L:
                _payloadLen = b;
                _calculateChecksum(b, _calcCkA, _calcCkB);
                _state = WAIT_LEN_H;
                break;

            case WAIT_LEN_H:
                _payloadLen |= ((uint16_t)b << 8);
                _calculateChecksum(b, _calcCkA, _calcCkB);
                _payloadCounter = 0;
                
                if (_payloadLen > sizeof(_payloadBuf)) {
                    // Превышение размера нашего буфера — сброс
                    _state = WAIT_SYNC1;
                } else if (_payloadLen == 0) {
                    _state = WAIT_CKA;
                } else {
                    _state = PAYLOAD;
                }
                break;

            case PAYLOAD:
                _payloadBuf[_payloadCounter++] = b;
                _calculateChecksum(b, _calcCkA, _calcCkB);
                if (_payloadCounter >= _payloadLen) {
                    _state = WAIT_CKA;
                }
                break;

            case WAIT_CKA:
                _ckA = b;
                _state = WAIT_CKB;
                break;

            case WAIT_CKB:
                _ckB = b;
                if (_ckA == _calcCkA && _ckB == _calcCkB) {
                    // Контрольная сумма совпала! Обрабатываем полезную нагрузку
                    if (_msgClass == 0x01 && _msgId == 0x07) {
                        _processPayload();
                        newPvtReceived = true;
                    }
                }
                _state = WAIT_SYNC1;
                break;
        }
    }

    return newPvtReceived;
}

void GpsEngine::_processPayload() {
    if (_payloadLen < 84) return;

    // Извлечение полей из структуры UBX-NAV-PVT (92 байта)
    // TOW (Time of Week ms) [0..3]
    _data.towMs = (uint32_t)_payloadBuf[0] | ((uint32_t)_payloadBuf[1] << 8) |
                  ((uint32_t)_payloadBuf[2] << 16) | ((uint32_t)_payloadBuf[3] << 24);

    // FixType [20]
    _data.fixType = _payloadBuf[20];
    
    // Flags [21]
    uint8_t flags = _payloadBuf[21];
    bool gnssFixOk = (flags & 0x01) != 0;

    // Number of satellites [23]
    _data.numSats = _payloadBuf[23];

    // Longitude [24..27] (1e-7 deg)
    int32_t rawLon = (int32_t)((uint32_t)_payloadBuf[24] | ((uint32_t)_payloadBuf[25] << 8) |
                               ((uint32_t)_payloadBuf[26] << 16) | ((uint32_t)_payloadBuf[27] << 24));
    _data.lon = (double)rawLon / 10000000.0;

    // Latitude [28..31] (1e-7 deg)
    int32_t rawLat = (int32_t)((uint32_t)_payloadBuf[28] | ((uint32_t)_payloadBuf[29] << 8) |
                               ((uint32_t)_payloadBuf[30] << 16) | ((uint32_t)_payloadBuf[31] << 24));
    _data.lat = (double)rawLat / 10000000.0;

    // Height MSL [36..39] (mm)
    int32_t rawHmsl = (int32_t)((uint32_t)_payloadBuf[36] | ((uint32_t)_payloadBuf[37] << 8) |
                                ((uint32_t)_payloadBuf[38] << 16) | ((uint32_t)_payloadBuf[39] << 24));
    _data.altMSL = (float)rawHmsl / 1000.0f;

    // Horizontal Accuracy [40..43] (mm)
    uint32_t rawHacc = (uint32_t)_payloadBuf[40] | ((uint32_t)_payloadBuf[41] << 8) |
                       ((uint32_t)_payloadBuf[42] << 16) | ((uint32_t)_payloadBuf[43] << 24);
    _data.hAccM = (float)rawHacc / 1000.0f;

    // Vertical Accuracy [44..47] (mm)
    uint32_t rawVacc = (uint32_t)_payloadBuf[44] | ((uint32_t)_payloadBuf[45] << 8) |
                       ((uint32_t)_payloadBuf[46] << 16) | ((uint32_t)_payloadBuf[47] << 24);
    _data.vAccM = (float)rawVacc / 1000.0f;

    // Ground Speed from Doppler [60..63] (mm/s)
    int32_t rawGspeed = (int32_t)((uint32_t)_payloadBuf[60] | ((uint32_t)_payloadBuf[61] << 8) |
                                  ((uint32_t)_payloadBuf[62] << 16) | ((uint32_t)_payloadBuf[63] << 24));
    _data.speedMs = (float)rawGspeed / 1000.0f;
    _data.speedKmh = _data.speedMs * 3.6f;

    // Heading of motion [64..67] (1e-5 deg)
    int32_t rawHead = (int32_t)((uint32_t)_payloadBuf[64] | ((uint32_t)_payloadBuf[65] << 8) |
                                ((uint32_t)_payloadBuf[66] << 16) | ((uint32_t)_payloadBuf[67] << 24));
    _data.headingDeg = (float)rawHead / 100000.0f;

    // Speed Accuracy [68..71] (mm/s)
    uint32_t rawSacc = (uint32_t)_payloadBuf[68] | ((uint32_t)_payloadBuf[69] << 8) |
                       ((uint32_t)_payloadBuf[70] << 16) | ((uint32_t)_payloadBuf[71] << 24);
    _data.sAccKmh = ((float)rawSacc / 1000.0f) * 3.6f;

    // pDOP [76..77] (0.01)
    if (_payloadLen >= 78) {
        uint16_t rawPdop = (uint16_t)_payloadBuf[76] | ((uint16_t)_payloadBuf[77] << 8);
        _data.pDOP = (float)rawPdop * 0.01f;
    }

    _data.validFix = gnssFixOk && (_data.fixType == 3 || _data.fixType == 4);
    _data.lastUpdateMs = millis();
}

bool GpsEngine::isReadyForRace() const {
    // Для профессионального замера необходимо минимум 7 спутников, 3D Fix и точность hAcc < 2.0м
    return _data.validFix && (_data.numSats >= 7) && (_data.hAccM < 2.5f) && ((millis() - _data.lastUpdateMs) < 300);
}

void GpsEngine::_sendUbxCommand(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    if (!_serial) return;

    uint8_t cka = 0, ckb = 0;
    
    // Преамбула UBX
    _serial->write(0xB5);
    _serial->write(0x62);

    // Заголовок (Class, ID, Len)
    _serial->write(cls);
    _calculateChecksum(cls, cka, ckb);

    _serial->write(id);
    _calculateChecksum(id, cka, ckb);

    _serial->write((uint8_t)(len & 0xFF));
    _calculateChecksum((uint8_t)(len & 0xFF), cka, ckb);

    _serial->write((uint8_t)((len >> 8) & 0xFF));
    _calculateChecksum((uint8_t)((len >> 8) & 0xFF), cka, ckb);

    // Полезная нагрузка
    for (uint16_t i = 0; i < len; i++) {
        _serial->write(payload[i]);
        _calculateChecksum(payload[i], cka, ckb);
    }

    // Контрольные суммы
    _serial->write(cka);
    _serial->write(ckb);
}

void GpsEngine::_calculateChecksum(uint8_t byte, uint8_t& cka, uint8_t& ckb) {
    cka += byte;
    ckb += cka;
}
