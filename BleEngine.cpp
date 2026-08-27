#include "BleEngine.h"

static int8_t s_latestRssi = -55;

BleEngine::BleEngine()
    : _pServer(nullptr),
      _pService(nullptr),
      _pTxCharacteristic(nullptr),
      _pRxCharacteristic(nullptr),
      _deviceConnected(false),
      _oldDeviceConnected(false),
      _lastTxTimeMs(0),
      _cmdHandler(nullptr),
      _rxAccumulatorLen(0)
{
    memset(_txBuffer, 0, sizeof(_txBuffer));
    memset(_rxAccumulator, 0, sizeof(_rxAccumulator));
}

bool BleEngine::begin(const char* deviceName) {
    Serial.printf("[BLE] Initializing BLE Server: %s\n", deviceName);

    // 1. Инициализация стека BLE Device на максимальной мощности передатчика (+9dBm)
    BLEDevice::init(deviceName);
    BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);
    BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);
    BLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_CONN_HDL0);
    BLEDevice::setMTU(517);

    // 2. Создание BLE Сервера
    _pServer = BLEDevice::createServer();
    if (!_pServer) {
        Serial.println("[BLE] ERROR: Failed to create BLE Server!");
        return false;
    }
    _pServer->setCallbacks(this);

    // 3. Создание Сервиса Nordic UART (NUS)
    _pService = _pServer->createService(BLE_NUS_SERVICE_UUID);
    if (!_pService) {
        Serial.println("[BLE] ERROR: Failed to create NUS Service!");
        return false;
    }

    // 4. Создание TX Характеристики (ESP32 -> Phone: Notify + Read)
    _pTxCharacteristic = _pService->createCharacteristic(
        BLE_NUS_TX_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );
    _pTxCharacteristic->addDescriptor(new BLE2902());

    // 5. Создание RX Характеристики (Phone -> ESP32: Write / Write Without Response)
    _pRxCharacteristic = _pService->createCharacteristic(
        BLE_NUS_RX_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    _pRxCharacteristic->setCallbacks(this);

    // 6. Запуск сервиса
    _pService->start();

    // 7. Четкое разделение пакетов рекламы для исключения переполнения 31 байта:
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    
    // Основной пакет рекламы: флаги + полное имя устройства (гарантированно входит в 31 байт)
    BLEAdvertisementData advData;
    advData.setFlags(0x06); // BR_EDR_NOT_SUPP | GENERAL_DISC_MODE
    advData.setName(deviceName);
    pAdvertising->setAdvertisementData(advData);

    // Пакет ответа на сканирование (Scan Response): 128-битный UUID сервиса NUS
    BLEAdvertisementData scanData;
    scanData.setCompleteServices(BLEUUID(BLE_NUS_SERVICE_UUID));
    pAdvertising->setScanResponseData(scanData);

    pAdvertising->setMinPreferred(0x06); // 7.5ms
    pAdvertising->setMaxPreferred(0x12); // 22.5ms
    pAdvertising->setMinInterval(32);    // 20ms
    pAdvertising->setMaxInterval(64);    // 40ms
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising started with max TX power. Waiting for smartphone connection...");
    return true;
}

void BleEngine::update() {
    // Внутренний контроль статуса и сторож активности рекламы
    if (!_deviceConnected) {
        if (_oldDeviceConnected) {
            _oldDeviceConnected = false;
            if (_pServer) {
                _pServer->startAdvertising();
            }
        } else {
            // Каждые 2.5 секунды при отсутствии подключения гарантируем активность рекламы
            static uint32_t lastAdvCheck = 0;
            if (millis() - lastAdvCheck > 2500) {
                lastAdvCheck = millis();
                if (_pServer) {
                    _pServer->startAdvertising();
                }
            }
        }
    } else if (!_oldDeviceConnected) {
        _oldDeviceConnected = true;
    }
}

void BleEngine::onConnect(BLEServer* pServer) {
    _deviceConnected = true;
    _oldDeviceConnected = true;
    s_latestRssi = -55;
    Serial.println("[BLE] Smartphone connected to GATT Server!");
}

void BleEngine::onDisconnect(BLEServer* pServer) {
    _deviceConnected = false;
    _oldDeviceConnected = false;
    s_latestRssi = -99;
    Serial.println("[BLE] Smartphone disconnected. Resuming advertising...");
    delay(20);
    pServer->startAdvertising();
}

void BleEngine::onWrite(BLECharacteristic* pCharacteristic) {
    String rxVal = pCharacteristic->getValue();
    if (rxVal.length() == 0) return;

    for (size_t i = 0; i < rxVal.length(); i++) {
        char c = rxVal[i];
        if (c == '\n' || c == '\r') {
            if (_rxAccumulatorLen > 0) {
                _rxAccumulator[_rxAccumulatorLen] = '\0';
                _parseIncomingLine(_rxAccumulator);
                _rxAccumulatorLen = 0;
            }
        } else {
            if (_rxAccumulatorLen < sizeof(_rxAccumulator) - 1) {
                _rxAccumulator[_rxAccumulatorLen++] = c;
                if (c == '}' && _rxAccumulator[0] == '{') {
                    _rxAccumulator[_rxAccumulatorLen] = '\0';
                    _parseIncomingLine(_rxAccumulator);
                    _rxAccumulatorLen = 0;
                }
            } else {
                _rxAccumulatorLen = 0; // Защита от переполнения
            }
        }
    }
}

void BleEngine::_parseIncomingLine(const char* line) {
    if (!line || line[0] == '\0') return;

    String trimmed = line;
    trimmed.trim();
    if (trimmed.length() == 0) return;

    Serial.printf("[BLE RX] %s\n", trimmed.c_str());

    String cmd = "";
    String val = "";

    // 1. Простой парсер JSON {"cmd":"...", "val":...}
    int cmdIdx = trimmed.indexOf("\"cmd\"");
    if (cmdIdx >= 0) {
        int colonIdx = trimmed.indexOf(':', cmdIdx);
        if (colonIdx >= 0) {
            int q1 = trimmed.indexOf('\"', colonIdx);
            if (q1 >= 0) {
                int q2 = trimmed.indexOf('\"', q1 + 1);
                if (q2 >= 0) {
                    cmd = trimmed.substring(q1 + 1, q2);
                }
            }
        }
    }

    int valIdx = trimmed.indexOf("\"val\"");
    if (valIdx >= 0) {
        int colonIdx = trimmed.indexOf(':', valIdx);
        if (colonIdx >= 0) {
            int endIdx = trimmed.indexOf(',', colonIdx);
            if (endIdx < 0) endIdx = trimmed.indexOf('}', colonIdx);
            if (endIdx >= 0) {
                val = trimmed.substring(colonIdx + 1, endIdx);
                val.trim();
                if (val.startsWith("\"") && val.endsWith("\"")) {
                    val = val.substring(1, val.length() - 1);
                }
            }
        }
    }

    // 2. Резервный текстовый парсер (если отправлена обычная строка "ARM", "RESET" и т.д.)
    if (cmd.length() == 0) {
        int spaceIdx = trimmed.indexOf(' ');
        if (spaceIdx >= 0) {
            cmd = trimmed.substring(0, spaceIdx);
            val = trimmed.substring(spaceIdx + 1);
        } else {
            cmd = trimmed;
        }
        cmd.toLowerCase();
    }

    if (_cmdHandler && cmd.length() > 0) {
        _cmdHandler(cmd, val);
    }
}

void BleEngine::sendJson(const char* jsonStr) {
    if (!_deviceConnected || !_pTxCharacteristic || !jsonStr) return;

    size_t len = strlen(jsonStr);
    if (len == 0) return;

    // Чанкирование по 100 байт: гарантирует передачу полных строк с \n при любом MTU
    const size_t CHUNK_SIZE = 100;
    if (len <= CHUNK_SIZE) {
        _pTxCharacteristic->setValue((uint8_t*)jsonStr, len);
        _pTxCharacteristic->notify();
    } else {
        size_t sent = 0;
        while (sent < len) {
            size_t toSend = len - sent;
            if (toSend > CHUNK_SIZE) toSend = CHUNK_SIZE;
            _pTxCharacteristic->setValue((uint8_t*)(jsonStr + sent), toSend);
            _pTxCharacteristic->notify();
            sent += toSend;
            if (sent < len) {
                delay(3); // Небольшая пауза между фрагментами для буфера стека
            }
        }
    }
}

void BleEngine::sendJson(const String& jsonStr) {
    sendJson(jsonStr.c_str());
}

void BleEngine::sendLiveTelemetry(
    const GpsData& gps,
    const ImuData& imu,
    RaceState state,
    RaceDiscipline disc,
    float liveTimeSec,
    float liveDistanceM,
    float liveSpeedKmh,
    float liveSlopePct,
    float batVolts,
    uint8_t batPct
) {
    if (!_deviceConnected) return;

    float gVal = (!isnan(imu.gLongitudinal) && !isinf(imu.gLongitudinal)) ? imu.gLongitudinal : 0.0f;
    float gxVal = (!isnan(imu.accelX) && !isinf(imu.accelX)) ? imu.accelX : 0.0f;
    float gyVal = (!isnan(imu.accelY) && !isinf(imu.accelY)) ? imu.accelY : 0.0f;
    float gzVal = (!isnan(imu.accelZ) && !isinf(imu.accelZ)) ? imu.accelZ : 0.0f;
    float gPeakVal = (!isnan(imu.gPeakAccel) && !isinf(imu.gPeakAccel)) ? imu.gPeakAccel : 0.0f;
    float slopeVal = (!isnan(liveSlopePct) && !isinf(liveSlopePct)) ? liveSlopePct : 0.0f;
    float spdVal = (!isnan(liveSpeedKmh) && !isinf(liveSpeedKmh)) ? liveSpeedKmh : 0.0f;
    float distVal = (!isnan(liveDistanceM) && !isinf(liveDistanceM)) ? liveDistanceM : 0.0f;
    float timeVal = (!isnan(liveTimeSec) && !isinf(liveTimeSec)) ? liveTimeSec : 0.0f;

    snprintf(
        _txBuffer, sizeof(_txBuffer),
        "{\"t\":\"live\",\"spd\":%.2f,\"dist\":%.1f,\"time\":%.3f,\"g\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"g_peak\":%.2f,"
        "\"sats\":%u,\"gps\":%u,\"glo\":%u,\"gal\":%u,\"bds\":%u,\"fix\":%u,\"hacc\":%.1f,\"vacc\":%.1f,\"sacc\":%.2f,\"pdop\":%.2f,"
        "\"state\":%u,\"disc\":%u,\"slope\":%.2f,\"alt\":%.1f,\"lat\":%.6f,\"lon\":%.6f,\"head\":%.1f,"
        "\"utc\":\"%02u:%02u:%02u\",\"date\":\"%02u.%02u.%04u\",\"bat\":%.2f,\"pct\":%u,\"rssi\":%d}\n",
        spdVal,
        distVal,
        timeVal,
        gVal,
        gxVal,
        gyVal,
        gzVal,
        gPeakVal,
        gps.numSats,
        gps.satsGps,
        gps.satsGlonass,
        gps.satsGalileo,
        gps.satsBeidou,
        gps.fixType,
        gps.hAccM,
        gps.vAccM,
        gps.sAccKmh,
        gps.pDOP,
        (uint8_t)state,
        (uint8_t)disc,
        slopeVal,
        gps.altMSL,
        gps.lat,
        gps.lon,
        gps.headingDeg,
        gps.hour, gps.min, gps.sec,
        gps.day, gps.month, gps.year,
        batVolts,
        batPct,
        (int)s_latestRssi
    );

    sendJson(_txBuffer);
}

void BleEngine::sendSplitEvent(const char* splitName, float timeSec, float trapSpeedKmh) {
    if (!_deviceConnected) return;

    snprintf(
        _txBuffer, sizeof(_txBuffer),
        "{\"t\":\"split\",\"name\":\"%s\",\"time\":%.3f,\"spd\":%.2f}\n",
        splitName,
        timeSec,
        trapSpeedKmh
    );

    sendJson(_txBuffer);
}

void BleEngine::sendRunRecord(const RunRecord& run) {
    if (!_deviceConnected) return;

    snprintf(
        _txBuffer, sizeof(_txBuffer),
        "{\"t\":\"run\",\"id\":%u,\"ts\":%u,\"disc\":%u,\"valid\":%s,\"slope\":%.2f,\"rollout\":%s,"
        "\"s0_60\":%.3f,\"s0_100\":%.3f,\"s100_150\":%.3f,\"s100_200\":%.3f,\"s0_200\":%.3f,\"s200_300\":%.3f,"
        "\"s60ft\":%.3f,\"s330ft\":%.3f,\"s1_8mi\":%.3f,\"s1000ft\":%.3f,\"s1_4mi\":%.3f,\"trap_spd\":%.2f,"
        "\"max_spd\":%.2f,\"max_g\":%.2f,\"dist\":%.1f,\"dur\":%.3f,\"b100_0_t\":%.3f,\"b100_0_d\":%.2f}\n",
        (unsigned int)run.id,
        (unsigned int)run.timestampUtc,
        (uint8_t)run.discipline,
        run.isValidSlope ? "true" : "false",
        run.slopePct,
        run.rolloutUsed ? "true" : "false",
        run.split0_60.achieved ? run.split0_60.timeSec : 0.0f,
        run.split0_100.achieved ? run.split0_100.timeSec : 0.0f,
        run.split100_150.achieved ? run.split100_150.timeSec : 0.0f,
        run.split100_200.achieved ? run.split100_200.timeSec : 0.0f,
        run.split0_200.achieved ? run.split0_200.timeSec : 0.0f,
        run.split200_300.achieved ? run.split200_300.timeSec : 0.0f,
        run.split60ft.achieved ? run.split60ft.timeSec : 0.0f,
        run.split330ft.achieved ? run.split330ft.timeSec : 0.0f,
        run.split1_8mi.achieved ? run.split1_8mi.timeSec : 0.0f,
        run.split1000ft.achieved ? run.split1000ft.timeSec : 0.0f,
        run.split1_4mi.achieved ? run.split1_4mi.timeSec : 0.0f,
        run.split1_4mi.achieved ? run.split1_4mi.trapSpeedKmh : run.maxSpeedKmh,
        run.maxSpeedKmh,
        run.maxAccelG,
        run.totalDistanceM,
        run.totalDurationSec,
        run.split100_0.achieved ? run.split100_0.timeSec : 0.0f,
        run.brakeDist100_0M
    );

    sendJson(_txBuffer);
}

void BleEngine::sendPersonalBests(const PersonalBests& pb) {
    if (!_deviceConnected) return;

    snprintf(
        _txBuffer, sizeof(_txBuffer),
        "{\"t\":\"pb\",\"best0_60\":%.3f,\"best0_100\":%.3f,\"best100_200\":%.3f,\"best1_4mi\":%.3f,\"best1_4mi_spd\":%.2f,\"best60ft\":%.3f,\"best100_0_d\":%.2f}\n",
        pb.best0_60,
        pb.best0_100,
        pb.best100_200,
        pb.best1_4mi,
        pb.best1_4miSpeed,
        pb.best60ft,
        pb.best100_0Dist
    );

    sendJson(_txBuffer);
}

void BleEngine::sendDeviceInfo(const DeviceSettings& settings, uint8_t runsCount, bool gpsReady, uint8_t sats, float batVolts, uint8_t batPct) {
    if (!_deviceConnected) return;

    snprintf(
        _txBuffer, sizeof(_txBuffer),
        "{\"t\":\"info\",\"fw\":\"%s\",\"name\":\"%s\",\"rollout\":%s,\"metric\":%s,\"slope_tol\":%.2f,\"runs_cnt\":%u,\"calibrated\":true,\"gps_ready\":%s,\"sats\":%u,\"bat\":%.2f,\"pct\":%u,\"bat_mode\":%u}\n",
        DRAGON_FW_VERSION,
        BLE_DEVICE_NAME,
        settings.use1FootRollout ? "true" : "false",
        settings.metricUnits ? "true" : "false",
        settings.slopeTolerancePct,
        runsCount,
        gpsReady ? "true" : "false",
        sats,
        batVolts,
        batPct,
        settings.batteryIndicationMode
    );

    sendJson(_txBuffer);
}

void BleEngine::sendDiagnostics(
    bool imuOk,
    const char* imuMsg,
    bool gpsOk,
    const char* gpsMsg,
    uint8_t gpsRateHz,
    uint32_t gpsBaud,
    bool storageOk,
    bool batOk,
    float batVolts,
    uint8_t batPct
) {
    if (!_deviceConnected) return;

    snprintf(
        _txBuffer, sizeof(_txBuffer),
        "{\"t\":\"diag\",\"imu_ok\":%s,\"imu_msg\":\"%s\",\"gps_ok\":%s,\"gps_msg\":\"%s\",\"gps_rate\":%u,\"gps_baud\":%u,\"storage_ok\":%s,\"bat_ok\":%s,\"bat_v\":%.2f,\"bat_pct\":%u,\"heap\":%u,\"min_heap\":%u,\"fw\":\"%s\"}\n",
        imuOk ? "true" : "false",
        imuMsg ? imuMsg : "OK",
        gpsOk ? "true" : "false",
        gpsMsg ? gpsMsg : "OK",
        gpsRateHz,
        (unsigned int)gpsBaud,
        storageOk ? "true" : "false",
        batOk ? "true" : "false",
        batVolts,
        batPct,
        (unsigned int)esp_get_free_heap_size(),
        (unsigned int)esp_get_minimum_free_heap_size(),
        DRAGON_FW_VERSION
    );

    sendJson(_txBuffer);
}
