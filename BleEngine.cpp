#include "BleEngine.h"

BleEngine::BleEngine()
    : _pServer(nullptr),
      _pService(nullptr),
      _pTxCharacteristic(nullptr),
      _pRxCharacteristic(nullptr),
      _deviceConnected(false),
      _oldDeviceConnected(false),
      _lastTxTimeMs(0),
      _cmdHandler(nullptr)
{
    memset(_txBuffer, 0, sizeof(_txBuffer));
    _rxAccumulator.reserve(256);
}

bool BleEngine::begin(const char* deviceName) {
    Serial.printf("[BLE] Initializing BLE Server: %s\n", deviceName);

    // 1. Инициализация стека BLE Device
    BLEDevice::init(deviceName);

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

    // 7. Настройка и запуск рекламы (Advertising)
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_NUS_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // 7.5ms (совместимость с iOS и Android)
    pAdvertising->setMaxPreferred(0x12); // 22.5ms
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising started. Waiting for smartphone connection...");
    return true;
}

void BleEngine::update() {
    // Внутренний контроль статуса
    if (!_deviceConnected && _oldDeviceConnected) {
        _oldDeviceConnected = false;
        if (_pServer) {
            _pServer->startAdvertising();
        }
    }
    if (_deviceConnected && !_oldDeviceConnected) {
        _oldDeviceConnected = true;
    }
}

void BleEngine::onConnect(BLEServer* pServer) {
    _deviceConnected = true;
    _oldDeviceConnected = true;
    Serial.println("[BLE] Smartphone connected to GATT Server!");
}

void BleEngine::onDisconnect(BLEServer* pServer) {
    _deviceConnected = false;
    _oldDeviceConnected = false;
    Serial.println("[BLE] Smartphone disconnected. Resuming advertising...");
    pServer->startAdvertising();
}

void BleEngine::onWrite(BLECharacteristic* pCharacteristic) {
    String rxVal = pCharacteristic->getValue();
    if (rxVal.length() == 0) return;

    for (size_t i = 0; i < rxVal.length(); i++) {
        char c = rxVal[i];
        if (c == '\n' || c == '\r') {
            if (_rxAccumulator.length() > 0) {
                _parseIncomingLine(_rxAccumulator);
                _rxAccumulator = "";
            }
        } else {
            _rxAccumulator += c;
            if (c == '}' && _rxAccumulator.startsWith("{")) {
                _parseIncomingLine(_rxAccumulator);
                _rxAccumulator = "";
            }
        }
    }
}

void BleEngine::_parseIncomingLine(const String& line) {
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

    _pTxCharacteristic->setValue((uint8_t*)jsonStr, len);
    _pTxCharacteristic->notify();
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

    snprintf(
        _txBuffer, sizeof(_txBuffer),
        "{\"t\":\"live\",\"spd\":%.2f,\"dist\":%.1f,\"time\":%.3f,\"g\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"g_peak\":%.2f,"
        "\"sats\":%u,\"gps\":%u,\"glo\":%u,\"gal\":%u,\"bds\":%u,\"fix\":%u,\"hacc\":%.1f,\"vacc\":%.1f,\"sacc\":%.2f,\"pdop\":%.2f,"
        "\"state\":%u,\"disc\":%u,\"slope\":%.2f,\"alt\":%.1f,\"lat\":%.6f,\"lon\":%.6f,\"head\":%.1f,"
        "\"utc\":\"%02u:%02u:%02u\",\"date\":\"%02u.%02u.%04u\",\"bat\":%.2f,\"pct\":%u}\n",
        liveSpeedKmh,
        liveDistanceM,
        liveTimeSec,
        imu.gLongitudinal,
        imu.accelX,
        imu.accelY,
        imu.accelZ,
        imu.gPeakAccel,
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
        liveSlopePct,
        gps.altMSL,
        gps.lat,
        gps.lon,
        gps.headingDeg,
        gps.hour, gps.min, gps.sec,
        gps.day, gps.month, gps.year,
        batVolts,
        batPct
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
        "{\"t\":\"info\",\"fw\":\"%s\",\"name\":\"%s\",\"rollout\":%s,\"metric\":%s,\"slope_tol\":%.2f,\"runs_cnt\":%u,\"calibrated\":true,\"gps_ready\":%s,\"sats\":%u,\"bat\":%.2f,\"pct\":%u}\n",
        DRAGON_FW_VERSION,
        BLE_DEVICE_NAME,
        settings.use1FootRollout ? "true" : "false",
        settings.metricUnits ? "true" : "false",
        settings.slopeTolerancePct,
        runsCount,
        gpsReady ? "true" : "false",
        sats,
        batVolts,
        batPct
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
        "{\"t\":\"diag\",\"imu_ok\":%s,\"imu_msg\":\"%s\",\"gps_ok\":%s,\"gps_msg\":\"%s\",\"gps_rate\":%u,\"gps_baud\":%u,\"storage_ok\":%s,\"bat_ok\":%s,\"bat_v\":%.2f,\"bat_pct\":%u,\"fw\":\"%s\"}\n",
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
        DRAGON_FW_VERSION
    );

    sendJson(_txBuffer);
}
