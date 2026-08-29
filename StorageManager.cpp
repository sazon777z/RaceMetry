#include "StorageManager.h"
#include <string.h>

static uint32_t calculateCRC32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(int32_t)(crc & 1)));
        }
    }
    return ~crc;
}

StorageManager::StorageManager()
    : _isInitialized(false),
      _runCount(0),
      _writeIndex(0),
      _nextRunId(1)
{
}

bool StorageManager::begin() {
    if (!_prefs.begin(NVS_NAMESPACE, false)) {
        _isInitialized = false;
        return false;
    }
    _runCount = _prefs.getUChar("run_cnt", 0);
    _writeIndex = _prefs.getUChar("run_idx", 0);
    _nextRunId = _prefs.getUInt("next_id", 1);
    if (_nextRunId == 0) _nextRunId = 1;
    _isInitialized = true;
    return true;
}

uint32_t StorageManager::getNextRunId() {
    uint32_t id = _nextRunId++;
    if (_isInitialized) {
        _prefs.putUInt("next_id", _nextRunId);
    }
    return id;
}

bool StorageManager::loadSettings(DeviceSettings& settings) {
    if (!_isInitialized) return false;

    if (!_prefs.isKey("cfg_init")) {
        // Значения по умолчанию
        settings.use1FootRollout = true;
        settings.metricUnits = true;
        settings.slopeTolerancePct = MAX_VALID_SLOPE_PCT;
        settings.displayBrightness = 220;
        settings.imuOffsetGx = 0.0f;
        settings.imuOffsetGy = 0.0f;
        settings.imuOffsetGz = 0.0f;
        settings.defaultScreen = 0;
        settings.batteryIndicationMode = 0;
        saveSettings(settings);
        return true;
    }

    settings.use1FootRollout = _prefs.getBool("rollout", true);
    settings.metricUnits = _prefs.getBool("metric", true);
    settings.slopeTolerancePct = _prefs.getFloat("slope_tol", MAX_VALID_SLOPE_PCT);
    settings.displayBrightness = _prefs.getUChar("bright", 220);
    settings.imuOffsetGx = _prefs.getFloat("off_gx", 0.0f);
    settings.imuOffsetGy = _prefs.getFloat("off_gy", 0.0f);
    settings.imuOffsetGz = _prefs.getFloat("off_gz", 0.0f);
    settings.defaultScreen = _prefs.getUChar("def_scr", 0);
    settings.batteryIndicationMode = _prefs.getUChar("bat_mode", 0);
    return true;
}

bool StorageManager::saveSettings(const DeviceSettings& settings) {
    if (!_isInitialized) return false;

    _prefs.putBool("cfg_init", true);
    _prefs.putBool("rollout", settings.use1FootRollout);
    _prefs.putBool("metric", settings.metricUnits);
    _prefs.putFloat("slope_tol", settings.slopeTolerancePct);
    _prefs.putUChar("bright", settings.displayBrightness);
    _prefs.putFloat("off_gx", settings.imuOffsetGx);
    _prefs.putFloat("off_gy", settings.imuOffsetGy);
    _prefs.putFloat("off_gz", settings.imuOffsetGz);
    _prefs.putUChar("def_scr", settings.defaultScreen);
    _prefs.putUChar("bat_mode", settings.batteryIndicationMode);
    return true;
}

bool StorageManager::saveRunRecord(RunRecord& run) {
    if (!_isInitialized) return false;

    if (run.id == 0) {
        run.id = getNextRunId();
    }

    char key[16];
    snprintf(key, sizeof(key), "run_%d", _writeIndex);

    // Упаковка в структурированный контейнер с версией и CRC32
    StorageRecordHeader header;
    header.magic = NVS_STORAGE_MAGIC;
    header.schemaVersion = NVS_SCHEMA_VERSION;
    header.reserved = 0;
    header.payloadLength = (uint32_t)sizeof(RunRecord);
    header.runId = run.id;
    header.crc32 = calculateCRC32((const uint8_t*)&run, sizeof(RunRecord));

    uint8_t buffer[sizeof(StorageRecordHeader) + sizeof(RunRecord)];
    memcpy(buffer, &header, sizeof(StorageRecordHeader));
    memcpy(buffer + sizeof(StorageRecordHeader), &run, sizeof(RunRecord));

    size_t written = _prefs.putBytes(key, buffer, sizeof(buffer));
    if (written != sizeof(buffer)) return false;

    _writeIndex = (_writeIndex + 1) % MAX_SAVED_RUNS;
    if (_runCount < MAX_SAVED_RUNS) _runCount++;

    _prefs.putUChar("run_cnt", _runCount);
    _prefs.putUChar("run_idx", _writeIndex);

    if (run.abortReason == RaceAbortReason::NONE) {
        _updatePersonalBests(run);
    }
    return true;
}

bool StorageManager::getRunRecord(uint8_t index, RunRecord& run) {
    if (!_isInitialized || index >= _runCount) return false;

    int actualIndex = ((int)_writeIndex - 1 - (int)index);
    if (actualIndex < 0) actualIndex += MAX_SAVED_RUNS;

    char key[16];
    snprintf(key, sizeof(key), "run_%d", actualIndex);

    uint8_t buffer[sizeof(StorageRecordHeader) + sizeof(RunRecord)];
    size_t readLen = _prefs.getBytes(key, buffer, sizeof(buffer));

    // Проверка версии 1 с заголовком и CRC
    if (readLen == sizeof(buffer)) {
        StorageRecordHeader header;
        memcpy(&header, buffer, sizeof(StorageRecordHeader));

        if (header.magic == NVS_STORAGE_MAGIC && header.schemaVersion == NVS_SCHEMA_VERSION && header.payloadLength == sizeof(RunRecord)) {
            const uint8_t* payloadPtr = buffer + sizeof(StorageRecordHeader);
            uint32_t calcCrc = calculateCRC32(payloadPtr, sizeof(RunRecord));
            if (calcCrc == header.crc32) {
                memcpy(&run, payloadPtr, sizeof(RunRecord));
                return true;
            }
        }
    }

    return false;
}

void StorageManager::clearAllRuns(bool clearPBs) {
    if (!_isInitialized) return;

    for (uint8_t i = 0; i < MAX_SAVED_RUNS; i++) {
        char key[16];
        snprintf(key, sizeof(key), "run_%d", i);
        _prefs.remove(key);
    }
    _runCount = 0;
    _writeIndex = 0;
    _prefs.putUChar("run_cnt", 0);
    _prefs.putUChar("run_idx", 0);

    if (clearPBs) {
        clearPersonalBests();
    }
}

void StorageManager::clearPersonalBests() {
    if (!_isInitialized) return;
    _prefs.remove("pb_0_60");
    _prefs.remove("pb_0_100");
    _prefs.remove("pb_100_200");
    _prefs.remove("pb_1_4mi");
    _prefs.remove("pb_1_4_spd");
    _prefs.remove("pb_60ft");
    _prefs.remove("pb_100_0_d");
}

void StorageManager::_updatePersonalBests(const RunRecord& run) {
    if (run.abortReason != RaceAbortReason::NONE) return;

    // 0 - 60 км/ч
    if (run.split0_60.achieved) {
        float best0_60 = _prefs.getFloat("pb_0_60", 999.0f);
        if (run.split0_60.timeSec < best0_60) {
            _prefs.putFloat("pb_0_60", run.split0_60.timeSec);
        }
    }
    // 0 - 100 км/ч
    if (run.split0_100.achieved) {
        float best0_100 = _prefs.getFloat("pb_0_100", 999.0f);
        if (run.split0_100.timeSec < best0_100) {
            _prefs.putFloat("pb_0_100", run.split0_100.timeSec);
        }
    }
    // 100 - 200 км/ч
    if (run.split100_200.achieved) {
        float best100_200 = _prefs.getFloat("pb_100_200", 999.0f);
        if (run.split100_200.timeSec < best100_200) {
            _prefs.putFloat("pb_100_200", run.split100_200.timeSec);
        }
    }
    // 1/4 мили
    if (run.split1_4mi.achieved) {
        float best1_4mi = _prefs.getFloat("pb_1_4mi", 999.0f);
        if (run.split1_4mi.timeSec < best1_4mi) {
            _prefs.putFloat("pb_1_4mi", run.split1_4mi.timeSec);
            _prefs.putFloat("pb_1_4_spd", run.split1_4mi.trapSpeedKmh);
        }
    }
    // 60 футов
    if (run.split60ft.achieved) {
        float best60ft = _prefs.getFloat("pb_60ft", 999.0f);
        if (run.split60ft.timeSec < best60ft) {
            _prefs.putFloat("pb_60ft", run.split60ft.timeSec);
        }
    }
    // 100 - 0 км/ч (торможение)
    if (run.split100_0.achieved && run.brakeDist100_0M > 0.1f) {
        float bestBrake = _prefs.getFloat("pb_100_0_d", 999.0f);
        if (run.brakeDist100_0M < bestBrake) {
            _prefs.putFloat("pb_100_0_d", run.brakeDist100_0M);
        }
    }
}

void StorageManager::getPersonalBests(PersonalBests& pb) {
    if (!_isInitialized) {
        memset(&pb, 0, sizeof(PersonalBests));
        return;
    }
    pb.best0_60 = _prefs.getFloat("pb_0_60", 0.0f);
    pb.best0_100 = _prefs.getFloat("pb_0_100", 0.0f);
    pb.best100_200 = _prefs.getFloat("pb_100_200", 0.0f);
    pb.best1_4mi = _prefs.getFloat("pb_1_4mi", 0.0f);
    pb.best1_4miSpeed = _prefs.getFloat("pb_1_4_spd", 0.0f);
    pb.best60ft = _prefs.getFloat("pb_60ft", 0.0f);
    pb.best100_0Dist = _prefs.getFloat("pb_100_0_d", 0.0f);
}

void StorageManager::getPersonalBests(float& best0_100, float& best100_200, float& best1_4mi, float& best1_4miSpeed) {
    if (!_isInitialized) {
        best0_100 = 0.0f; best100_200 = 0.0f; best1_4mi = 0.0f; best1_4miSpeed = 0.0f;
        return;
    }
    best0_100 = _prefs.getFloat("pb_0_100", 0.0f);
    best100_200 = _prefs.getFloat("pb_100_200", 0.0f);
    best1_4mi = _prefs.getFloat("pb_1_4mi", 0.0f);
    best1_4miSpeed = _prefs.getFloat("pb_1_4_spd", 0.0f);
}

