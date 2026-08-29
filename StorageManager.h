#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                    STORAGE MANAGER (ESP32 NVS PREFERENCES)
 * ============================================================================
 * Управление энергонезависимой памятью NVS:
 * - Версионированное хранилище заездов (Magic, Schema Version, CRC32)
 * - Автоматическое присвоение уникальных монотонных ID заездов
 * - Сохранение пользовательских настроек
 * - Хранение и раздельный сброс личных рекордов (Personal Bests)
 */

struct StorageRecordHeader {
    uint16_t magic;         // NVS_STORAGE_MAGIC (0x524D)
    uint8_t  schemaVersion; // NVS_SCHEMA_VERSION (1)
    uint8_t  reserved;      // 0
    uint32_t payloadLength; // sizeof(RunRecord)
    uint32_t runId;
    uint32_t crc32;
};

class StorageManager {
public:
    StorageManager();

    bool begin();
    bool isOk() const { return _isInitialized; }

    // Настройки устройства
    bool loadSettings(DeviceSettings& settings);
    bool saveSettings(const DeviceSettings& settings);

    // История заездов
    bool saveRunRecord(RunRecord& run);
    uint8_t getSavedRunsCount() const { return _runCount; }
    bool getRunRecord(uint8_t index, RunRecord& run);
    void clearAllRuns(bool clearPersonalBests = true);
    void clearPersonalBests();

    // Генерация ID
    uint32_t getNextRunId();

    // Личные рекорды
    void getPersonalBests(PersonalBests& pb);
    void getPersonalBests(float& best0_100, float& best100_200, float& best1_4mi, float& best1_4miSpeed);

private:
    Preferences _prefs;
    bool _isInitialized;
    uint8_t _runCount;
    uint8_t _writeIndex;
    uint32_t _nextRunId;

    void _updatePersonalBests(const RunRecord& run);
};

