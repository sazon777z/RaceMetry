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
 * - Сохранение пользовательских настроек (1-foot rollout, яркость)
 * - Сохранение калибровочных оффсетов акселерометра MPU-9250
 * - Хранение истории лучших заездов (Personal Bests) и последних заездов
 */

class StorageManager {
public:
    StorageManager();

    bool begin();

    // Настройки устройства
    bool loadSettings(DeviceSettings& settings);
    bool saveSettings(const DeviceSettings& settings);

    // История заездов
    bool saveRunRecord(const RunRecord& run);
    uint8_t getSavedRunsCount();
    bool getRunRecord(uint8_t index, RunRecord& run);
    void clearAllRuns();

    // Личные рекорды
    void getPersonalBests(float& best0_100, float& best100_200, float& best1_4mi, float& best1_4miSpeed);

private:
    Preferences _prefs;
    uint8_t _runCount;
    uint8_t _writeIndex;

    void _updatePersonalBests(const RunRecord& run);
};
