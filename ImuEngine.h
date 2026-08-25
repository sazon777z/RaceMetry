#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                    IMU ENGINE (MPU-9250 / MPU-6500)
 * ============================================================================
 * Драйвер скоростного опроса акселерометра и гироскопа (200-500 Гц).
 * Отвечает за:
 * - Мгновенную фиксацию старта (Launch Jerk Trigger < 1 мс)
 * - Расчет перегрузок (G-Force) во всех плоскостях
 * - Калибровку нуля и фильтрацию вибраций двигателя
 */

class ImuEngine {
public:
    ImuEngine();

    // Инициализация I2C шины и регистров MPU-9250
    bool begin(uint8_t sdaPin = PIN_I2C_SDA, uint8_t sclPin = PIN_I2C_SCL, uint32_t freq = I2C_FREQUENCY);

    // Опрос датчика (вызывать в скоростном цикле)
    bool update();

    // Калибровка нуля (автомобиль должен стоять на ровной поверхности)
    void calibrateZero(uint16_t sampleCount = 500);

    // Установка калибровочных оффсетов из NVS
    void setOffsets(float axOffset, float ayOffset, float azOffset);
    void getOffsets(float& axOffset, float& ayOffset, float& azOffset) const;

    // Сброс пиковых значений G
    void resetPeaks();

    // Получить текущие данные
    const ImuData& getData() const { return _data; }

    // Проверка готовности датчика
    bool isReady() const { return _isInitialized; }

    // Проверка, есть ли резкий рывок вперед (старт)
    bool isLaunchDetected() const;

private:
    bool _isInitialized;
    uint8_t _i2cAddr;
    ImuData _data;

    float _offsetX;
    float _offsetY;
    float _offsetZ;

    // Коэффициент чувствительности (для диапазона ±4G -> 8192.0 LSB/g)
    const float ACCEL_SCALE = 8192.0f;
    const float GYRO_SCALE = 65.5f; // для диапазона ±500 deg/s

    // Экспоненциальный фильтр скользящего среднего
    float _filtAx;
    float _filtAy;
    float _filtAz;

    bool _writeRegister(uint8_t reg, uint8_t value);
    bool _readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
};
