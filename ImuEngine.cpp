#include "ImuEngine.h"

// Регистры MPU-9250 / MPU-6500
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_CONFIG2   0x1D
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_WHO_AM_I        0x75

ImuEngine::ImuEngine()
    : _isInitialized(false),
      _i2cAddr(IMU_I2C_ADDR),
      _offsetX(0.0f),
      _offsetY(0.0f),
      _offsetZ(0.0f),
      _filtAx(0.0f),
      _filtAy(0.0f),
      _filtAz(0.0f)
{
    memset(&_data, 0, sizeof(ImuData));
}

bool ImuEngine::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t freq) {
    _isInitialized = false;
    Wire.begin(sdaPin, sclPin, freq);
    delay(50);

    // Проверяем WhoAmI (для MPU-9250 = 0x71, 0x73, MPU-6500 = 0x70, MPU-9255 = 0x73)
    uint8_t whoAmI = 0;
    if (!_readRegisters(MPU_REG_WHO_AM_I, &whoAmI, 1)) {
        // Попробуем альтернативный адрес 0x69
        _i2cAddr = 0x69;
        if (!_readRegisters(MPU_REG_WHO_AM_I, &whoAmI, 1)) {
            _i2cAddr = IMU_I2C_ADDR; // Возвращаем по умолчанию
            return false;
        }
    }

    // 1. Сброс и пробуждение (Power Management: тактирование от гироскопа)
    _writeRegister(MPU_REG_PWR_MGMT_1, 0x80); // Reset
    delay(100);
    _writeRegister(MPU_REG_PWR_MGMT_1, 0x01); // Auto select clock source PLL
    delay(20);

    // 2. Настройка цифрового фильтра низкой частоты (DLPF 41 Гц) для фильтрации вибраций ДВС
    _writeRegister(MPU_REG_CONFIG, 0x03);

    // 3. Диапазон акселерометра ±4G (8192 LSB/g) — оптимально для авто
    _writeRegister(MPU_REG_ACCEL_CONFIG, 0x08);
    _writeRegister(MPU_REG_ACCEL_CONFIG2, 0x03); // Accel DLPF 41 Hz

    // 4. Диапазон гироскопа ±500 град/с
    _writeRegister(MPU_REG_GYRO_CONFIG, 0x08);

    // 5. Частота выборки (Sample Rate Divider = 4 при 1 кГц базовой -> 200 Гц)
    _writeRegister(MPU_REG_SMPLRT_DIV, 0x04);

    _data.isCalibrated = false;
    _isInitialized = true;
    return true;
}

bool ImuEngine::update() {
    uint8_t rawBuf[14];
    if (!_readRegisters(MPU_REG_ACCEL_XOUT_H, rawBuf, 14)) {
        return false;
    }

    // Разбор сырых значений акселерометра
    int16_t rawAx = (int16_t)((rawBuf[0] << 8) | rawBuf[1]);
    int16_t rawAy = (int16_t)((rawBuf[2] << 8) | rawBuf[3]);
    int16_t rawAz = (int16_t)((rawBuf[4] << 8) | rawBuf[5]);

    // Разбор гироскопа
    int16_t rawGx = (int16_t)((rawBuf[8] << 8) | rawBuf[9]);
    int16_t rawGy = (int16_t)((rawBuf[10] << 8) | rawBuf[11]);
    int16_t rawGz = (int16_t)((rawBuf[12] << 8) | rawBuf[13]);

    // Перевод в физические величины
    float ax = (float)rawAx / ACCEL_SCALE - _offsetX;
    float ay = (float)rawAy / ACCEL_SCALE - _offsetY;
    float az = (float)rawAz / ACCEL_SCALE - _offsetZ;

    // Экспоненциальный фильтр (Alpha = 0.25 для гладкости)
    _filtAx += 0.25f * (ax - _filtAx);
    _filtAy += 0.25f * (ay - _filtAy);
    _filtAz += 0.25f * (az - _filtAz);

    _data.accelX = _filtAx;
    _data.accelY = _filtAy;
    _data.accelZ = _filtAz;

    _data.gyroX = (float)rawGx / GYRO_SCALE;
    _data.gyroY = (float)rawGy / GYRO_SCALE;
    _data.gyroZ = (float)rawGz / GYRO_SCALE;

    // Продольное ускорение (направление вперед): в зависимости от монтажа платы
    // По умолчанию считаем, что ось X направлена вперед автомобиля, ось Y — вправо
    _data.gLongitudinal = _data.accelX;
    _data.gLateral = _data.accelY;
    _data.gVertical = _data.accelZ;

    // Обновление пиков
    if (_data.gLongitudinal > _data.gPeakAccel) {
        _data.gPeakAccel = _data.gLongitudinal;
    }
    if (_data.gLongitudinal < -_data.gPeakBrake) {
        _data.gPeakBrake = -_data.gLongitudinal;
    }

    _data.lastSampleUs = micros();
    return true;
}

void ImuEngine::calibrateZero(uint16_t sampleCount) {
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (uint16_t i = 0; i < sampleCount; i++) {
        uint8_t rawBuf[6];
        if (_readRegisters(MPU_REG_ACCEL_XOUT_H, rawBuf, 6)) {
            int16_t rawAx = (int16_t)((rawBuf[0] << 8) | rawBuf[1]);
            int16_t rawAy = (int16_t)((rawBuf[2] << 8) | rawBuf[3]);
            int16_t rawAz = (int16_t)((rawBuf[4] << 8) | rawBuf[5]);

            sumX += (float)rawAx / ACCEL_SCALE;
            sumY += (float)rawAy / ACCEL_SCALE;
            sumZ += (float)rawAz / ACCEL_SCALE;
        }
        delay(2);
    }

    _offsetX = sumX / (float)sampleCount;
    _offsetY = sumY / (float)sampleCount;
    // Ось Z при горизонтальной установке содержит 1.0G гравитации
    _offsetZ = (sumZ / (float)sampleCount) - 1.0f;

    _data.isCalibrated = true;
}

void ImuEngine::setOffsets(float axOffset, float ayOffset, float azOffset) {
    _offsetX = axOffset;
    _offsetY = ayOffset;
    _offsetZ = azOffset;
    _data.isCalibrated = true;
}

void ImuEngine::getOffsets(float& axOffset, float& ayOffset, float& azOffset) const {
    axOffset = _offsetX;
    ayOffset = _offsetY;
    azOffset = _offsetZ;
}

void ImuEngine::resetPeaks() {
    _data.gPeakAccel = 0.0f;
    _data.gPeakBrake = 0.0f;
}

bool ImuEngine::isLaunchDetected() const {
    return _data.gLongitudinal >= LAUNCH_G_THRESHOLD;
}

bool ImuEngine::_writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

bool ImuEngine::_readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t bytesRead = Wire.requestFrom(_i2cAddr, length);
    if (bytesRead != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}
