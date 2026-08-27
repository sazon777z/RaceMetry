#include "ImuEngine.h"

// Регистры MPU-9250 / MPU-6500 / MPU-6050
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_CONFIG2   0x1D
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_PWR_MGMT_2      0x6C
#define MPU_REG_WHO_AM_I        0x75

ImuEngine::ImuEngine()
    : _isInitialized(false),
      _i2cAddr(IMU_I2C_ADDR),
      _offsetX(0.0f),
      _offsetY(0.0f),
      _offsetZ(0.0f),
      _filtAx(0.0f),
      _filtAy(0.0f),
      _filtAz(0.0f),
      _sdaPin(PIN_I2C_SDA),
      _sclPin(PIN_I2C_SCL),
      _freq(I2C_FREQUENCY),
      _errorCount(0)
{
    memset(&_data, 0, sizeof(ImuData));
}

void ImuEngine::_recoverI2cBus(uint8_t sda, uint8_t scl) {
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    delay(10);
}

bool ImuEngine::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t freq) {
    _sdaPin = sdaPin;
    _sclPin = sclPin;
    _freq = freq;
    _errorCount = 0;

    Serial.printf("[IMU] Initializing MPU on SDA: %u, SCL: %u (%u Hz)...\n", sdaPin, sclPin, freq);

    // 1. Инициализация I2C шины
    Wire.end();
    delay(10);
    Wire.begin(sdaPin, sclPin, freq);
    delay(30);

    // 2. Проверяем адрес 0x68 и 0x69
    bool found = false;
    uint8_t addrs[2] = { 0x68, 0x69 };
    for (uint8_t a : addrs) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            _i2cAddr = a;
            found = true;
            Serial.printf("[IMU] Detected I2C device at 0x%02X\n", a);
            break;
        }
    }

    if (!found) {
        // Попробуем на 100 кГц
        Wire.setClock(100000);
        for (uint8_t a : addrs) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) {
                _i2cAddr = a;
                found = true;
                Serial.printf("[IMU] Detected I2C device at 0x%02X (100kHz)\n", a);
                break;
            }
        }
    }

    if (!found) {
        Serial.println("[IMU] WARNING: No I2C response at 0x68 or 0x69. Defaulting to 0x68.");
        _i2cAddr = 0x68;
    }

    // 3. Сброс и пробуждение
    _writeRegister(MPU_REG_PWR_MGMT_1, 0x80); // Reset
    delay(50);
    _writeRegister(MPU_REG_PWR_MGMT_1, 0x00); // Wake up (Clear Sleep bit)
    delay(20);
    _writeRegister(MPU_REG_PWR_MGMT_1, 0x01); // Clock Source PLL
    delay(10);
    _writeRegister(MPU_REG_PWR_MGMT_2, 0x00); // Enable all 6 axes
    delay(10);

    // 4. Фильтр и диапазоны
    _writeRegister(MPU_REG_CONFIG, 0x03);         // DLPF 41 Hz
    _writeRegister(MPU_REG_ACCEL_CONFIG, 0x08);    // ±4G (8192 LSB/g)
    _writeRegister(MPU_REG_ACCEL_CONFIG2, 0x03);   // Accel DLPF 41 Hz
    _writeRegister(MPU_REG_GYRO_CONFIG, 0x08);     // ±500 deg/s
    _writeRegister(MPU_REG_SMPLRT_DIV, 0x04);      // 200 Hz

    _isInitialized = true;
    Serial.printf("[IMU] MPU-9250 initialized on 0x%02X at 200 Hz\n", _i2cAddr);
    return true;
}

bool ImuEngine::update() {
    uint8_t rawBuf[14];
    if (!_readRegisters(MPU_REG_ACCEL_XOUT_H, rawBuf, 14)) {
        _errorCount++;
        if (_errorCount > 40) {
            _isInitialized = false;
            // Пробуем мягкую переинициализацию
            begin(_sdaPin, _sclPin, _freq);
        }
        return false;
    }
    _errorCount = 0;
    _isInitialized = true;

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

    // Экспоненциальный фильтр (Alpha = 0.25 для плавности)
    _filtAx += 0.25f * (ax - _filtAx);
    _filtAy += 0.25f * (ay - _filtAy);
    _filtAz += 0.25f * (az - _filtAz);

    _data.accelX = _filtAx;
    _data.accelY = _filtAy;
    _data.accelZ = _filtAz;

    _data.gyroX = (float)rawGx / GYRO_SCALE;
    _data.gyroY = (float)rawGy / GYRO_SCALE;
    _data.gyroZ = (float)rawGz / GYRO_SCALE;

    // Продольное и поперечное ускорение
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
