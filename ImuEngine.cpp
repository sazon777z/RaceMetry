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
    Wire.end();
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, OUTPUT);
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
    // 9 тактов тактирования SCL для принудительного освобождения зависшей шины SDA
    for (int i = 0; i < 9; i++) {
        digitalWrite(scl, LOW);
        delayMicroseconds(5);
        digitalWrite(scl, HIGH);
        delayMicroseconds(5);
    }
    pinMode(scl, INPUT_PULLUP);
    delay(5);
}

bool ImuEngine::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t freq) {
    _sdaPin = sdaPin;
    _sclPin = sclPin;
    _freq = freq;
    _errorCount = 0;

    // Аппаратная защита: таймаут 25 мс исключает любые зависания I2C на уровне драйвера
    Wire.setTimeOut(25);

    Serial.printf("[IMU] Probing I2C for MPU-9250/6500/6050 (Primary SDA: %u, SCL: %u)...\n", sdaPin, sclPin);

    // Пары пинов для проверки: 1) основные (13,12), 2) реверсивные (12,13)
    uint8_t pinPairs[2][2] = {
        { sdaPin, sclPin },
        { sclPin, sdaPin }
    };

    bool found = false;
    uint8_t activeSda = sdaPin;
    uint8_t activeScl = sclPin;

    for (int p = 0; p < 2; p++) {
        uint8_t curSda = pinPairs[p][0];
        uint8_t curScl = pinPairs[p][1];

        _recoverI2cBus(curSda, curScl);
        Wire.begin(curSda, curScl, freq);
        Wire.setTimeOut(25);
        delay(20);

        // Проверяем адреса 0x68 и 0x69
        uint8_t addrs[2] = { 0x68, 0x69 };
        for (uint8_t a : addrs) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) {
                _i2cAddr = a;
                activeSda = curSda;
                activeScl = curScl;
                found = true;
                Serial.printf("[IMU] SUCCESS: Found I2C device at 0x%02X on SDA: %u, SCL: %u!\n", a, curSda, curScl);
                break;
            }
        }
        if (found) break;

        // Попробуем на пониженной частоте 100 кГц
        Wire.setClock(100000);
        for (uint8_t a : addrs) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) {
                _i2cAddr = a;
                activeSda = curSda;
                activeScl = curScl;
                found = true;
                Serial.printf("[IMU] SUCCESS: Found I2C device at 0x%02X on SDA: %u, SCL: %u (100kHz)!\n", a, curSda, curScl);
                break;
            }
        }
        if (found) break;
    }

    // Если не найден на 0x68/0x69, сканируем всю шину I2C (1..127)
    if (!found) {
        Serial.println("[IMU] Full I2C bus scan (addresses 1..127):");
        Wire.end();
        Wire.begin(sdaPin, sclPin, 100000);
        int devCount = 0;
        for (uint8_t a = 1; a < 128; a++) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) {
                Serial.printf("[IMU] -> Detected unknown I2C device at address 0x%02X\n", a);
                _i2cAddr = a;
                found = true;
                devCount++;
            }
        }
        if (devCount == 0) {
            Serial.println("[IMU] CRITICAL: No I2C devices found on bus! Check SDA/SCL wiring and 3.3V power.");
            _i2cAddr = 0x68;
        }
    }

    _sdaPin = activeSda;
    _sclPin = activeScl;

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

    // 5. Тестовое считывание для валидации данных
    uint8_t testBuf[6] = {0};
    if (_readRegisters(MPU_REG_ACCEL_XOUT_H, testBuf, 6)) {
        int16_t ax = (int16_t)((testBuf[0] << 8) | testBuf[1]);
        int16_t ay = (int16_t)((testBuf[2] << 8) | testBuf[3]);
        int16_t az = (int16_t)((testBuf[4] << 8) | testBuf[5]);
        Serial.printf("[IMU LIVE TEST] Accelerometer OK: Ax=%d, Ay=%d, Az=%d (~%.2f G)\n", ax, ay, az, (float)az / ACCEL_SCALE);
    } else {
        Serial.println("[IMU LIVE TEST] Warning: Direct register read failed.");
    }

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

    if (isnan(ax) || isinf(ax)) ax = 0.0f;
    if (isnan(ay) || isinf(ay)) ay = 0.0f;
    if (isnan(az) || isinf(az)) az = 0.0f;

    // Экспоненциальный фильтр (Alpha = 0.25 для плавности)
    _filtAx += 0.25f * (ax - _filtAx);
    _filtAy += 0.25f * (ay - _filtAy);
    _filtAz += 0.25f * (az - _filtAz);

    if (isnan(_filtAx)) _filtAx = ax;
    if (isnan(_filtAy)) _filtAy = ay;
    if (isnan(_filtAz)) _filtAz = az;

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
    if (isnan(axOffset) || isinf(axOffset) || fabsf(axOffset) > 8.0f) axOffset = 0.0f;
    if (isnan(ayOffset) || isinf(ayOffset) || fabsf(ayOffset) > 8.0f) ayOffset = 0.0f;
    if (isnan(azOffset) || isinf(azOffset) || fabsf(azOffset) > 8.0f) azOffset = 0.0f;
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
