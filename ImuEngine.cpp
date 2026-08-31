#include "ImuEngine.h"

// Регистры MPU-9250 / MPU-6500 / MPU-6050 / ICM
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
      _whoAmI(0),
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
    strncpy(_statusMsg, "IMU: Ожидание инициализации", sizeof(_statusMsg) - 1);
}

void ImuEngine::_recoverI2cBus(uint8_t sda, uint8_t scl) {
    Wire.end();
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    delayMicroseconds(50);

    pinMode(scl, OUTPUT);
    digitalWrite(scl, HIGH);
    delayMicroseconds(10);

    // 16 тактов SCL для принудительного освобождения зависшей шины SDA
    for (int i = 0; i < 16; i++) {
        digitalWrite(scl, LOW);
        delayMicroseconds(10);
        digitalWrite(scl, HIGH);
        delayMicroseconds(10);
    }

    // Формируем STOP условие
    pinMode(sda, OUTPUT);
    digitalWrite(sda, LOW);
    delayMicroseconds(10);
    digitalWrite(scl, HIGH);
    delayMicroseconds(10);
    digitalWrite(sda, HIGH);
    delayMicroseconds(10);

    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    delay(10);
}

uint8_t ImuEngine::_probeWhoAmI(uint8_t addr) {
    uint8_t who = 0;

    // Попытка 1: чтение регистра 0x75 (WHO_AM_I MPU/ICM) с repeated start
    Wire.beginTransmission(addr);
    Wire.write(MPU_REG_WHO_AM_I);
    if (Wire.endTransmission(false) == 0) {
        if (Wire.requestFrom(addr, (uint8_t)1) == 1) {
            who = Wire.read();
        }
    }

    // Попытка 2: чтение с явным STOP
    if (who == 0 || who == 0xFF) {
        Wire.beginTransmission(addr);
        Wire.write(MPU_REG_WHO_AM_I);
        if (Wire.endTransmission(true) == 0) {
            if (Wire.requestFrom(addr, (uint8_t)1, (bool)true) == 1) {
                who = Wire.read();
            }
        }
    }

    // Попытка 3: для некоторых сенсоров ST/Bosch (регистр 0x0F)
    if (who == 0 || who == 0xFF) {
        Wire.beginTransmission(addr);
        Wire.write(0x0F);
        if (Wire.endTransmission(true) == 0) {
            if (Wire.requestFrom(addr, (uint8_t)1, (bool)true) == 1) {
                who = Wire.read();
            }
        }
    }

    return who;
}

bool ImuEngine::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t freq) {
    _sdaPin = sdaPin;
    _sclPin = sclPin;
    _freq = freq;
    _errorCount = 0;
    _isInitialized = false;

    Serial.printf("[IMU] Scanning I2C for Accelerometer (Requested SDA: %u, SCL: %u)...\n", sdaPin, sclPin);

    // Пары пинов для автоопределения:
    // 1) Заданные пины (sdaPin, sclPin)
    // 2) Реверсивные пины (sclPin, sdaPin)
    // 3) Альтернативные пины ESP32-S3 (12, 13)
    uint8_t pinPairs[3][2] = {
        { sdaPin, sclPin },
        { sclPin, sdaPin },
        { 12, 13 }
    };

    bool found = false;
    uint8_t activeSda = sdaPin;
    uint8_t activeScl = sclPin;
    uint8_t detectedAddr = 0;
    uint8_t detectedWho = 0;

    for (int p = 0; p < 3; p++) {
        uint8_t curSda = pinPairs[p][0];
        uint8_t curScl = pinPairs[p][1];

        // Пропускаем дубликаты
        if (p == 2 && ((curSda == sdaPin && curScl == sclPin) || (curSda == sclPin && curScl == sdaPin))) {
            continue;
        }

        _recoverI2cBus(curSda, curScl);

        // Начинаем со стандартной скорости 100 кГц для надежного обнаружения
        Wire.begin(curSda, curScl, 100000);
        Wire.setTimeOut(30);
        delay(25);

        // Сканируем все типичные адреса IMU датчиков (0x68, 0x69, 0x6A, 0x6B)
        const uint8_t candidateAddrs[] = { 0x68, 0x69, 0x6A, 0x6B };
        for (uint8_t a : candidateAddrs) {
            Wire.beginTransmission(a);
            uint8_t ack = Wire.endTransmission();
            if (ack == 0) {
                uint8_t who = _probeWhoAmI(a);
                Serial.printf("[IMU] Found I2C ACK at 0x%02X (WHO_AM_I: 0x%02X) on SDA: %u, SCL: %u\n", a, who, curSda, curScl);

                if (who != 0x00 && who != 0xFF) {
                    detectedAddr = a;
                    detectedWho = who;
                    activeSda = curSda;
                    activeScl = curScl;
                    found = true;
                    break;
                } else {
                    // Если устройство ответило ACK, но WHO_AM_I=0, пробуем разбудить
                    _i2cAddr = a;
                    _writeRegister(MPU_REG_PWR_MGMT_1, 0x00);
                    delay(15);
                    who = _probeWhoAmI(a);
                    if (who != 0x00 && who != 0xFF) {
                        detectedAddr = a;
                        detectedWho = who;
                        activeSda = curSda;
                        activeScl = curScl;
                        found = true;
                        break;
                    }
                }
            }
        }
        if (found) break;
    }

    if (!found) {
        snprintf(_statusMsg, sizeof(_statusMsg), "IMU: Ошибка I2C (нет ответа на GPIO %u/%u)", sdaPin, sclPin);
        Serial.printf("[IMU] ERROR: No response on I2C bus (SDA: %u, SCL: %u)!\n", sdaPin, sclPin);
        return false;
    }

    _sdaPin = activeSda;
    _sclPin = activeScl;
    _i2cAddr = detectedAddr;
    _whoAmI = detectedWho;

    // 1. Аппаратный сброс датчика
    _writeRegister(MPU_REG_PWR_MGMT_1, 0x80);
    delay(60);

    // 2. Пробуждение: сброс бита SLEEP (запись 0x00 в PWR_MGMT_1)
    _writeRegister(MPU_REG_PWR_MGMT_1, 0x00);
    delay(20);

    // 3. Выбор тактового генератора PLL с гироскопом (0x01)
    _writeRegister(MPU_REG_PWR_MGMT_1, 0x01);
    delay(10);

    // 4. Включение всех 6 осей акселерометра и гироскопа
    _writeRegister(MPU_REG_PWR_MGMT_2, 0x00);
    delay(10);

    // 5. Настройка диапазонов и фильтрации:
    // Диапазон акселерометра: ±4G (8192 LSB/g) -> 0x08
    _writeRegister(MPU_REG_ACCEL_CONFIG, 0x08);
    // Цифровой НЧ фильтр акселерометра 41 Hz -> 0x03
    _writeRegister(MPU_REG_ACCEL_CONFIG2, 0x03);
    // Цифровой фильтр DLPF 41 Hz -> 0x03
    _writeRegister(MPU_REG_CONFIG, 0x03);
    // Диапазон гироскопа ±500 deg/s -> 0x08
    _writeRegister(MPU_REG_GYRO_CONFIG, 0x08);
    // Делитель частоты сэмплирования: 1kHz / (1 + 4) = 200 Hz
    _writeRegister(MPU_REG_SMPLRT_DIV, 0x04);

    // Переводим шину I2C на рабочую частоту (400 кГц)
    Wire.setClock(freq);
    delay(10);

    // 6. Проверочное считывание ускорений
    uint8_t testBuf[6] = {0};
    if (!_readRegisters(MPU_REG_ACCEL_XOUT_H, testBuf, 6)) {
        snprintf(_statusMsg, sizeof(_statusMsg), "IMU: Ошибка чтения регистров 0x3B");
        Serial.println("[IMU] ERROR: Direct register read failed.");
        return false;
    }

    _isInitialized = true;
    int16_t ax = (int16_t)((testBuf[0] << 8) | testBuf[1]);
    int16_t ay = (int16_t)((testBuf[2] << 8) | testBuf[3]);
    int16_t az = (int16_t)((testBuf[4] << 8) | testBuf[5]);

    snprintf(_statusMsg, sizeof(_statusMsg), "MPU (0x%02X ID:0x%02X SDA:%u SCL:%u): OK", detectedAddr, detectedWho, activeSda, activeScl);
    Serial.printf("[IMU INIT SUCCESS] %s | Live: Ax=%d, Ay=%d, Az=%d (~%.2f G)\n", _statusMsg, ax, ay, az, (float)az / ACCEL_SCALE);

    return true;
}

bool ImuEngine::reinit() {
    return begin(_sdaPin, _sclPin, _freq);
}

bool ImuEngine::update() {
    if (!_isInitialized) return false;

    uint8_t rawBuf[14];
    if (!_readRegisters(MPU_REG_ACCEL_XOUT_H, rawBuf, 14)) {
        _errorCount++;
        if (_errorCount > 80) {
            _isInitialized = false;
            snprintf(_statusMsg, sizeof(_statusMsg), "IMU: Потеряна связь I2C");
        }
        return false;
    }
    _errorCount = 0;

    // Разбор сырых значений акселерометра (16-битный прямой дополнительный код)
    int16_t rawAx = (int16_t)((rawBuf[0] << 8) | rawBuf[1]);
    int16_t rawAy = (int16_t)((rawBuf[2] << 8) | rawBuf[3]);
    int16_t rawAz = (int16_t)((rawBuf[4] << 8) | rawBuf[5]);

    // Разбор гироскопа
    int16_t rawGx = (int16_t)((rawBuf[8] << 8) | rawBuf[9]);
    int16_t rawGy = (int16_t)((rawBuf[10] << 8) | rawBuf[11]);
    int16_t rawGz = (int16_t)((rawBuf[12] << 8) | rawBuf[13]);

    // Перевод в физические величины G
    float ax = (float)rawAx / ACCEL_SCALE - _offsetX;
    float ay = (float)rawAy / ACCEL_SCALE - _offsetY;
    float az = (float)rawAz / ACCEL_SCALE - _offsetZ;

    if (isnan(ax) || isinf(ax)) ax = 0.0f;
    if (isnan(ay) || isinf(ay)) ay = 0.0f;
    if (isnan(az) || isinf(az)) az = 0.0f;

    // Экспоненциальный фильтр (EMA) для устранения вибраций
    if (fabsf(_filtAx) < 0.001f && fabsf(_filtAy) < 0.001f && fabsf(_filtAz) < 0.001f) {
        _filtAx = ax;
        _filtAy = ay;
        _filtAz = az;
    } else {
        _filtAx += 0.28f * (ax - _filtAx);
        _filtAy += 0.28f * (ay - _filtAy);
        _filtAz += 0.28f * (az - _filtAz);
    }

    _data.accelX = _filtAx;
    _data.accelY = _filtAy;
    _data.accelZ = _filtAz;

    _data.gyroX = (float)rawGx / GYRO_SCALE;
    _data.gyroY = (float)rawGy / GYRO_SCALE;
    _data.gyroZ = (float)rawGz / GYRO_SCALE;

    // Продольное (разгон/торможение) и поперечное (повороты) ускорение
    _data.gLongitudinal = _data.accelX;
    _data.gLateral = _data.accelY;
    _data.gVertical = _data.accelZ;

    // Фиксация пиковых перегрузок
    if (_data.gLongitudinal > _data.gPeakAccel) {
        _data.gPeakAccel = _data.gLongitudinal;
    }
    if (_data.gLongitudinal < -_data.gPeakBrake) {
        _data.gPeakBrake = -_data.gLongitudinal;
    }

    _data.lastSampleUs = micros();
    return true;
}

bool ImuEngine::calibrateZero(uint16_t sampleCount) {
    if (!_isInitialized) {
        if (!reinit()) return false;
    }
    float sumX = 0, sumY = 0, sumZ = 0;
    uint16_t validCount = 0;

    for (uint16_t i = 0; i < sampleCount; i++) {
        uint8_t rawBuf[6];
        if (_readRegisters(MPU_REG_ACCEL_XOUT_H, rawBuf, 6)) {
            int16_t rawAx = (int16_t)((rawBuf[0] << 8) | rawBuf[1]);
            int16_t rawAy = (int16_t)((rawBuf[2] << 8) | rawBuf[3]);
            int16_t rawAz = (int16_t)((rawBuf[4] << 8) | rawBuf[5]);

            sumX += (float)rawAx / ACCEL_SCALE;
            sumY += (float)rawAy / ACCEL_SCALE;
            sumZ += (float)rawAz / ACCEL_SCALE;
            validCount++;
        }
        delay(2);
    }

    if (validCount < (sampleCount * 6 / 10) || validCount == 0) {
        Serial.printf("[IMU] Calibration failed: insufficient valid samples (%u/%u)\n", validCount, sampleCount);
        _data.isCalibrated = false;
        return false;
    }

    _offsetX = sumX / (float)validCount;
    _offsetY = sumY / (float)validCount;
    _offsetZ = (sumZ / (float)validCount) - 1.0f; // Вычитаем 1.0G земного притяжения
    _data.isCalibrated = true;
    Serial.printf("[IMU] Calibration success: OffX=%.3f, OffY=%.3f, OffZ=%.3f (%u samples)\n", _offsetX, _offsetY, _offsetZ, validCount);
    return true;
}

ImuSample ImuEngine::getLatestSample() const {
    ImuSample sample;
    sample.data = _data;
    sample.sampleUs = _data.lastSampleUs;
    return sample;
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
    // Попытка 1: Repeated Start
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0) {
        uint8_t bytesRead = Wire.requestFrom(_i2cAddr, length);
        if (bytesRead == length) {
            for (uint8_t i = 0; i < length; i++) {
                buffer[i] = Wire.read();
            }
            return true;
        }
    }

    // Попытка 2: Явный STOP
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    if (Wire.endTransmission(true) == 0) {
        uint8_t bytesRead = Wire.requestFrom(_i2cAddr, length, (bool)true);
        if (bytesRead == length) {
            for (uint8_t i = 0; i < length; i++) {
                buffer[i] = Wire.read();
            }
            return true;
        }
    }

    return false;
}
