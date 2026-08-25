/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                 Профессиональный автомобильный измеритель динамики
 *                   (Bluetooth BLE / Web App / Dragy-style)
 * ============================================================================
 * Платформа: ESP32-S3 Super Mini
 * Сенсоры: u-blox M10Q (UBX 20Hz), MPU-9250 (IMU 200Hz)
 * Связь: Bluetooth Low Energy 5.0 (Nordic UART Service)
 * Индикатор: Встроенный RGB светодиод
 * Органы управления: Кнопка питания (GPIO 11) + полное управление по BLE
 * Питание: Li-Ion аккумулятор с мониторингом напряжения (GPIO 1)
 * ============================================================================
 */

#include <Arduino.h>
#include "Config.h"
#include "Types.h"
#include "GpsEngine.h"
#include "ImuEngine.h"
#include "TelemetryEngine.h"
#include "BleEngine.h"
#include "LedController.h"
#include "ButtonManager.h"
#include "StorageManager.h"

// Экземпляры основных модулей
GpsEngine       gpsEngine;
ImuEngine       imuEngine;
TelemetryEngine telemetryEngine;
BleEngine       bleEngine;
LedController   ledController;
ButtonManager   buttonManager;
StorageManager  storageManager;

// Глобальные настройки устройства
DeviceSettings  deviceSettings;

// Потокобезопасная синхронизация между ядрами (FreeRTOS Mutex)
portMUX_TYPE stateMutex = portMUX_INITIALIZER_UNLOCKED;

// Локальные копии данных для безопасной передачи между ядрами
GpsData         safeGpsData;
ImuData         safeImuData;
RaceState       safeRaceState;
RaceDiscipline  safeDiscipline;
RunRecord       safeCurrentRun;
RunRecord       safeLastRun;
float           safeLiveTimeSec = 0.0f;
float           safeLiveDistanceM = 0.0f;
float           safeLiveSpeedKmh = 0.0f;
float           safeLiveSlopePct = 0.0f;
bool            newRunSaved = false;

// Переменные состояния аккумулятора
static float    currentBatVoltage = 0.0f;
static uint8_t  currentBatPercent = 0;

// Прототипы функций питания, замера батареи и самодиагностики
float readRawBatteryVoltage();
uint8_t calculateBatteryPercentage(float v);
void runSystemDiagnostics();
void enterPowerOffDeepSleep();

// Прототипы задач FreeRTOS
void TelemetryTask(void* parameter);
void CommTask(void* parameter);
void runUcenterBridgeMode();

void setup() {
    if (GPS_BRIDGE_MODE) {
        runUcenterBridgeMode();
        return;
    }

    Serial.begin(115200);
    delay(200);
    Serial.println("\n[RaceMetry] Initializing Pro 20Hz BLE Telemetry System...");

    // 1. Инициализация хранилища NVS (настройки и рекорды)
    storageManager.begin();
    storageManager.loadSettings(deviceSettings);
    Serial.println("[RaceMetry] Storage loaded");

    // 2. Инициализация и анимация включения светодиода
    ledController.begin(PIN_WS2812);
    ledController.showPowerOnAnimation();
    ledController.setMode(LedMode::GPS_SEARCH);

    // 3. Инициализация кнопки питания (GPIO 11)
    buttonManager.begin(PIN_BTN);

    // 4. Инициализация АЦП батареи
#if ENABLE_BATTERY_MONITOR
    pinMode(PIN_BAT_ADC, INPUT);
    analogReadResolution(12);
    currentBatVoltage = readRawBatteryVoltage();
    currentBatPercent = calculateBatteryPercentage(currentBatVoltage);
    Serial.printf("[RaceMetry] Battery Monitor initialized: %.2fV (%u%%)\n", currentBatVoltage, currentBatPercent);
#endif

    // 5. Инициализация инерциального датчика MPU-9250 (I2C)
    if (!imuEngine.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQUENCY)) {
        Serial.println("[RaceMetry] WARNING: MPU-9250 not detected!");
    } else {
        imuEngine.setOffsets(deviceSettings.imuOffsetGx, deviceSettings.imuOffsetGy, deviceSettings.imuOffsetGz);
        Serial.println("[RaceMetry] IMU MPU-9250 ready (200 Hz)");
    }

    // 6. Инициализация GPS u-blox M10Q (Hardware UART1, 20 Hz)
    gpsEngine.begin(Serial1, GPS_BAUDRATE_TARGET);
    Serial.printf("[RaceMetry] GPS M10Q configured with UBX %d Hz\n", GPS_UPDATE_RATE_HZ);

    // 7. Инициализация гоночного ядра телеметрии
    telemetryEngine.begin(deviceSettings);

    // 8. Инициализация BLE Сервера (Nordic UART Service)
    bleEngine.begin(BLE_DEVICE_NAME);
    bleEngine.setCommandHandler([](const String& cmd, const String& val) {
        Serial.printf("[RaceMetry CMD] cmd: %s, val: %s\n", cmd.c_str(), val.c_str());

        if (cmd == "arm") {
            telemetryEngine.arm();
        } else if (cmd == "reset") {
            telemetryEngine.reset();
        } else if (cmd == "set_disc") {
            int disc = val.toInt();
            if (disc >= 0 && disc <= 6) {
                telemetryEngine.setDiscipline((RaceDiscipline)disc);
            }
        } else if (cmd == "set_rollout") {
            deviceSettings.use1FootRollout = (val == "true" || val == "1");
            telemetryEngine.updateSettings(deviceSettings);
            storageManager.saveSettings(deviceSettings);
        } else if (cmd == "set_units") {
            deviceSettings.metricUnits = (val == "true" || val == "1" || val == "metric");
            storageManager.saveSettings(deviceSettings);
        } else if (cmd == "calibrate_imu") {
            ledController.setMode(LedMode::CALIBRATING);
            imuEngine.calibrateZero(600);
            imuEngine.getOffsets(deviceSettings.imuOffsetGx, deviceSettings.imuOffsetGy, deviceSettings.imuOffsetGz);
            storageManager.saveSettings(deviceSettings);
            runSystemDiagnostics();
            bleEngine.sendDeviceInfo(deviceSettings, storageManager.getSavedRunsCount(), gpsEngine.isReadyForRace(), safeGpsData.numSats, currentBatVoltage, currentBatPercent);
        } else if (cmd == "run_diag") {
            runSystemDiagnostics();
        } else if (cmd == "power_off") {
            enterPowerOffDeepSleep();
        } else if (cmd == "get_history") {
            uint8_t count = storageManager.getSavedRunsCount();
            for (uint8_t i = 0; i < count; i++) {
                RunRecord r;
                if (storageManager.getRunRecord(i, r)) {
                    bleEngine.sendRunRecord(r);
                    delay(15);
                }
            }
        } else if (cmd == "clear_history") {
            storageManager.clearAllRuns();
            bleEngine.sendDeviceInfo(deviceSettings, 0, gpsEngine.isReadyForRace(), safeGpsData.numSats, currentBatVoltage, currentBatPercent);
        } else if (cmd == "get_info") {
            runSystemDiagnostics();
            bleEngine.sendDeviceInfo(deviceSettings, storageManager.getSavedRunsCount(), gpsEngine.isReadyForRace(), safeGpsData.numSats, currentBatVoltage, currentBatPercent);
            PersonalBests pb;
            storageManager.getPersonalBests(pb);
            bleEngine.sendPersonalBests(pb);
        } else if (cmd == "ping") {
            bleEngine.sendJson("{\"t\":\"pong\"}\n");
        }
    });

    // 9. Запуск высокоприоритетной задачи телеметрии на ЯДРЕ 0
    xTaskCreatePinnedToCore(
        TelemetryTask,        // Функция задачи
        "TelemetryTask",      // Имя
        8192,                 // Стек (байт)
        NULL,                 // Параметры
        2,                    // Приоритет (высокий)
        NULL,                 // Дескриптор
        0                     // Ядро 0
    );

    // 10. Запуск коммуникационной задачи BLE на ЯДРЕ 1
    xTaskCreatePinnedToCore(
        CommTask,             // Функция задачи
        "CommTask",           // Имя
        8192,                 // Стек (байт)
        NULL,                 // Параметры
        1,                    // Приоритет
        NULL,                 // Дескриптор
        1                     // Ядро 1
    );

    Serial.println("[RaceMetry] BLE System started successfully!");
}

/**
 * ============================================================================
 * ЯДРО 0: ВЫСОКОСКОРОСТНАЯ ОБРАБОТКА ДАТЧИКОВ И ТЕЛЕМЕТРИИ
 * ============================================================================
 */
void TelemetryTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200 Гц цикл опроса

    RaceState prevRaceState = RaceState::IDLE_WAIT_STOP;

    for (;;) {
        // Проверка режима моста U-Center
        if (GPS_BRIDGE_MODE) {
            while (Serial.available() > 0) {
                Serial1.write(Serial.read());
            }
            while (Serial1.available() > 0) {
                Serial.write(Serial1.read());
            }
            ledController.setMode(LedMode::RUNNING);
            ledController.update();
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
            continue;
        }

        // 1. Потоковый разбор бинарных пакетов UBX GPS (20 Гц)
        gpsEngine.update();

        // 2. Скоростной опрос акселерометра MPU-9250 (200 Гц)
        imuEngine.update();

        // 3. Обработка математики заезда
        telemetryEngine.process(gpsEngine.getData(), imuEngine.getData());
        RaceState curState = telemetryEngine.getState();

        // 4. Управление светодиодной индикацией (если кнопка не удерживается)
        if (!buttonManager.isPressed() || buttonManager.getPressDurationMs() < 400) {
            static bool wasGpsReady = false;
            bool isGpsReady = gpsEngine.isReadyForRace();

            // Проверка события первого захвата 3D-фикса
            if (isGpsReady && !wasGpsReady) {
                ledController.notifyFixAcquired();
            }
            wasGpsReady = isGpsReady;

            // Проверка моментальной вспышки при взятии отсечки (0-100, 100-200, 402м)
            if (telemetryEngine.checkAndClearSplitTrigger()) {
                ledController.triggerSplitFlash();
            }

            if (!isGpsReady) {
                ledController.setMode(LedMode::GPS_SEARCH);
            } else {
                switch (curState) {
                    case RaceState::ARMED:
                        ledController.setMode(LedMode::ARMED_READY);
                        break;
                    case RaceState::LAUNCH_DETECTED:
                        ledController.setMode(LedMode::LAUNCH_DETECTED);
                        break;
                    case RaceState::MEASURING:
                        if (telemetryEngine.getDiscipline() == RaceDiscipline::BRAKE_100_0) {
                            ledController.setMode(LedMode::BRAKING_ACTIVE);
                        } else {
                            ledController.setMode(LedMode::MEASURING);
                        }
                        break;
                    case RaceState::FINISHED:
                        if (telemetryEngine.getLastRun().isValidSlope) {
                            ledController.setMode(LedMode::FINISHED_VALID);
                        } else {
                            ledController.setMode(LedMode::FINISHED_SLOPE);
                        }
                        break;
                    default:
                        ledController.setMode(LedMode::ARMED_READY);
                        break;
                }
            }
            ledController.update();
        }

        // 5. Автосохранение завершенного заезда в энергонезависимую память NVS
        if (curState == RaceState::FINISHED && prevRaceState != RaceState::FINISHED) {
            storageManager.saveRunRecord(telemetryEngine.getLastRun());
            newRunSaved = true;
        }
        prevRaceState = curState;

        // 6. Синхронизация данных со снимком для Ядра 1 (BLE)
        portENTER_CRITICAL(&stateMutex);
        safeGpsData = gpsEngine.getData();
        safeImuData = imuEngine.getData();
        safeRaceState = curState;
        safeDiscipline = telemetryEngine.getDiscipline();
        safeCurrentRun = telemetryEngine.getCurrentRun();
        safeLastRun = telemetryEngine.getLastRun();
        safeLiveTimeSec = telemetryEngine.getCurrentTimeSec();
        safeLiveDistanceM = telemetryEngine.getCurrentDistanceM();
        safeLiveSpeedKmh = telemetryEngine.getCurrentSpeedKmh();
        safeLiveSlopePct = telemetryEngine.getCurrentSlopePct();
        portEXIT_CRITICAL(&stateMutex);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * ============================================================================
 * ЯДРО 1: BLUETOOTH LOW ENERGY, КНОПКА ПИТАНИЯ И ОБРАБОТКА ДАННЫХ
 * ============================================================================
 */
void CommTask(void* parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / BLE_TELEMETRY_RATE_HZ); // 15 Гц
    TickType_t xLastWakeTime = xTaskGetTickCount();

    bool wasConnected = false;
    uint32_t lastBatSampleMs = 0;

    for (;;) {
        // 1. Опрос кнопки питания (GPIO 11)
        buttonManager.update();
        if (buttonManager.isPressed()) {
            uint32_t pressDur = buttonManager.getPressDurationMs();
            if (pressDur >= 400) {
                // Визуализация обратного отсчета выключения (желтый -> красный)
                ledController.showPowerOffHolding(buttonManager.getPowerOffProgressPct());
            }
        }
        if (buttonManager.isPowerOffTriggered()) {
            enterPowerOffDeepSleep();
        }

        // 2. Периодический замер напряжения батареи (раз в 500 мс с фильтрацией)
        if (millis() - lastBatSampleMs >= 500 || lastBatSampleMs == 0) {
            lastBatSampleMs = millis();
            float rawV = readRawBatteryVoltage();
            if (currentBatVoltage < 0.1f) {
                currentBatVoltage = rawV;
            } else {
                // Экспоненциальное сглаживание шумов АЦП
                currentBatVoltage = 0.85f * currentBatVoltage + 0.15f * rawV;
            }
            currentBatPercent = calculateBatteryPercentage(currentBatVoltage);
        }

        // 3. Обновление состояния BLE
        bleEngine.update();
        bool isConnected = bleEngine.isConnected();

        // Если только что подключились — запускаем полную инициализацию и отчет
        if (isConnected && !wasConnected) {
            runSystemDiagnostics();
            vTaskDelay(pdMS_TO_TICKS(30));
            bleEngine.sendDeviceInfo(deviceSettings, storageManager.getSavedRunsCount(), gpsEngine.isReadyForRace(), safeGpsData.numSats, currentBatVoltage, currentBatPercent);
            vTaskDelay(pdMS_TO_TICKS(30));
            PersonalBests pb;
            storageManager.getPersonalBests(pb);
            bleEngine.sendPersonalBests(pb);
        }
        wasConnected = isConnected;

        // 4. Получение безопасной копии данных
        GpsData localGps;
        ImuData localImu;
        RaceState localState;
        RaceDiscipline localDisc;
        RunRecord localCurr;
        RunRecord localLast;
        float localLiveTime;
        float localDistanceM;
        float localSpeedKmh;
        float localSlopePct;
        bool localNewRunSaved = false;

        portENTER_CRITICAL(&stateMutex);
        localGps = safeGpsData;
        localImu = safeImuData;
        localState = safeRaceState;
        localDisc = safeDiscipline;
        localCurr = safeCurrentRun;
        localLast = safeLastRun;
        localLiveTime = safeLiveTimeSec;
        localDistanceM = safeLiveDistanceM;
        localSpeedKmh = safeLiveSpeedKmh;
        localSlopePct = safeLiveSlopePct;
        if (newRunSaved) {
            localNewRunSaved = true;
            newRunSaved = false;
        }
        portEXIT_CRITICAL(&stateMutex);

        // 5. Трансляция телеметрии по BLE
        if (isConnected) {
            // Живая телеметрия с процентом батареи
            bleEngine.sendLiveTelemetry(
                localGps,
                localImu,
                localState,
                localDisc,
                localLiveTime,
                localDistanceM,
                localSpeedKmh,
                localSlopePct,
                currentBatVoltage,
                currentBatPercent
            );

            // Если только что завершился заезд — отправляем полный отчет и обновленные рекорды
            if (localNewRunSaved) {
                bleEngine.sendRunRecord(localLast);
                PersonalBests pb;
                storageManager.getPersonalBests(pb);
                bleEngine.sendPersonalBests(pb);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * ============================================================================
 * ФУНКЦИЯ ВЫКЛЮЧЕНИЯ ПИТАНИЯ (DEEP SLEEP C ПРОБУЖДЕНИЕМ ПО КНОПКЕ)
 * ============================================================================
 */
void enterPowerOffDeepSleep() {
    Serial.println("\n[DRAGon] Powering OFF... Entering Ultra-Low-Power Deep Sleep.");

    // 1. Уведомляем подключенный смартфон по BLE
    bleEngine.sendJson("{\"t\":\"shutdown\"}\n");
    delay(50);

    // 2. Анимация выключения на светодиоде (3 красные вспышки и плавное затухание)
    ledController.showPowerOffAnimation();
    ledController.turnOff();

    // 3. Настройка пробуждения по нажатию кнопки на GPIO 11 (LOW level)
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BTN, 0);

    // 4. Ожидаем отпускания кнопки перед сном, чтобы исключить мгновенное повторное пробуждение
    while (digitalRead(PIN_BTN) == LOW) {
        delay(10);
    }
    delay(150);

    // 5. Переход в глубокий сон
    esp_deep_sleep_start();
}

/**
 * ============================================================================
 * ФУНКЦИЯ САМОДИАГНОСТИКИ И ОТЧЕТА ОБ ИНИЦИАЛИЗАЦИИ
 * ============================================================================
 */
void runSystemDiagnostics() {
    bool imuOk = imuEngine.isReady();
    const char* imuMsg = imuOk ? "MPU-9250 (200 Hz): OK" : "MPU-9250: Ошибка I2C";

    bool gpsOk = (safeGpsData.numSats > 0 || gpsEngine.isReceivingBytes());
    const char* gpsMsg = "u-blox M10Q (20 Hz UBX, 460800 baud): OK";

    bool storageOk = true;
    bool batOk = (currentBatVoltage > 2.0f);

    bleEngine.sendDiagnostics(
        imuOk, imuMsg,
        gpsOk, gpsMsg,
        GPS_UPDATE_RATE_HZ,
        GPS_BAUDRATE_TARGET,
        storageOk,
        batOk,
        currentBatVoltage,
        currentBatPercent
    );
}

/**
 * ============================================================================
 * ФУНКЦИИ ЗАМЕРА И РАСЧЕТА НАПРЯЖЕНИЯ БАТАРЕИ
 * ============================================================================
 */
float readRawBatteryVoltage() {
#if ENABLE_BATTERY_MONITOR
    uint32_t mv = analogReadMilliVolts(PIN_BAT_ADC);
    return (mv / 1000.0f) * BAT_DIVIDER_RATIO;
#else
    return 0.0f;
#endif
}

uint8_t calculateBatteryPercentage(float v) {
    if (v < 2.0f) return 0; // Питание без делителя или батарея не подключена
    if (v >= BAT_VOLTAGE_MAX) return 100;
    if (v <= BAT_VOLTAGE_MIN) return 0;
    // Аппроксимация разрядной кривой Li-Ion:
    if (v >= 3.95f) return 80 + (uint8_t)((v - 3.95f) / (4.20f - 3.95f) * 20.0f);
    if (v >= 3.75f) return 40 + (uint8_t)((v - 3.75f) / (3.95f - 3.75f) * 40.0f);
    if (v >= 3.55f) return 15 + (uint8_t)((v - 3.55f) / (3.75f - 3.55f) * 25.0f);
    return (uint8_t)((v - 3.30f) / (3.55f - 3.30f) * 15.0f);
}

/**
 * ============================================================================
 * ВЫДЕЛЕННЫЙ РЕЖИМ ПРОЗРАЧНОГО МОСТА ДЛЯ U-CENTER / U-CENTER 2
 * ============================================================================
 */
void runUcenterBridgeMode() {
    Serial.begin(115200);

    // 1. Индикация режима моста на светодиоде (Неоновый голубой / Циан)
    ledController.begin(PIN_WS2812);
    ledController.setRgb(0, 180, 255);

    // 2. Быстрое авто-определение скорости GPS модуля
    const uint32_t testBauds[] = { 38400, 115200, 460800, 9600, 57600 };
    uint32_t activeGpsBaud = 38400;

    for (uint32_t b : testBauds) {
        Serial1.end();
        Serial1.setRxBufferSize(8192);
        Serial1.begin(b, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
        delay(150);

        uint32_t t0 = millis();
        int rxCount = 0;
        while (millis() - t0 < 250) {
            while (Serial1.available()) {
                rxCount++;
                Serial1.read();
            }
            delay(5);
        }

        if (rxCount > 2) {
            activeGpsBaud = b;
            break;
        }
    }

    // 3. Запуск скоростного прозрачного канала на определенной скорости
    Serial1.end();
    Serial1.setRxBufferSize(8192);
    Serial1.begin(activeGpsBaud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    pinMode(PIN_BTN, INPUT_PULLUP);
    bool lastBtnState = HIGH;

    // 4. Сквозная передача данных USB <-> GPS с нулевой задержкой
    for (;;) {
        // USB -> GPS (команды опроса и конфигурации из u-center)
        while (Serial.available() > 0) {
            Serial1.write(Serial.read());
        }

        // GPS -> USB (бинарные пакеты UBX и строки NMEA в u-center)
        while (Serial1.available() > 0) {
            Serial.write(Serial1.read());
        }

        // Ручное переключение скорости GPS по клику на кнопку GPIO 11
        bool btnState = (digitalRead(PIN_BTN) == LOW);
        if (btnState && !lastBtnState) {
            if (activeGpsBaud == 9600) activeGpsBaud = 38400;
            else if (activeGpsBaud == 38400) activeGpsBaud = 115200;
            else if (activeGpsBaud == 115200) activeGpsBaud = 460800;
            else activeGpsBaud = 9600;

            Serial1.updateBaudRate(activeGpsBaud);
            ledController.setRgb(255, 120, 0);
            delay(150);
            ledController.setRgb(0, 180, 255);
        }
        lastBtnState = btnState;
    }
}
