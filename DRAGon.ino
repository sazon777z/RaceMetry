/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                 Профессиональный автомобильный измеритель динамики
 *                   (Bluetooth BLE / Web App / Dragy-style)
 * ============================================================================
 * Платформа: ESP32-S3 Super Mini
 * Сенсоры: u-blox M10Q (UBX 10-18Hz), MPU-9250 (IMU 200Hz)
 * Связь: Bluetooth Low Energy 5.0 (Nordic UART Service)
 * Индикатор: WS2812B RGB
 * Органы управления: 2 кнопки (опционально) + полное управление по BLE
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
    delay(500);
    Serial.println("\n[DRAGon] Initializing Pro BLE Telemetry System...");

    // 1. Инициализация хранилища NVS (настройки и рекорды)
    storageManager.begin();
    storageManager.loadSettings(deviceSettings);
    Serial.println("[DRAGon] Storage loaded");

    // 2. Инициализация индикатора WS2812B
    ledController.begin(PIN_WS2812);
    ledController.setMode(LedMode::GPS_SEARCH);

    // 3. Инициализация кнопок управления
    buttonManager.begin(PIN_BTN_LEFT, PIN_BTN_RIGHT);

    // 4. Инициализация инерциального датчика MPU-9250 (I2C)
    if (!imuEngine.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQUENCY)) {
        Serial.println("[DRAGon] WARNING: MPU-9250 not detected!");
    } else {
        imuEngine.setOffsets(deviceSettings.imuOffsetGx, deviceSettings.imuOffsetGy, deviceSettings.imuOffsetGz);
        Serial.println("[DRAGon] IMU MPU-9250 ready");
    }

    // 5. Инициализация GPS u-blox M10Q (Hardware UART1)
    gpsEngine.begin(Serial1, GPS_BAUDRATE_TARGET);
    Serial.println("[DRAGon] GPS M10Q configured with UBX 10-18Hz");

    // 6. Инициализация гоночного ядра телеметрии
    telemetryEngine.begin(deviceSettings);

    // 7. Инициализация BLE Сервера (Nordic UART Service)
    bleEngine.begin(BLE_DEVICE_NAME);
    bleEngine.setCommandHandler([](const String& cmd, const String& val) {
        Serial.printf("[DRAGon CMD] cmd: %s, val: %s\n", cmd.c_str(), val.c_str());

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
            bleEngine.sendDeviceInfo(deviceSettings, storageManager.getSavedRunsCount(), gpsEngine.isReadyForRace(), safeGpsData.numSats);
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
            bleEngine.sendDeviceInfo(deviceSettings, 0, gpsEngine.isReadyForRace(), safeGpsData.numSats);
        } else if (cmd == "get_info") {
            bleEngine.sendDeviceInfo(deviceSettings, storageManager.getSavedRunsCount(), gpsEngine.isReadyForRace(), safeGpsData.numSats);
            PersonalBests pb;
            storageManager.getPersonalBests(pb);
            bleEngine.sendPersonalBests(pb);
        } else if (cmd == "ping") {
            bleEngine.sendJson("{\"t\":\"pong\"}\n");
        }
    });

    // 8. Запуск высокоприоритетной задачи телеметрии на ЯДРЕ 0
    xTaskCreatePinnedToCore(
        TelemetryTask,        // Функция задачи
        "TelemetryTask",      // Имя
        8192,                 // Стек (байт)
        NULL,                 // Параметры
        2,                    // Приоритет (высокий)
        NULL,                 // Дескриптор
        0                     // Ядро 0
    );

    // 9. Запуск коммуникационной задачи BLE на ЯДРЕ 1
    xTaskCreatePinnedToCore(
        CommTask,             // Функция задачи
        "CommTask",           // Имя
        8192,                 // Стек (байт)
        NULL,                 // Параметры
        1,                    // Приоритет
        NULL,                 // Дескриптор
        1                     // Ядро 1
    );

    Serial.println("[DRAGon] BLE System started successfully!");
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

        // 1. Потоковый разбор бинарных пакетов UBX GPS (10-18 Гц)
        gpsEngine.update();

        // 2. Скоростной опрос акселерометра MPU-9250 (200 Гц)
        imuEngine.update();

        // 3. Обработка математики заезда
        telemetryEngine.process(gpsEngine.getData(), imuEngine.getData());

        // 4. Управление светодиодной индикацией WS2812B
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

        RaceState curState = telemetryEngine.getState();
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
 * ЯДРО 1: BLUETOOTH LOW ENERGY, КНОПКИ И ОБРАБОТКА ДАННЫХ
 * ============================================================================
 */
void CommTask(void* parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / BLE_TELEMETRY_RATE_HZ); // 15 Гц
    TickType_t xLastWakeTime = xTaskGetTickCount();

    bool wasConnected = false;

    for (;;) {
        // 1. Опрос кнопок
        buttonManager.update();
        ButtonEvent evLeft = buttonManager.getLeftEvent();
        ButtonEvent evRight = buttonManager.getRightEvent();

        // Обработка кнопки 1 (Left: Клик = Взведение, Длинное = Смена дисциплины)
        if (evLeft == ButtonEvent::CLICK) {
            telemetryEngine.arm();
        } else if (evLeft == ButtonEvent::LONG_PRESS) {
            uint8_t nextDisc = ((uint8_t)telemetryEngine.getDiscipline() + 1) % 6;
            telemetryEngine.setDiscipline((RaceDiscipline)nextDisc);
        }

        // Обработка кнопки 2 (Right: Клик = Сброс, Длинное = Калибровка IMU)
        if (evRight == ButtonEvent::CLICK) {
            telemetryEngine.reset();
        } else if (evRight == ButtonEvent::LONG_PRESS) {
            ledController.setMode(LedMode::CALIBRATING);
            imuEngine.calibrateZero(600);
            imuEngine.getOffsets(deviceSettings.imuOffsetGx, deviceSettings.imuOffsetGy, deviceSettings.imuOffsetGz);
            storageManager.saveSettings(deviceSettings);
        }

        // 2. Обновление состояния BLE
        bleEngine.update();
        bool isConnected = bleEngine.isConnected();

        // Если только что подключились — отправляем инфо о приборе и рекорды
        if (isConnected && !wasConnected) {
            bleEngine.sendDeviceInfo(deviceSettings, storageManager.getSavedRunsCount(), gpsEngine.isReadyForRace(), safeGpsData.numSats);
            PersonalBests pb;
            storageManager.getPersonalBests(pb);
            bleEngine.sendPersonalBests(pb);
        }
        wasConnected = isConnected;

        // 3. Получение безопасной копии данных
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

        // 4. Трансляция телеметрии по BLE
        if (isConnected) {
            // Живая телеметрия
            bleEngine.sendLiveTelemetry(
                localGps,
                localImu,
                localState,
                localDisc,
                localLiveTime,
                localDistanceM,
                localSpeedKmh,
                localSlopePct
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
 * ВЫДЕЛЕННЫЙ РЕЖИМ ПРОЗРАЧНОГО МОСТА ДЛЯ U-CENTER / U-CENTER 2
 * ============================================================================
 */
void runUcenterBridgeMode() {
    Serial.begin(115200);

    Serial1.setRxBufferSize(8192);
    uint32_t currentGpsBaud = 38400;
    Serial1.begin(currentGpsBaud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    pinMode(PIN_BTN_LEFT, INPUT_PULLUP);

    ledController.begin(PIN_WS2812);
    ledController.setMode(LedMode::RUNNING);

    uint8_t chunkBuf[512];
    bool lastBtnState = HIGH;

    for (;;) {
        int availPC = Serial.available();
        if (availPC > 0) {
            int toRead = min(availPC, (int)sizeof(chunkBuf));
            int readBytes = Serial.readBytes(chunkBuf, toRead);
            if (readBytes > 0) {
                Serial1.write(chunkBuf, readBytes);
            }
        }

        int availGPS = Serial1.available();
        if (availGPS > 0) {
            int toRead = min(availGPS, (int)sizeof(chunkBuf));
            int readBytes = Serial1.readBytes(chunkBuf, toRead);
            if (readBytes > 0) {
                Serial.write(chunkBuf, readBytes);
            }
        }

        bool btnState = (digitalRead(PIN_BTN_LEFT) == LOW);
        if (btnState && !lastBtnState) {
            if (currentGpsBaud == 38400) currentGpsBaud = 115200;
            else if (currentGpsBaud == 115200) currentGpsBaud = 460800;
            else currentGpsBaud = 38400;

            Serial1.updateBaudRate(currentGpsBaud);
            delay(100);
        }
        lastBtnState = btnState;
        ledController.update();
    }
}
