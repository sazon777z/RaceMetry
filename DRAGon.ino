/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                 Профессиональный автомобильный измеритель динамики
 *                     (Аналог Dragy, RaceLogic, RaceBox)
 * ============================================================================
 * Платформа: ESP32-S3 Super Mini
 * Сенсоры: u-blox M10Q (UBX 10-18Hz), MPU-9250 (IMU 200Hz)
 * Экран: 1.47" IPS ST7789 SPI (320x172)
 * Индикатор: WS2812B RGB
 * Органы управления: 2 кнопки
 * ============================================================================
 */

#include <Arduino.h>
#include "Config.h"
#include "Types.h"
#include "GpsEngine.h"
#include "ImuEngine.h"
#include "TelemetryEngine.h"
#include "DisplayEngine.h"
#include "LedController.h"
#include "ButtonManager.h"
#include "StorageManager.h"

// Экземпляры основных модулей
GpsEngine       gpsEngine;
ImuEngine       imuEngine;
TelemetryEngine telemetryEngine;
DisplayEngine   displayEngine;
LedController   ledController;
ButtonManager   buttonManager;
StorageManager  storageManager;

// Глобальные настройки устройства
DeviceSettings  deviceSettings;

// Потокобезопасная синхронизация между ядрами (FreeRTOS Mutex)
portMUX_TYPE stateMutex = portMUX_INITIALIZER_UNLOCKED;

// Локальные копии данных для безопасной передачи в поток отрисовки
GpsData         safeGpsData;
ImuData         safeImuData;
RaceState       safeRaceState;
RaceDiscipline  safeDiscipline;
RunRecord       safeCurrentRun;
RunRecord       safeLastRun;
float           safeLiveTimeSec = 0.0f;
bool            newRunSaved = false;

// Прототипы задач FreeRTOS
void TelemetryTask(void* parameter);
void UiTask(void* parameter);
void runUcenterBridgeMode();

void setup() {
    if (GPS_BRIDGE_MODE) {
        runUcenterBridgeMode();
        return;
    }

    Serial.begin(115200);
    delay(500);
    Serial.println("\n[DRAGon] Initializing Pro Telemetry System...");

    // 1. Инициализация хранилища NVS (настройки и рекорды)
    storageManager.begin();
    storageManager.loadSettings(deviceSettings);
    Serial.println("[DRAGon] Storage loaded");

    // 2. Инициализация индикатора WS2812B
    ledController.begin(PIN_WS2812);
    ledController.setMode(LedMode::GPS_SEARCH);

    // 3. Инициализация кнопок управления
    buttonManager.begin(PIN_BTN_LEFT, PIN_BTN_RIGHT);

    // 4. Инициализация дисплея IPS ST7789
    displayEngine.begin();
    displayEngine.setBrightness(deviceSettings.displayBrightness);
    displayEngine.setScreen((AppScreen)deviceSettings.defaultScreen);
    Serial.println("[DRAGon] Display initialized");

    // 5. Инициализация инерциального датчика MPU-9250 (I2C)
    if (!imuEngine.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQUENCY)) {
        Serial.println("[DRAGon] WARNING: MPU-9250 not detected!");
    } else {
        imuEngine.setOffsets(deviceSettings.imuOffsetGx, deviceSettings.imuOffsetGy, deviceSettings.imuOffsetGz);
        Serial.println("[DRAGon] IMU MPU-9250 ready");
    }

    // 6. Инициализация GPS u-blox M10Q (Hardware UART1)
    gpsEngine.begin(Serial1, GPS_BAUDRATE_TARGET);
    Serial.println("[DRAGon] GPS M10Q configured with UBX 10-18Hz");

    // 7. Инициализация гоночного ядра телеметрии
    telemetryEngine.begin(deviceSettings);

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

    // 9. Запуск задачи графического интерфейса и кнопок на ЯДРЕ 1
    xTaskCreatePinnedToCore(
        UiTask,               // Функция задачи
        "UiTask",             // Имя
        8192,                 // Стек (байт)
        NULL,                 // Параметры
        1,                    // Приоритет
        NULL,                 // Дескриптор
        1                     // Ядро 1
    );

    Serial.println("[DRAGon] System started successfully!");
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
        if (displayEngine.getScreen() == AppScreen::UCENTER_BRIDGE || GPS_BRIDGE_MODE) {
            // Прозрачный двунаправленный аппаратный проброс: USB CDC <-> UART1 GPS
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

        // 1. Потоковый разбор бинарных пакетов UBX GPS (Стандартный режим)
        bool newGpsPvt = gpsEngine.update();

        // 2. Скоростной опрос акселерометра MPU-9250
        imuEngine.update();

        // 3. Обработка телеметрии
        telemetryEngine.process(gpsEngine.getData(), imuEngine.getData());

        // 4. Управление светодиодной индикацией WS2812B
        RaceState curState = telemetryEngine.getState();
        if (!gpsEngine.isReadyForRace()) {
            ledController.setMode(LedMode::GPS_SEARCH);
        } else {
            switch (curState) {
                case RaceState::ARMED:
                    ledController.setMode(LedMode::ARMED_READY);
                    break;
                case RaceState::LAUNCH_DETECTED:
                case RaceState::MEASURING:
                    ledController.setMode(LedMode::RUNNING);
                    break;
                case RaceState::FINISHED:
                    if (telemetryEngine.getLastRun().isValidSlope) {
                        ledController.setMode(LedMode::FINISHED_VALID);
                    } else {
                        ledController.setMode(LedMode::FINISHED_SLOPE);
                    }
                    break;
                default:
                    ledController.setMode(LedMode::GPS_SEARCH);
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

        // 6. Синхронизация данных со снимком для Ядра 1 (UI)
        portENTER_CRITICAL(&stateMutex);
        safeGpsData = gpsEngine.getData();
        safeImuData = imuEngine.getData();
        safeRaceState = curState;
        safeDiscipline = telemetryEngine.getDiscipline();
        safeCurrentRun = telemetryEngine.getCurrentRun();
        safeLastRun = telemetryEngine.getLastRun();
        safeLiveTimeSec = telemetryEngine.getCurrentTimeSec();
        portEXIT_CRITICAL(&stateMutex);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * ============================================================================
 * ЯДРО 1: ГРАФИЧЕСКИЙ ИНТЕРФЕЙС, КНОПКИ И ВЗАИМОДЕЙСТВИЕ
 * ============================================================================
 */
void UiTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // ~40 FPS отрисовка экрана

    for (;;) {
        // 1. Опрос кнопок
        buttonManager.update();
        ButtonEvent evLeft = buttonManager.getLeftEvent();
        ButtonEvent evRight = buttonManager.getRightEvent();

        // 2. Обработка событий кнопок
        // Кнопка 1 (Left)
        if (evLeft == ButtonEvent::CLICK) {
            displayEngine.nextScreen();
        } else if (evLeft == ButtonEvent::LONG_PRESS) {
            // Переключение режима/дисциплины
            uint8_t nextDisc = ((uint8_t)telemetryEngine.getDiscipline() + 1) % 6;
            telemetryEngine.setDiscipline((RaceDiscipline)nextDisc);
        }

        // Кнопка 2 (Right)
        if (evRight == ButtonEvent::CLICK) {
            if (displayEngine.getScreen() == AppScreen::RUN_RESULTS) {
                // Возврат на драг-экран
                displayEngine.setScreen(AppScreen::DRAG_RACE);
            } else if (displayEngine.getScreen() == AppScreen::SETTINGS) {
                // Переключение 1-Foot Rollout
                deviceSettings.use1FootRollout = !deviceSettings.use1FootRollout;
                telemetryEngine.updateSettings(deviceSettings);
                storageManager.saveSettings(deviceSettings);
            } else {
                // Принудительное взведение/сброс
                telemetryEngine.arm();
            }
        } else if (evRight == ButtonEvent::LONG_PRESS) {
            if (displayEngine.getScreen() == AppScreen::SETTINGS) {
                // Калибровка нуля акселерометра MPU-9250
                ledController.setMode(LedMode::CALIBRATING);
                imuEngine.calibrateZero(600);
                imuEngine.getOffsets(deviceSettings.imuOffsetGx, deviceSettings.imuOffsetGy, deviceSettings.imuOffsetGz);
                storageManager.saveSettings(deviceSettings);
            } else {
                telemetryEngine.reset();
            }
        }

        // Если заезд только что завершился — автоматически переключаем на экран результатов!
        if (newRunSaved) {
            newRunSaved = false;
            displayEngine.setScreen(AppScreen::RUN_RESULTS);
        }

        // 3. Получение безопасной копии данных
        GpsData localGps;
        ImuData localImu;
        RaceState localState;
        RaceDiscipline localDisc;
        RunRecord localCurr;
        RunRecord localLast;
        float localLiveTime;

        portENTER_CRITICAL(&stateMutex);
        localGps = safeGpsData;
        localImu = safeImuData;
        localState = safeRaceState;
        localDisc = safeDiscipline;
        localCurr = safeCurrentRun;
        localLast = safeLastRun;
        localLiveTime = safeLiveTimeSec;
        portEXIT_CRITICAL(&stateMutex);

        // 4. Отрисовка графического интерфейса
        displayEngine.render(
            displayEngine.getScreen(),
            localGps,
            localImu,
            localState,
            localDisc,
            localCurr,
            localLast,
            deviceSettings,
            localLiveTime
        );

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void loop() {
    // Вся работа выполняется задачами FreeRTOS на Core 0 и Core 1
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * ============================================================================
 * ВЫДЕЛЕННЫЙ РЕЖИМ ПРОЗРАЧНОГО МОСТА ДЛЯ U-CENTER / U-CENTER 2
 * ============================================================================
 */
void runUcenterBridgeMode() {
    Serial.begin(115200);

    // Увеличиваем аппаратный буфер UART1 до 4096 байт для исключения переполнения
    Serial1.setRxBufferSize(4096);
    Serial1.begin(38400, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    // Индикация и экран
    ledController.begin(PIN_WS2812);
    ledController.setMode(LedMode::RUNNING);

    displayEngine.begin();
    displayEngine.setScreen(AppScreen::UCENTER_BRIDGE);

    uint32_t lastHostBaud = 0;
    uint32_t lastUiUpdate = 0;

    for (;;) {
        // Динамическая адаптация скорости UART под запрос u-center с ПК
        uint32_t hostBaud = Serial.baudRate();
        if (hostBaud >= 9600 && hostBaud <= 921600 && hostBaud != lastHostBaud) {
            Serial1.updateBaudRate(hostBaud);
            lastHostBaud = hostBaud;
        }

        // Прямой проброс байт PC -> GPS
        while (Serial.available() > 0) {
            Serial1.write(Serial.read());
        }

        // Прямой проброс байт GPS -> PC
        while (Serial1.available() > 0) {
            Serial.write(Serial1.read());
        }

        // Отрисовка статуса на дисплее
        if (millis() - lastUiUpdate >= 120) {
            lastUiUpdate = millis();
            displayEngine.render(AppScreen::UCENTER_BRIDGE, safeGpsData, safeImuData, RaceState::IDLE_WAIT_STOP, RaceDiscipline::ALL_IN_ONE_DRAG, safeCurrentRun, safeLastRun, deviceSettings, 0.0f);
            ledController.update();
        }
    }
}
