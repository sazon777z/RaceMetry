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
        static uint32_t finishSummaryShowTime = 0;
        static bool inAutoSummary = false;

        if (newRunSaved) {
            newRunSaved = false;
            inAutoSummary = true;
            finishSummaryShowTime = millis();
            displayEngine.setScreen(AppScreen::RUN_RESULTS);
        }

        // Автоматический возврат с экрана результатов через 7 секунд
        if (inAutoSummary && (millis() - finishSummaryShowTime >= AUTO_SUMMARY_DURATION_MS)) {
            inAutoSummary = false;
            if (displayEngine.getScreen() == AppScreen::RUN_RESULTS) {
                displayEngine.setScreen(AppScreen::DRAG_RACE);
            }
        }

        // Если пользователь сам нажал кнопку во время авто-показа — сбрасываем флаг
        if (evLeft != ButtonEvent::NONE || evRight != ButtonEvent::NONE) {
            inAutoSummary = false;
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

        PersonalBests personalBests;
        storageManager.getPersonalBests(personalBests);

        // 4. Отрисовка графического интерфейса
        displayEngine.render(
            displayEngine.getScreen(),
            localGps,
            localImu,
            localState,
            localDisc,
            localCurr,
            localLast,
            personalBests,
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

    // Буфер 8 КБ для предотвращения любых потерь и переполнений
    Serial1.setRxBufferSize(8192);
    
    // Начальная скорость по умолчанию для u-blox M10 (38400 бод)
    uint32_t currentGpsBaud = 38400;
    Serial1.begin(currentGpsBaud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    // Кнопка для ручного переключения скорости прямо с прибора
    pinMode(PIN_BTN_LEFT, INPUT_PULLUP);

    // Индикация и экран
    ledController.begin(PIN_WS2812);
    ledController.setMode(LedMode::RUNNING);

    displayEngine.begin();
    displayEngine.setScreen(AppScreen::UCENTER_BRIDGE);

    uint8_t chunkBuf[512];
    uint32_t lastUiUpdate = 0;
    uint32_t totalRx = 0, totalTx = 0;
    bool lastBtnState = HIGH;

    for (;;) {
        // 1. Сверхбыстрый блочный проброс PC -> GPS
        int availPC = Serial.available();
        if (availPC > 0) {
            int toRead = min(availPC, (int)sizeof(chunkBuf));
            int readBytes = Serial.readBytes(chunkBuf, toRead);
            if (readBytes > 0) {
                Serial1.write(chunkBuf, readBytes);
                totalTx += readBytes;
            }
        }

        // 2. Сверхбыстрый блочный проброс GPS -> PC
        int availGPS = Serial1.available();
        if (availGPS > 0) {
            int toRead = min(availGPS, (int)sizeof(chunkBuf));
            int readBytes = Serial1.readBytes(chunkBuf, toRead);
            if (readBytes > 0) {
                Serial.write(chunkBuf, readBytes);
                totalRx += readBytes;
            }
        }

        // 3. Обработка Кнопки 10 (переключение скорости 38400 -> 115200 -> 460800)
        bool btnState = (digitalRead(PIN_BTN_LEFT) == LOW);
        if (btnState && !lastBtnState) {
            if (currentGpsBaud == 38400) currentGpsBaud = 115200;
            else if (currentGpsBaud == 115200) currentGpsBaud = 460800;
            else currentGpsBaud = 38400;

            Serial1.updateBaudRate(currentGpsBaud);
            delay(100);
        }
        lastBtnState = btnState;

        // 4. Отрисовка статуса на дисплее (раз в 150 мс, не замедляя UART)
        if (millis() - lastUiUpdate >= 150) {
            lastUiUpdate = millis();
            _renderBridgeUi(currentGpsBaud, totalRx, totalTx);
            ledController.update();
        }
    }
}

static void _renderBridgeUi(uint32_t baud, uint32_t rx, uint32_t tx) {
    displayEngine.renderBridge(baud, rx, tx);
}
