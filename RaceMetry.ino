/**
 * ============================================================================
 *                          RACEMETRY PRO TELEMETRY METER
 *                 Профессиональный автомобильный измеритель динамики
 *                   (Bluetooth BLE / Web App / Dragy-style)
 * ============================================================================
 * Платформа: ESP32-S3 Super Mini
 * Сенсоры: u-blox M10Q (UBX 20Hz), MPU-9250 (IMU 200Hz)
 * Связь: Bluetooth Low Energy 5.0 (Nordic UART Service)
 * Индикатор: Адресный RGB светодиод WS2812B (GPIO 10)
 * Питание: Модуль MH-CD42 (5V Boost + Li-Ion Charge) + Делитель АКБ на GPIO 1
 * Органы управления: Кнопка на корпусе (KEY на MH-CD42 / GPIO 11) + BLE Web App
 * ============================================================================
 */

#include <Arduino.h>
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
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

// Команды от Core 1 (CommTask) к Core 0 (TelemetryTask)
enum class Core0CmdType : uint8_t {
    ARM,
    RESET,
    SET_DISCIPLINE,
    UPDATE_SETTINGS,
    CALIBRATE_IMU,
    POWER_OFF
};

struct Core0Command {
    Core0CmdType type;
    union {
        RaceDiscipline discipline;
        DeviceSettings settings;
    } payload;
};

// Очереди межъядерного взаимодействия FreeRTOS
static QueueHandle_t telemetryCmdQueue = nullptr;
static QueueHandle_t splitQueue = nullptr;
static QueueHandle_t completedRunQueue = nullptr;

// Потокобезопасная синхронизация моментальных снимков (TelemetryTask -> CommTask)
static portMUX_TYPE stateMutex = portMUX_INITIALIZER_UNLOCKED;

GpsData         safeGpsData;
ImuData         safeImuData;
RaceState       safeRaceState = RaceState::IDLE_WAIT_STOP;
RaceDiscipline  safeDiscipline = RaceDiscipline::SPEED_0_100;
float           safeLiveTimeSec = 0.0f;
float           safeLiveDistanceM = 0.0f;
float           safeLiveSpeedKmh = 0.0f;
float           safeLiveSlopePct = 0.0f;
bool            safeGpsReady = false;

// Переменные состояния аккумулятора
static float    currentBatVoltage = 0.0f;
static uint8_t  currentBatPercent = 0;

// Прототипы функций
float readRawBatteryVoltage();
uint8_t calculateBatteryPercentage(float v);
void runSystemDiagnostics();
void enterPowerOffDeepSleep();

// Прототипы задач FreeRTOS
void TelemetryTask(void* parameter);
void CommTask(void* parameter);
void runUcenterBridgeMode();

void setup() {
    // 0. Снятие аппаратной фиксации GPIO после сна
    if (PIN_BTN != 255) {
        gpio_hold_dis((gpio_num_t)PIN_BTN);
    }
    gpio_hold_dis((gpio_num_t)PIN_GPS_TX);

    // Проверка причины старта: если пробуждение по кнопке из сна
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (PIN_BTN != 255 && (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 || wakeup_reason == ESP_SLEEP_WAKEUP_EXT1)) {
        pinMode(PIN_BTN, INPUT_PULLUP);
        // Защита от дребезга и ложного включения: требуем удержание кнопки 350 мс для включения
        uint32_t pressStart = millis();
        bool validPress = true;
        while (millis() - pressStart < 350) {
            if (digitalRead(PIN_BTN) != LOW) {
                validPress = false;
                break;
            }
            delay(10);
        }

        if (!validPress) {
            // Кнопка не удерживалась -> немедленно возвращаемся в глубокий сон
            enterPowerOffDeepSleep();
            return;
        }
    }

    if (GPS_BRIDGE_MODE) {
        runUcenterBridgeMode();
        return;
    }

    Serial.begin(115200);
    delay(100);
    Serial.println("\n[RaceMetry] Initializing Pro 20Hz BLE Telemetry System...");

    // Отключение встроенных светодиодов на плате (GPIO 48 / GPIO 8)
    neopixelWrite(48, 0, 0, 0);
#ifdef RGB_BUILTIN
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
#endif
    pinMode(8, INPUT);
#ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, INPUT);
#endif

    // Инициализация очередей FreeRTOS
    telemetryCmdQueue = xQueueCreate(16, sizeof(Core0Command));
    splitQueue = xQueueCreate(16, sizeof(SplitEvent));
    completedRunQueue = xQueueCreate(8, sizeof(RunRecord));

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
    analogSetPinAttenuation(PIN_BAT_ADC, ADC_11db);
    delay(20);
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

    // 6. Пробуждение и инициализация GPS u-blox M10Q (Hardware UART1, 20 Hz)
    gpsEngine.begin(Serial1, GPS_BAUDRATE_TARGET);
    Serial.printf("[RaceMetry] GPS M10Q configured with UBX %d Hz\n", GPS_UPDATE_RATE_HZ);

    // 7. Инициализация гоночного ядра телеметрии
    telemetryEngine.begin(deviceSettings);

    // 8. Инициализация BLE Сервера (Nordic UART Service)
    bleEngine.begin(BLE_DEVICE_NAME);

    // 9. Запуск высокоприоритетной задачи телеметрии на ЯДРЕ 0 (200 Гц)
    xTaskCreatePinnedToCore(
        TelemetryTask,
        "TelemetryTask",
        4096,
        NULL,
        2,
        NULL,
        0
    );

    // 10. Запуск коммуникационной задачи BLE на ЯДРЕ 1 (15 Гц)
    xTaskCreatePinnedToCore(
        CommTask,
        "CommTask",
        8192,
        NULL,
        1,
        NULL,
        1
    );

    Serial.println("[RaceMetry] BLE System started successfully!");
}

/**
 * ============================================================================
 * ЯДРО 0: ВЫСОКОСКОРОСТНАЯ ОБРАБОТКА ДАТЧИКОВ И ТЕЛЕМЕТРИИ (200 Гц)
 * ============================================================================
 */
void TelemetryTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200 Гц цикл опроса

    RaceState prevRaceState = RaceState::IDLE_WAIT_STOP;

    for (;;) {
        // 0. Высокоскоростной опрос аппаратной кнопки на корпусе
        buttonManager.update();

        // Визуализация прогресса удержания для выключения питания
        if (buttonManager.isPressed()) {
            uint32_t pressDur = buttonManager.getPressDurationMs();
            if (pressDur >= 350) {
                ledController.showPowerOffHolding(buttonManager.getPowerOffProgressPct());
            }
        }

        // Диспетчер событий кнопки
        ButtonEvent btnEvt = buttonManager.popEvent();
        if (btnEvt == ButtonEvent::DOUBLE_CLICK) {
            Serial.printf("[RaceMetry BTN] Double Click -> Battery Status (%d%%, Mode %d)\n",
                currentBatPercent, deviceSettings.batteryIndicationMode);
            ledController.showBatteryStatus(currentBatPercent, deviceSettings.batteryIndicationMode);
        } else if (btnEvt == ButtonEvent::LONG_PRESS) {
            Serial.println("[RaceMetry BTN] Long Press (1.5s) -> Power Off");
            enterPowerOffDeepSleep();
        }

        // 1. Обработка команд управления от Ядра 1 (CommTask)
        Core0Command cmd;
        while (telemetryCmdQueue && xQueueReceive(telemetryCmdQueue, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case Core0CmdType::ARM:
                    if (gpsEngine.isReadyForRace() && gpsEngine.getData().speedKmh <= 1.5f) {
                        telemetryEngine.arm();
                        Serial.println("[RaceMetry] ARMED: Ready for launch!");
                    } else {
                        Serial.printf("[RaceMetry] Arm rejected: Fix=%d, Sats=%d, Spd=%.1f km/h\n",
                            gpsEngine.getData().fixType, gpsEngine.getData().numSats, gpsEngine.getData().speedKmh);
                    }
                    break;
                case Core0CmdType::RESET:
                    telemetryEngine.reset();
                    break;
                case Core0CmdType::SET_DISCIPLINE:
                    telemetryEngine.setDiscipline(cmd.payload.discipline);
                    break;
                case Core0CmdType::UPDATE_SETTINGS:
                    telemetryEngine.updateSettings(cmd.payload.settings);
                    break;
                case Core0CmdType::CALIBRATE_IMU:
                    ledController.setMode(LedMode::CALIBRATING);
                    if (imuEngine.calibrateZero(600)) {
                        float offGx, offGy, offGz;
                        imuEngine.getOffsets(offGx, offGy, offGz);
                        deviceSettings.imuOffsetGx = offGx;
                        deviceSettings.imuOffsetGy = offGy;
                        deviceSettings.imuOffsetGz = offGz;
                        storageManager.saveSettings(deviceSettings);
                    }
                    break;
                case Core0CmdType::POWER_OFF:
                    enterPowerOffDeepSleep();
                    break;
            }
        }

        // 2. Скоростной опрос акселерометра MPU-9250 (200 Гц)
        if (imuEngine.update()) {
            telemetryEngine.processImuSample(imuEngine.getLatestSample());
        }

        // 3. Потоковый разбор бинарных пакетов UBX GPS (20 Гц)
        if (gpsEngine.update()) {
            telemetryEngine.processGpsEpoch(gpsEngine.getLatestEpoch());
        }

        // 4. Проверка таймаута GPS (детектор зависания / потери потока)
        telemetryEngine.checkStaleTimeout(micros());

        RaceState curState = telemetryEngine.getState();

        // 5. Обработка моментальных отсечек (Split Events) -> светодиод и очередь BLE
        SplitEvent splitEvt;
        while (telemetryEngine.popSplitEvent(splitEvt)) {
            ledController.triggerSplitFlash();
            if (splitQueue) {
                xQueueSend(splitQueue, &splitEvt, 0);
            }
        }

        // 6. Обработка завершенных заездов -> очередь на Ядро 1 для записи в NVS и BLE
        RunRecord completedRun;
        if (telemetryEngine.popCompletedRun(completedRun)) {
            if (completedRunQueue) {
                xQueueSend(completedRunQueue, &completedRun, 0);
            }
        }

        // 7. Управление светодиодной индикацией
        if (!buttonManager.isPressed() || buttonManager.getPressDurationMs() < 350) {
            static bool wasGpsReady = false;
            bool isGpsReady = gpsEngine.isReadyForRace();

            if (isGpsReady && !wasGpsReady) {
                ledController.notifyFixAcquired();
            }
            wasGpsReady = isGpsReady;

            if (curState == RaceState::LAUNCH_DETECTED && prevRaceState == RaceState::ARMED) {
                ledController.triggerSplitFlash();
            }

            if (!isGpsReady) {
                ledController.setMode(LedMode::GPS_SEARCH);
            } else {
                switch (curState) {
                    case RaceState::ARMED:
                    case RaceState::LAUNCH_DETECTED:
                    case RaceState::MEASURING:
                        if (telemetryEngine.getDiscipline() == RaceDiscipline::BRAKE_100_0 && curState == RaceState::MEASURING) {
                            ledController.setMode(LedMode::BRAKING_ACTIVE);
                        } else {
                            ledController.setMode(LedMode::ARMED_READY);
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

        prevRaceState = curState;

        // 8. Синхронизация данных снимка для Ядра 1 (CommTask)
        portENTER_CRITICAL(&stateMutex);
        safeGpsData = gpsEngine.getData();
        safeImuData = imuEngine.getData();
        safeRaceState = curState;
        safeDiscipline = telemetryEngine.getDiscipline();
        safeLiveTimeSec = telemetryEngine.getCurrentTimeSec();
        safeLiveDistanceM = telemetryEngine.getCurrentDistanceM();
        safeLiveSpeedKmh = telemetryEngine.getCurrentSpeedKmh();
        safeLiveSlopePct = telemetryEngine.getCurrentSlopePct();
        safeGpsReady = gpsEngine.isReadyForRace();
        portEXIT_CRITICAL(&stateMutex);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * ============================================================================
 * ЯДРО 1: BLUETOOTH LOW ENERGY, ЗАМЕР БАТАРЕИ И ХРАНИЛИЩЕ NVS (15 Гц)
 * ============================================================================
 */
void CommTask(void* parameter) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / BLE_TELEMETRY_RATE_HZ); // 15 Гц
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint32_t lastBatSampleMs = 0;

    for (;;) {
        // 1. Периодический замер напряжения батареи (раз в 500 мс с фильтрацией)
        if (millis() - lastBatSampleMs >= 500 || lastBatSampleMs == 0) {
            lastBatSampleMs = millis();
            float rawV = readRawBatteryVoltage();
            if (currentBatVoltage < 0.1f) {
                currentBatVoltage = rawV;
            } else {
                currentBatVoltage = 0.85f * currentBatVoltage + 0.15f * rawV;
            }
            currentBatPercent = calculateBatteryPercentage(currentBatVoltage);
        }

        // 2. Обновление состояния BLE
        bleEngine.update();
        bool isConnected = bleEngine.isConnected();

        // 3. Обработка входящих команд BLE
        BleCommand bCmd;
        while (bleEngine.popCommand(bCmd)) {
            String cmd = bCmd.cmd;
            String val = bCmd.val;
            Serial.printf("[BLE CMD DISPATCH] cmd: %s, val: %s\n", cmd.c_str(), val.c_str());

            if (cmd == "arm") {
                Core0Command c; c.type = Core0CmdType::ARM;
                if (telemetryCmdQueue) xQueueSend(telemetryCmdQueue, &c, 0);
            } else if (cmd == "reset") {
                Core0Command c; c.type = Core0CmdType::RESET;
                if (telemetryCmdQueue) xQueueSend(telemetryCmdQueue, &c, 0);
            } else if (cmd == "set_disc") {
                int disc = val.toInt();
                if (disc >= 0 && disc <= 7) {
                    Core0Command c; c.type = Core0CmdType::SET_DISCIPLINE;
                    c.payload.discipline = (RaceDiscipline)disc;
                    if (telemetryCmdQueue) xQueueSend(telemetryCmdQueue, &c, 0);
                }
            } else if (cmd == "set_rollout") {
                deviceSettings.use1FootRollout = (val == "true" || val == "1");
                storageManager.saveSettings(deviceSettings);
                Core0Command c; c.type = Core0CmdType::UPDATE_SETTINGS;
                c.payload.settings = deviceSettings;
                if (telemetryCmdQueue) xQueueSend(telemetryCmdQueue, &c, 0);
            } else if (cmd == "set_units") {
                deviceSettings.metricUnits = (val == "true" || val == "1" || val == "metric");
                storageManager.saveSettings(deviceSettings);
            } else if (cmd == "set_bat_mode") {
                deviceSettings.batteryIndicationMode = (uint8_t)val.toInt();
                storageManager.saveSettings(deviceSettings);
                ledController.showBatteryStatus(currentBatPercent, deviceSettings.batteryIndicationMode);
            } else if (cmd == "calibrate_imu") {
                Core0Command c; c.type = Core0CmdType::CALIBRATE_IMU;
                if (telemetryCmdQueue) xQueueSend(telemetryCmdQueue, &c, 0);
            } else if (cmd == "run_diag") {
                runSystemDiagnostics();
            } else if (cmd == "power_off") {
                Core0Command c; c.type = Core0CmdType::POWER_OFF;
                if (telemetryCmdQueue) xQueueSend(telemetryCmdQueue, &c, 0);
            } else if (cmd == "get_history") {
                uint8_t count = storageManager.getSavedRunsCount();
                for (uint8_t i = 0; i < count; i++) {
                    RunRecord r;
                    if (storageManager.getRunRecord(i, r)) {
                        bleEngine.sendRunRecord(r);
                        vTaskDelay(pdMS_TO_TICKS(15));
                    }
                }
            } else if (cmd == "clear_history") {
                storageManager.clearAllRuns(true);
                bleEngine.sendDeviceInfo(deviceSettings, 0, safeGpsReady, safeGpsData.numSats, currentBatVoltage, currentBatPercent);
            } else if (cmd == "get_info") {
                runSystemDiagnostics();
                bleEngine.sendDeviceInfo(deviceSettings, storageManager.getSavedRunsCount(), safeGpsReady, safeGpsData.numSats, currentBatVoltage, currentBatPercent);
                PersonalBests pb;
                storageManager.getPersonalBests(pb);
                bleEngine.sendPersonalBests(pb);
            } else if (cmd == "ping") {
                // ping приходит только после startNotifications(): разрешаем
                // TX-трафик и подтверждаем фактический обратный канал.
                bleEngine.setClientReady(true);
                bleEngine.sendJson("{\"t\":\"pong\"}\n");
            }
        }

        // После физического подключения ничего не отправляем до подписки
        // клиента на TX notifications. Web-клиент после завершения полного
        // GATT handshake сам запрашивает get_info/get_history. Это исключает
        // конкуренцию service discovery с ранними notify на холодном старте.

        // 5. Обработка завершенных заездов из очереди
        RunRecord r;
        while (completedRunQueue && xQueueReceive(completedRunQueue, &r, 0) == pdTRUE) {
            storageManager.saveRunRecord(r);
            if (isConnected) {
                bleEngine.sendRunRecord(r);
                PersonalBests pb;
                storageManager.getPersonalBests(pb);
                bleEngine.sendPersonalBests(pb);
            }
        }

        // 6. Обработка отсечек из очереди
        SplitEvent s;
        while (splitQueue && xQueueReceive(splitQueue, &s, 0) == pdTRUE) {
            if (isConnected) {
                bleEngine.sendSplitEvent(s);
            }
        }

        // 7. Получение безопасного снимка телеметрии
        GpsData localGps;
        ImuData localImu;
        RaceState localState;
        RaceDiscipline localDisc;
        float localLiveTime;
        float localDistanceM;
        float localSpeedKmh;
        float localSlopePct;

        portENTER_CRITICAL(&stateMutex);
        localGps = safeGpsData;
        localImu = safeImuData;
        localState = safeRaceState;
        localDisc = safeDiscipline;
        localLiveTime = safeLiveTimeSec;
        localDistanceM = safeLiveDistanceM;
        localSpeedKmh = safeLiveSpeedKmh;
        localSlopePct = safeLiveSlopePct;
        portEXIT_CRITICAL(&stateMutex);

        // 8. Трансляция телеметрии по BLE (15 Гц)
        if (isConnected) {
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
    Serial.println("\n[RaceMetry] Powering OFF... Entering Ultra-Low-Power Deep Sleep.");

    // 1. Уведомляем подключенный смартфон по BLE
    bleEngine.sendJson("{\"t\":\"shutdown\"}\n");
    delay(50);

    // 2. Программный перевод u-blox M10Q в энергосберегающий режим Backup Standby (~15 мкА)
    gpsEngine.powerOff();

    // 3. Анимация выключения на светодиоде (3 красные вспышки и плавное затухание)
    ledController.showPowerOffAnimation();
    ledController.turnOff();

    // 4. Обработка кнопки при наличии
    if (PIN_BTN != 255) {
        pinMode(PIN_BTN, INPUT_PULLUP);
        while (digitalRead(PIN_BTN) == LOW) {
            delay(10);
        }
        delay(250);
    }

    // 5. Фиксация линии UART TX в HIGH для исключения паразитных токов через GPS
    pinMode(PIN_GPS_TX, OUTPUT);
    digitalWrite(PIN_GPS_TX, HIGH);
    gpio_hold_en((gpio_num_t)PIN_GPS_TX);

    // 6. Настройка пробуждения (если кнопка назначена)
    if (PIN_BTN != 255) {
        rtc_gpio_init((gpio_num_t)PIN_BTN);
        rtc_gpio_set_direction((gpio_num_t)PIN_BTN, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_en((gpio_num_t)PIN_BTN);
        rtc_gpio_pulldown_dis((gpio_num_t)PIN_BTN);
        gpio_hold_en((gpio_num_t)PIN_BTN);
        gpio_deep_sleep_hold_en();
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BTN, 0);
    }

    // 7. Переход в глубокий сон
    esp_deep_sleep_start();
}

/**
 * ============================================================================
 * ФУНКЦИЯ САМОДИАГНОСТИКИ И ОТЧЕТА ОБ ИНИЦИАЛИЗАЦИИ
 * ============================================================================
 */
void runSystemDiagnostics() {
    if (!imuEngine.isReady()) {
        imuEngine.reinit();
    }
    bool imuOk = imuEngine.isReady();
    const char* imuMsg = imuEngine.getStatusMessage();

    bool gpsOk = gpsEngine.isReceivingBytes();
    const char* gpsMsg = gpsOk ? "GNSS u-blox M10Q (20 Hz, 460800 baud): OK" : "GNSS: Нет входящих данных UART";

    bool storageOk = storageManager.isOk();
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
    uint32_t minMv = 0xFFFFFFFF;
    uint32_t maxMv = 0;
    uint32_t sumMv = 0;

    // 32 отсчета с отсечением крайних шумов
    for (int i = 0; i < 32; i++) {
        uint32_t sample = analogReadMilliVolts(PIN_BAT_ADC);
        if (sample < minMv) minMv = sample;
        if (sample > maxMv) maxMv = sample;
        sumMv += sample;
        delayMicroseconds(100);
    }
    // Вычитаем 2 крайних выброса
    sumMv -= (minMv + maxMv);
    float avgMv = (float)sumMv / 30.0f;
    float instantV = (avgMv / 1000.0f) * BAT_DIVIDER_RATIO;

    // Экспоненциальный фильтр (EMA) для устранения просадок при работе BLE/LED
    static float s_filteredV = 0.0f;
    if (s_filteredV < 1.0f) {
        s_filteredV = instantV;
    } else {
        s_filteredV = s_filteredV * 0.88f + instantV * 0.12f;
    }
    return s_filteredV;
#else
    return 0.0f;
#endif
}

uint8_t calculateBatteryPercentage(float v) {
    if (v < 2.5f) return 0; // Питание от USB без батареи
    if (v >= BAT_VOLTAGE_MAX) return 100;
    if (v <= BAT_VOLTAGE_MIN) return 0;
    // Реалистичная табличная интерполяция разрядной кривой Li-Ion (3.30V - 4.18V):
    if (v >= 4.05f) return 90 + (uint8_t)((v - 4.05f) / (BAT_VOLTAGE_MAX - 4.05f) * 10.0f);
    if (v >= 3.90f) return 70 + (uint8_t)((v - 3.90f) / (4.05f - 3.90f) * 20.0f);
    if (v >= 3.78f) return 45 + (uint8_t)((v - 3.78f) / (3.90f - 3.78f) * 25.0f);
    if (v >= 3.65f) return 20 + (uint8_t)((v - 3.65f) / (3.78f - 3.65f) * 25.0f);
    if (v >= 3.45f) return 5  + (uint8_t)((v - 3.45f) / (3.65f - 3.45f) * 15.0f);
    return (uint8_t)((v - 3.30f) / (3.45f - 3.30f) * 5.0f);
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
