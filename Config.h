#pragma once
#include <Arduino.h>

/**
 * ============================================================================
 *                          RACEMETRY PRO METER
 *                 КОНФИГУРАЦИЯ ОБОРУДОВАНИЯ И ПАРАМЕТРОВ (BLE)
 * ============================================================================
 * Микроконтроллер: ESP32-S3 Super Mini
 * GPS: u-blox M10Q (UB10050F) на 20 Гц (UBX-NAV-PVT)
 * IMU: MPU-9250 / MPU-6500 (200 Гц Launch Jerk Trigger)
 * Индикатор: Внешний адресный RGB светодиод WS2812B (GPIO 10)
 * Питание: Модуль MH-CD42 (5V Boost + Li-Ion Charge + 4-LED Indicator)
 * Органы управления: Кнопка управления (KEY на MH-CD42 / GPIO 11 к GND)
 * Беспроводная связь: Bluetooth Low Energy 5.0 (Nordic UART Service)
 * Мониторинг АКБ: Делитель напряжения на BAT+ модуля MH-CD42 -> GPIO 1 (АЦП)
 * ============================================================================
 */

// ----------------------------------------------------------------------------
// 1. НАЗНАЧЕНИЕ ПИНОВ (GPIO)
// ----------------------------------------------------------------------------
// GPS u-blox M10Q (Hardware UART1) — Подключено к шелкографии RX / TX платы
#define PIN_GPS_RX              44  // ESP32 RX (GPIO 44) <- GPS TX
#define PIN_GPS_TX              43  // ESP32 TX (GPIO 43) -> GPS RX
#define GPS_BAUDRATE_INITIAL    9600
#define GPS_BAUDRATE_TARGET     460800  // Скоростной порт для минимальной латентности (UBX 20Hz)
#define GPS_UPDATE_RATE_HZ      20      // Частота навигации: 20 Гц (каждые 50 мс)

// IMU Акселерометр/Гироскоп MPU-9250/6500 (I2C) — Фактическое подключение
#define PIN_I2C_SCL             13  // I2C SCL -> GPIO 13
#define PIN_I2C_SDA             12  // I2C SDA -> GPIO 12
#define I2C_FREQUENCY           400000  // 400 кГц (Fast Mode)
#define IMU_I2C_ADDR            0x68    // Адрес 0x68 (если AD0 к GND) или 0x69
#define IMU_SAMPLE_RATE_HZ      200     // Частота дискретизации акселерометра (200 Гц)

// Адресный RGB светодиод WS2812B (Встроенный на плате ESP32-S3 -> GPIO 48)
#define PIN_WS2812              48  // Встроенный адресный RGB светодиод WS2812B (GPIO 48)
#define NUM_WS2812_LEDS         1

// Аппаратная кнопка управления (GPIO 11 к GND или через KEY пин платы MH-CD42)
#define PIN_BTN                 11      // Кнопка на корпусе -> GPIO 11 (к GND)
#define BTN_HOLD_POWER_OFF_MS   1500    // Длительное удержание для выключения (1.5 сек)
#define BTN_HOLD_POWER_ON_MS    400     // Удержание кнопки для включения (0.4 сек)

// Контроль заряда аккумулятора (Battery Monitor)
// Подключается к силовому контакту BAT+ платы MH-CD42 через резистивный делитель 1:2 (например, 220k / 220k)
#define ENABLE_BATTERY_MONITOR  true    // Включить замер напряжения батареи
#define PIN_BAT_ADC             1       // Пин АЦП (GPIO 1) для средней точки делителя 1:2
#define BAT_DIVIDER_RATIO       2.024f  // Точно калиброванный коэффициент делителя (компенсация погрешности резисторов)
#define BAT_VOLTAGE_MIN         3.30f   // Напряжение полностью разряженной батареи (0%)
#define BAT_VOLTAGE_MAX         4.18f   // Напряжение полностью заряженной батареи (100%)

// ----------------------------------------------------------------------------
// 2. ПАРАМЕТРЫ BLUETOOTH LOW ENERGY (BLE)
// ----------------------------------------------------------------------------
#define BLE_DEVICE_NAME         "RaceMetry-Pro"
// Стандартный сервис Nordic UART (NUS) для максимальной совместимости с Web Bluetooth и мобильными ОС
#define BLE_NUS_SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_RX_CHAR_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_TX_CHAR_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Частота потоковой трансляции телеметрии по BLE на смартфон (Гц)
#define BLE_TELEMETRY_RATE_HZ   15      // 15 пакетов в секунду (плавный 60fps UI со сглаживанием)

// Режим прямого моста USB <-> GPS для программы u-blox U-Center / u-center 2
#define GPS_BRIDGE_MODE         false

// ----------------------------------------------------------------------------
// 3. МАТЕМАТИЧЕСКИЕ И ГОНОЧНЫЕ КОНСТАНТЫ
// ----------------------------------------------------------------------------

// Пороги детекции старта (Launch Detection)
#define LAUNCH_G_THRESHOLD      0.15f   // Продольная перегрузка в G (> 0.15G = начало движения)
#define LAUNCH_SPEED_KMH        0.80f   // Скорость GPS в км/ч для дублирующего триггера
#define STOP_SPEED_THRESHOLD    0.50f   // Скорость ниже которой авто считается остановившимся
#define STOP_TIME_STABLE_MS     1200    // Время покоя (мс) для перехода в режим "ГОТОВ" (ARMED)

// 1-Foot Rollout (Стандарт Drag-Racing / NHRA / Dragy)
#define ONE_FOOT_METERS         0.3048f // 1 фут = 30.48 см

// Контроль наклона (Slope Checking)
#define MAX_VALID_SLOPE_PCT     -1.0f   // Максимально допустимый уклон вниз (-1.0%) для валидации
#define SLOPE_TOLERANCE_WINDOW  1.5f    // Предел отображения

// Буфер телеметрических точек заезда (для сплайн-интерполяции и графика G)
#define MAX_RUN_SAMPLES         400     // 400 точек при 10 Гц = 40 секунд записи заезда

// Хранилище энергонезависимой памяти (Preferences)
#define NVS_NAMESPACE           "racemetry_cfg"
#define MAX_SAVED_RUNS          20      // Количество сохраняемых заездов в истории
#define NVS_STORAGE_MAGIC       0x524DU // "RM" (RaceMetry)
#define NVS_SCHEMA_VERSION      1U      // Версия схемы хранения

// Контроль качества и свежести данных GNSS
#define GPS_STALE_TIMEOUT_MS    350     // Максимально допустимый интервал между пакетами GNSS при заезде (мс)
#define GPS_MIN_RACE_SATS       6       // Минимальное количество спутников для заезда
#define GPS_MAX_SACC_KMH        4.0f    // Максимально допустимая погрешность скорости sAcc (км/ч)

// Версия прошивки
#define RACEMETRY_FW_VERSION    "v2.1.0 BLE"
#define DRAGON_FW_VERSION       RACEMETRY_FW_VERSION
