#pragma once
#include <Arduino.h>

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                 КОНФИГУРАЦИЯ ОБОРУДОВАНИЯ И ПАРАМЕТРОВ (BLE)
 * ============================================================================
 * Микроконтроллер: ESP32-S3 Super Mini
 * GPS: u-blox M10Q (UB10050F) на 10-18 Гц (UBX-NAV-PVT)
 * IMU: MPU-9250 / MPU-6500 (200 Гц Launch Jerk Trigger)
 * Индикатор: WS2812B (Адресный RGB)
 * Беспроводная связь: Bluetooth Low Energy 5.0 (Nordic UART Service)
 * Кнопки: 2 тактовые кнопки с подтяжкой к VCC/GND
 * ============================================================================
 */

// ----------------------------------------------------------------------------
// 1. НАЗНАЧЕНИЕ ПИНОВ (GPIO)
// ----------------------------------------------------------------------------

// GPS u-blox M10Q (Hardware UART1) — Подключено к шелкографии RX / TX платы
#define PIN_GPS_RX              44  // ESP32 RX (GPIO 44) <- GPS TX
#define PIN_GPS_TX              43  // ESP32 TX (GPIO 43) -> GPS RX
#define GPS_BAUDRATE_INITIAL    9600
#define GPS_BAUDRATE_TARGET     460800  // Скоростной порт для минимальной латентности (UBX 18Hz)
#define GPS_UPDATE_RATE_HZ      18      // Частота навигации: 18 Гц (каждые 55 мс)

// IMU Акселерометр/Гироскоп MPU-9250/6500 (I2C) — Фактическое подключение
#define PIN_I2C_SCL             12  // I2C SCL -> GPIO 12
#define PIN_I2C_SDA             13  // I2C SDA -> GPIO 13
#define I2C_FREQUENCY           400000  // 400 кГц (Fast Mode)
#define IMU_I2C_ADDR            0x68    // Адрес 0x68 (если AD0 к GND) или 0x69
#define IMU_SAMPLE_RATE_HZ      200     // Частота дискретизации акселерометра (200 Гц)

// Адресный светодиод WS2812B (NeoPixel)
#define PIN_WS2812              11  // Data pin (GPIO 11)
#define NUM_WS2812_LEDS         1

// Кнопки управления (активный уровень - LOW, INPUT_PULLUP)
#define PIN_BTN_LEFT            10  // Кнопка 1 (Взведение / Режим) -> GPIO 10
#define PIN_BTN_RIGHT           9   // Кнопка 2 (Сброс / Калибровка) -> GPIO 9
#define BTN_DEBOUNCE_MS         35  // Защита от дребезга (мс)
#define BTN_LONG_PRESS_MS       600 // Порог длинного нажатия (мс)

// ----------------------------------------------------------------------------
// 2. ПАРАМЕТРЫ BLUETOOTH LOW ENERGY (BLE)
// ----------------------------------------------------------------------------
#define BLE_DEVICE_NAME         "DRAGon-Telemetry"
// Стандартный сервис Nordic UART (NUS) для максимальной совместимости с Web Bluetooth и мобильными ОС
#define BLE_NUS_SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_RX_CHAR_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_TX_CHAR_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Частота потоковой трансляции телеметрии по BLE на смартфон (Гц)
#define BLE_TELEMETRY_RATE_HZ   15      // 15 пакетов в секунду (плавный 60fps UI со сглаживанием)

// Режим прямого моста USB <-> GPS для программы u-blox U-Center
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
#define NVS_NAMESPACE           "dragon_cfg"
#define MAX_SAVED_RUNS          20      // Количество сохраняемых заездов в истории

// Версия прошивки
#define DRAGON_FW_VERSION       "v2.0.0 BLE"
