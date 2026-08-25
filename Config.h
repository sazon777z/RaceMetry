#pragma once
#include <Arduino.h>

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                      КОНФИГУРАЦИЯ ОБОРУДОВАНИЯ И ПИНОВ
 * ============================================================================
 * Микроконтроллер: ESP32-S3 Super Mini
 * GPS: u-blox M10Q (UB10050F)
 * IMU: MPU-9250 / MPU-6500 / MPU-9255
 * Дисплей: 1.47" IPS SPI (ST7789, 172x320)
 * Светодиод: WS2812B (Адресный RGB)
 * Кнопки: 2 тактовые кнопки с подтяжкой к VCC/GND
 * ============================================================================
 */

// ----------------------------------------------------------------------------
// 1. НАЗНАЧЕНИЕ ПИНОВ (GPIO)
// ----------------------------------------------------------------------------

// Дисплей 1.47" IPS ST7789 (SPI) — Фактическое подключение пользователя
#define PIN_LCD_SCLK            1   // SCL (SPI Clock) -> GPIO 1
#define PIN_LCD_MOSI            2   // SDA (SPI MOSI)  -> GPIO 2
#define PIN_LCD_MISO            -1  // Не используется
#define PIN_LCD_RST             3   // RST (Reset)     -> GPIO 3
#define PIN_LCD_DC              4   // DC (Data/Cmd)   -> GPIO 4
#define PIN_LCD_CS              5   // CS (Chip Select)-> GPIO 5
#define PIN_LCD_BLK             6   // IPL / BLK (Подсветка PWM) -> GPIO 6

#define LCD_WIDTH               320 // Ширина в горизонтальной ориентации
#define LCD_HEIGHT              172 // Высота в горизонтальной ориентации
#define LCD_ROTATION            1   // Альбомная ориентация (0-3)

// GPS u-blox M10Q (Hardware UART1)
#define PIN_GPS_RX              7   // ESP32 RX <- GPS TX (GPIO 7)
#define PIN_GPS_TX              8   // ESP32 TX -> GPS RX (GPIO 8)
#define GPS_BAUDRATE_INITIAL    9600
#define GPS_BAUDRATE_TARGET     460800  // Скоростной порт для минимальной латентности (UBX 10-18Hz)
#define GPS_UPDATE_RATE_HZ      10      // Частота навигации (10 Гц в режиме Multi-GNSS)

// IMU Акселерометр/Гироскоп MPU-9250/6500 (I2C)
#define PIN_I2C_SDA             9   // I2C SDA (GPIO 9)
#define PIN_I2C_SCL             10  // I2C SCL (GPIO 10)
#define I2C_FREQUENCY           400000  // 400 кГц (Fast Mode)
#define IMU_I2C_ADDR            0x68    // Адрес 0x68 (если AD0 к GND) или 0x69
#define IMU_SAMPLE_RATE_HZ      200     // Частота дискретизации акселерометра (200 Гц)

// Адресный светодиод WS2812B (NeoPixel)
#define PIN_WS2812              11  // Data pin (GPIO 11)
#define NUM_WS2812_LEDS         1

// Кнопки управления (активный уровень - LOW, INPUT_PULLUP)
#define PIN_BTN_LEFT            12  // Кнопка 1 (Навигация / Режим) -> GPIO 12
#define PIN_BTN_RIGHT           13  // Кнопка 2 (Действие / Сброс / Старт) -> GPIO 13
#define BTN_DEBOUNCE_MS         35  // Защита от дребезга (мс)
#define BTN_LONG_PRESS_MS       600 // Порог длинного нажатия (мс)

// ----------------------------------------------------------------------------
// 2. МАТЕМАТИЧЕСКИЕ И ГОНОЧНЫЕ КОНСТАНТЫ
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
#define DRAGON_FW_VERSION       "v1.0.0 Pro"
