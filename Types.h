#pragma once
#include <Arduino.h>
#include "Config.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                          ТИПЫ ДАННЫХ И СТРУКТУРЫ (BLE)
 * ============================================================================
 */

// Состояние гоночного автомата телеметрии
enum class RaceState : uint8_t {
    IDLE_WAIT_STOP = 0, // Ожидание полной остановки автомобиля
    ARMED,              // Готов к старту (авто стоит, светодиод зеленый)
    LAUNCH_DETECTED,    // Зафиксирован старт по акселерометру / скорости
    MEASURING,          // Заезд в процессе
    FINISHED,           // Заезд завершен, вычислены все интервалы
    ABORTED             // Заезд прерван (остановка до финиша или сбой)
};

// Дисциплина / Режим замера
enum class RaceDiscipline : uint8_t {
    ALL_IN_ONE_DRAG = 0, // Автоматический замер всех скоростей (0-60, 0-100, 100-200) и дистанций (60ft, 1/8mi, 1/4mi)
    SPEED_0_60,          // Только 0 - 60 км/ч
    SPEED_0_100,         // Только 0 - 100 км/ч
    SPEED_100_200,       // Только 100 - 200 км/ч
    SPEED_0_200,         // Только 0 - 200 км/ч
    QUARTER_MILE_402M,   // Только 1/4 мили (402.33 м)
    HALF_MILE_804M,      // Только 1/2 мили (804.67 м)
    BRAKE_100_0          // Торможение 100 - 0 км/ч
};

// Данные со спутников GNSS (из UBX-NAV-PVT и UBX-NAV-SAT)
struct GpsData {
    double lat;             // Широта в градусах
    double lon;             // Долгота в градусах
    float altMSL;           // Высота над уровнем моря (м)
    float speedKmh;         // Скорость по Doppler 3D velocity (км/ч)
    float speedMs;          // Скорость в м/с
    float headingDeg;       // Курс движения (0 - 360 град)
    float hAccM;            // Оценка горизонтальной точности (м)
    float vAccM;            // Оценка вертикальной точности (м)
    float sAccKmh;          // Оценка точности скорости (км/ч)
    uint8_t numSats;        // Общее количество активных спутников в решении
    uint8_t satsGps;        // Спутники GPS (США)
    uint8_t satsGlonass;    // Спутники ГЛОНАСС (Россия)
    uint8_t satsGalileo;    // Спутники Galileo (Европа)
    uint8_t satsBeidou;     // Спутники BeiDou (Китай)
    uint8_t fixType;        // 0=NoFix, 2=2D, 3=3D, 4=GNSS+dead reckoning
    bool validFix;          // Флаг валидности 3D фикса
    uint32_t towMs;         // GPS Time of Week (мс)
    uint16_t year;          // Год UTC
    uint8_t month;          // Месяц UTC (1-12)
    uint8_t day;            // День UTC (1-31)
    uint8_t hour;           // Часы UTC (0-23)
    uint8_t min;            // Минуты UTC (0-59)
    uint8_t sec;            // Секунды UTC (0-59)
    uint32_t epochSeconds;  // Unix Epoch Timestamp (секунды с 01.01.1970 UTC)
    uint32_t lastUpdateMs;  // Время последнего полученного пакета (millis)
    float pDOP;             // Position Dilution of Precision
};

// Данные с инерциального датчика IMU (MPU-9250)
struct ImuData {
    float accelX;           // Ускорение X в G
    float accelY;           // Ускорение Y в G
    float accelZ;           // Ускорение Z в G
    float gyroX;            // Угловая скорость X (град/с)
    float gyroY;            // Угловая скорость Y (град/с)
    float gyroZ;            // Угловая скорость Z (град/с)
    float gLongitudinal;    // Продольное ускорение автомобиля (разгон/торможение) в G
    float gLateral;         // Поперечное ускорение (поворот) в G
    float gVertical;        // Вертикальное ускорение в G
    float gPeakAccel;       // Максимальное ускорение за заезд (G)
    float gPeakBrake;       // Максимальное тормозное замедление (G)
    bool isCalibrated;      // Флаг успешной калибровки нуля
    uint32_t lastSampleUs;  // Время выборки (micros)

    inline bool isLaunchDetected() const {
        return gLongitudinal >= LAUNCH_G_THRESHOLD;
    }
};

// Точка трека телеметрии во время заезда
struct TelemetrySample {
    float timeSec;          // Время от старта (с)
    float speedKmh;         // Скорость (км/ч)
    float distanceM;        // Пройденная дистанция (м)
    float altitudeM;        // Высота (м)
    float gLong;            // Продольная перегрузка (G)
};

// Промежуточные отсечки заезда (Splits)
struct SplitTime {
    float timeSec;          // Время прохождения отсечки (с)
    float trapSpeedKmh;     // Скорость на отсечке (км/ч)
    bool achieved;          // Достигнута ли отсечка
};

// Полный отчет о заезде (Run Record)
struct RunRecord {
    uint32_t id;            // Порядковый номер заезда
    uint32_t timestampUtc;  // Время заезда (Unix timestamp)
    RaceDiscipline discipline;

    // Скоростные отсечки (время в секундах)
    SplitTime split0_60;    // 0 - 60 км/ч
    SplitTime split0_100;   // 0 - 100 км/ч
    SplitTime split100_150; // 100 - 150 км/ч
    SplitTime split100_200; // 100 - 200 км/ч
    SplitTime split0_200;   // 0 - 200 км/ч
    SplitTime split200_300; // 200 - 300 км/ч
    SplitTime split60_120;  // 60 - 120 км/ч
    SplitTime split80_120;  // 80 - 120 км/ч

    // Дистанционные отсечки драг-рейсинга
    SplitTime split60ft;    // 60 футов (18.28 м)
    SplitTime split330ft;   // 330 футов (100.58 м)
    SplitTime split1_8mi;   // 1/8 мили (201.16 м)
    SplitTime split1000ft;  // 1000 футов (304.8 м)
    SplitTime split1_4mi;   // 1/4 мили (402.33 м)
    SplitTime split1_2mi;   // 1/2 мили (804.67 м)
    SplitTime split1mi;     // 1 миля (1609.34 м)

    // Торможение
    SplitTime split100_0;   // 100 - 0 км/ч (время)
    float brakeDist100_0M;  // Дистанция торможения со 100 км/ч (м)

    // Общие метрики заезда
    float maxSpeedKmh;      // Максимальная скорость за заезд (км/ч)
    float maxAccelG;        // Пиковая перегрузка разгона (G)
    float startAltM;        // Высота на старте (м)
    float finishAltM;       // Высота на финише (м)
    float totalDistanceM;   // Общая дистанция (м)
    float totalDurationSec; // Общее время заезда (с)
    float slopePct;         // Итоговый уклон трассы в процентах ((H_fin - H_start) / Dist * 100%)
    bool isValidSlope;      // Валиден ли заезд по уклону (уклон >= MAX_VALID_SLOPE_PCT)
    bool rolloutUsed;       // Был ли применен 1-Foot Rollout
};

// Личные рекорды (Personal Bests)
struct PersonalBests {
    float best0_60;
    float best0_100;
    float best100_200;
    float best1_4mi;
    float best1_4miSpeed;
    float best60ft;
    float best100_0Dist;
};

// События кнопок
enum class ButtonEvent : uint8_t {
    NONE = 0,
    CLICK,
    DOUBLE_CLICK,
    LONG_PRESS
};

// Состояние аккумулятора
struct BatteryData {
    float voltage;       // Напряжение батареи в вольтах (3.3 - 4.2 В)
    uint8_t percentage;  // Оценка заряда (0 - 100%)
    bool isConnected;    // Подключен ли аккумулятор
};

// Пользовательские настройки (сохраняются в NVS)
struct DeviceSettings {
    bool use1FootRollout;       // Использовать 1 фут раската (по умолчанию true, как в Dragy)
    bool metricUnits;           // true = км/ч и метры, false = мили/ч и футы
    float slopeTolerancePct;    // Допустимый уклон (по умолчанию -1.0%)
    uint8_t displayBrightness;  // Резерв для обратной совместимости NVS
    float imuOffsetGx;          // Калибровочный оффсет X
    float imuOffsetGy;          // Калибровочный оффсет Y
    float imuOffsetGz;          // Калибровочный оффсет Z
    uint8_t defaultScreen;      // Резерв для обратной совместимости NVS
};
