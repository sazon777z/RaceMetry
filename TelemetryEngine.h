#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                   TELEMETRY ENGINE (RACE & DRAG MATH)
 * ============================================================================
 * Главное математическое ядро прибора.
 * Реализует:
 * - Автоматический конечный автомат заезда (Wait Stop -> Armed -> Running -> Done)
 * - 1-Foot Rollout (раскат по регламенту NHRA / Dragy)
 * - Субвыборочную квадратично-сплайновую интерполяцию отсечек времени и скорости
 * - Контроль уклона полосы (Slope Check) и валидацию заезда
 * - Все дисциплины: 0-100, 100-200, 1/4 мили (402м), 100-0 км/ч торможение
 */

class TelemetryEngine {
public:
    TelemetryEngine();

    void begin(const DeviceSettings& settings);

    // Основной цикл обновления телеметрии (вызывается на каждом пакете GPS и IMU)
    void process(const GpsData& gps, const ImuData& imu);

    // Управление состоянием
    void arm();                  // Принудительно взвести прибор
    void reset();                // Сбросить текущий заезд
    void setDiscipline(RaceDiscipline discipline);

    // Геттеры текущего состояния
    RaceState getState() const { return _state; }
    RaceDiscipline getDiscipline() const { return _discipline; }
    const RunRecord& getLastRun() const { return _lastRun; }
    const RunRecord& getCurrentRun() const { return _currentRun; }
    
    // Живые параметры текущего заезда
    float getCurrentSpeedKmh() const { return _liveSpeedKmh; }
    float getCurrentDistanceM() const { return _liveDistanceM; }
    float getCurrentTimeSec() const;
    float getCurrentGLong() const { return _liveGLong; }
    float getCurrentSlopePct() const { return _liveSlopePct; }

    // Настройки
    void updateSettings(const DeviceSettings& settings);

private:
    RaceState _state;
    RaceDiscipline _discipline;
    DeviceSettings _settings;

    RunRecord _currentRun;
    RunRecord _lastRun;

    // Живые переменные
    float _liveSpeedKmh;
    float _liveDistanceM;
    float _liveGLong;
    float _liveSlopePct;

    // Временные метки и точки
    uint32_t _stopStartMs;
    uint32_t _launchTimeUs;
    uint32_t _rolloutTimeUs;
    bool _rolloutPassed;

    // Предыдущая выборка для интерполяции
    float _prevSpeedMs;
    float _prevSpeedKmh;
    float _prevDistM;
    float _prevTimeSec;
    float _prevAltM;
    bool _hasPrevSample;

    // Состояния замеров интервалов
    float _timeAt100KmhSec;
    bool _passed100Kmh;
    bool _passed200Kmh;

    // Замер торможения
    bool _brakeActive;
    float _brakeStartDistM;
    uint32_t _brakeStartTimeUs;

    // Вспомогательные математические методы
    void _handleStateTransitions(const GpsData& gps, const ImuData& imu);
    void _updateMeasuring(const GpsData& gps, const ImuData& imu);
    void _interpolateSpeedSplit(float targetKmh, SplitTime& split, float t1, float v1, float t2, float v2, float s1, float s2);
    void _interpolateDistanceSplit(float targetM, SplitTime& split, float t1, float v1, float t2, float v2, float s1, float s2);
    void _finalizeRun();
    void _checkRunCompletion();
};
