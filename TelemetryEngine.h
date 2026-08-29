#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                   TELEMETRY ENGINE (RACE & DRAG MATH)
 * ============================================================================
 * Главное математическое ядро прибора:
 * - Раздельная обработка быстрых выборок IMU (200 Гц) и эпох GNSS (20 Гц)
 * - Непрерывный интегратор дистанции (исправлен 1-Foot Rollout без разрыва времени)
 * - Честная субмиллисекундная интерполяция по реальным эпохам GNSS
 * - Контроль качества и устаревания данных GPS (GPS_STALE / FIX_LOST)
 * - Очередь событий промежуточных отсечек (SplitEvent)
 */

class TelemetryEngine {
public:
    TelemetryEngine();

    void begin(const DeviceSettings& settings);
    void updateSettings(const DeviceSettings& settings);
    void setDiscipline(RaceDiscipline discipline);

    // Управление состоянием
    void arm(uint64_t currentUs = 0);
    void reset(RaceAbortReason reason = RaceAbortReason::USER_RESET);

    // Входные потоки данных
    void processImuSample(const ImuSample& imu);
    void processGpsEpoch(const GpsEpoch& epoch);
    void checkStaleTimeout(uint64_t currentUs);

    // Совместимый метод комплексного обновления
    void process(const GpsData& gps, const ImuData& imu, uint64_t currentUs = 0);

    // Геттеры состояния
    RaceState getState() const { return _state; }
    RaceDiscipline getDiscipline() const { return _discipline; }
    RaceAbortReason getLastAbortReason() const { return _lastAbortReason; }
    const RunRecord& getLastRun() const { return _lastRun; }
    const RunRecord& getCurrentRun() const { return _currentRun; }

    // Живые параметры
    float getCurrentSpeedKmh() const { return _liveSpeedKmh; }
    float getCurrentDistanceM() const { return _liveDistanceM; }
    float getCurrentGLong() const { return _liveGLong; }
    float getCurrentSlopePct() const { return _liveSlopePct; }
    float getCurrentTimeSec(uint64_t currentUs = 0) const;

    // События отсечек и завершения
    bool popSplitEvent(SplitEvent& event);
    bool popCompletedRun(RunRecord& record);
    bool checkAndClearSplitTrigger();

private:
    RaceState _state;
    RaceDiscipline _discipline;
    RaceAbortReason _lastAbortReason;
    DeviceSettings _settings;

    RunRecord _currentRun;
    RunRecord _lastRun;

    // Живые параметры
    float _liveSpeedKmh;
    float _liveDistanceM;
    float _liveGLong;
    float _liveSlopePct;

    // Временные шкалы
    uint64_t _launchTimeUs;
    uint64_t _rolloutCrossingUs;
    uint64_t _rolloutOffsetUs;
    bool _rolloutPassed;

    // Предыдущая валидная эпоха GNSS
    bool _hasPrevEpoch;
    uint32_t _prevGpsTowMs;
    uint64_t _prevEpochArrivalUs;
    float _prevEpochSpeedMs;
    float _prevEpochSpeedKmh;
    float _prevEpochDistM;
    float _prevEpochAltM;

    // Контроль поступления пакетов
    uint64_t _lastGpsArrivalUs;
    uint32_t _lastGpsTowMs;
    uint32_t _lastGpsSequence;

    // Промежуточные отсечки
    float _timeAt100KmhSec;
    float _timeAt200KmhSec;
    bool _passed100Kmh;
    bool _passed200Kmh;
    bool _splitTriggered;

    // Торможение
    bool _brakeActive;
    float _brakeStartDistM;
    uint64_t _brakeStartTimeUs;

    // Очередь событий отсечек
    static const uint8_t MAX_SPLIT_EVENTS = 16;
    SplitEvent _splitEventQueue[MAX_SPLIT_EVENTS];
    uint8_t _splitQueueHead;
    uint8_t _splitQueueTail;

    // Флаг готового завершенного заезда
    bool _hasCompletedRun;

    // Вспомогательные методы
    float _calcElapsedSec(uint64_t timestampUs) const;
    void _pushSplitEvent(SplitType type, const char* name, float timeSec, float speedKmh);
    void _handleLaunch(uint64_t launchUs, float startAltM, uint32_t epochSec);
    void _checkSpeedSplits(const GpsEpoch& epoch, float t1, float v1, float t2, float v2);
    void _checkDistanceSplits(const GpsEpoch& epoch, float t1, float v1, float s1, float h1, float t2, float v2, float s2, float h2);
    void _interpolateSpeedSplit(float targetKmh, SplitType type, const char* name, SplitTime& split, float t1, float v1, float t2, float v2);
    void _interpolateDistanceSplit(float targetM, SplitType type, const char* name, SplitTime& split, float t1, float v1, float s1, float h1, float t2, float v2, float s2, float h2);
    void _finalizeRun(RaceState finalState, RaceAbortReason abortReason = RaceAbortReason::NONE, float finishTimeSec = 0.0f, float finishAltM = 0.0f);
    void _checkRunCompletion();
};

