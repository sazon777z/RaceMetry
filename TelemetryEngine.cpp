#include "TelemetryEngine.h"
#include <math.h>
#include <string.h>

TelemetryEngine::TelemetryEngine()
    : _state(RaceState::IDLE_WAIT_STOP),
      _discipline(RaceDiscipline::ALL_IN_ONE_DRAG),
      _lastAbortReason(RaceAbortReason::NONE),
      _liveSpeedKmh(0.0f),
      _liveDistanceM(0.0f),
      _liveGLong(0.0f),
      _liveSlopePct(0.0f),
      _launchTimeUs(0),
      _rolloutCrossingUs(0),
      _rolloutOffsetUs(0),
      _rolloutPassed(false),
      _hasPrevEpoch(false),
      _prevGpsTowMs(0),
      _prevEpochArrivalUs(0),
      _prevEpochSpeedMs(0.0f),
      _prevEpochSpeedKmh(0.0f),
      _prevEpochDistM(0.0f),
      _prevEpochAltM(0.0f),
      _lastGpsArrivalUs(0),
      _lastGpsTowMs(0),
      _lastGpsSequence(0),
      _timeAt100KmhSec(0.0f),
      _timeAt200KmhSec(0.0f),
      _passed100Kmh(false),
      _passed200Kmh(false),
      _splitTriggered(false),
      _brakeActive(false),
      _brakeStartDistM(0.0f),
      _brakeStartTimeUs(0),
      _splitQueueHead(0),
      _splitQueueTail(0),
      _hasCompletedRun(false)
{
    memset(&_settings, 0, sizeof(_settings));
    _settings.use1FootRollout = true;
    _settings.metricUnits = true;
    _settings.slopeTolerancePct = MAX_VALID_SLOPE_PCT;

    memset(&_currentRun, 0, sizeof(RunRecord));
    memset(&_lastRun, 0, sizeof(RunRecord));
    memset(_splitEventQueue, 0, sizeof(_splitEventQueue));
}

void TelemetryEngine::begin(const DeviceSettings& settings) {
    _settings = settings;
    reset(RaceAbortReason::NONE);
}

void TelemetryEngine::updateSettings(const DeviceSettings& settings) {
    _settings = settings;
}

void TelemetryEngine::setDiscipline(RaceDiscipline discipline) {
    _discipline = discipline;
    reset(RaceAbortReason::USER_RESET);
}

void TelemetryEngine::arm(uint64_t currentUs) {
    _state = RaceState::ARMED;
    _lastAbortReason = RaceAbortReason::NONE;
    _rolloutPassed = !_settings.use1FootRollout;
    _rolloutOffsetUs = 0;
    _rolloutCrossingUs = 0;
    _liveDistanceM = 0.0f;
    _liveSpeedKmh = 0.0f;
    _liveGLong = 0.0f;
    _liveSlopePct = 0.0f;
    _hasPrevEpoch = false;
    _timeAt100KmhSec = 0.0f;
    _timeAt200KmhSec = 0.0f;
    _passed100Kmh = false;
    _passed200Kmh = false;
    _brakeActive = false;
    _splitTriggered = false;
    _splitQueueHead = 0;
    _splitQueueTail = 0;
    _hasCompletedRun = false;

    memset(&_currentRun, 0, sizeof(RunRecord));
    _currentRun.discipline = _discipline;
    _currentRun.rolloutUsed = _settings.use1FootRollout;
}

void TelemetryEngine::reset(RaceAbortReason reason) {
    _state = RaceState::IDLE_WAIT_STOP;
    _lastAbortReason = reason;
    _liveDistanceM = 0.0f;
    _liveSpeedKmh = 0.0f;
    _liveGLong = 0.0f;
    _liveSlopePct = 0.0f;
    _hasPrevEpoch = false;
    _brakeActive = false;
    _splitTriggered = false;
    _splitQueueHead = 0;
    _splitQueueTail = 0;
    _hasCompletedRun = false;

    memset(&_currentRun, 0, sizeof(RunRecord));
    _currentRun.discipline = _discipline;
}

float TelemetryEngine::_calcElapsedSec(uint64_t timestampUs) const {
    if (_launchTimeUs == 0 || timestampUs < _launchTimeUs) {
        return 0.0f;
    }
    uint64_t fromLaunchUs = timestampUs - _launchTimeUs;
    if (_settings.use1FootRollout && _rolloutPassed) {
        if (fromLaunchUs <= _rolloutOffsetUs) return 0.0f;
        return (float)(fromLaunchUs - _rolloutOffsetUs) / 1000000.0f;
    }
    return (float)fromLaunchUs / 1000000.0f;
}

float TelemetryEngine::getCurrentTimeSec(uint64_t currentUs) const {
    if (_state != RaceState::MEASURING && _state != RaceState::LAUNCH_DETECTED) {
        return 0.0f;
    }
    if (currentUs == 0) {
        currentUs = (uint64_t)micros();
    }
    return _calcElapsedSec(currentUs);
}

void TelemetryEngine::_pushSplitEvent(SplitType type, const char* name, float timeSec, float speedKmh) {
    uint8_t nextHead = (_splitQueueHead + 1) % MAX_SPLIT_EVENTS;
    if (nextHead == _splitQueueTail) {
        // Очередь переполнена, вытесняем старейшее
        _splitQueueTail = (_splitQueueTail + 1) % MAX_SPLIT_EVENTS;
    }
    SplitEvent& evt = _splitEventQueue[_splitQueueHead];
    evt.type = type;
    evt.timeSec = timeSec;
    evt.trapSpeedKmh = speedKmh;
    evt.runId = _currentRun.id;
    strncpy(evt.name, name, sizeof(evt.name) - 1);
    evt.name[sizeof(evt.name) - 1] = '\0';
    _splitQueueHead = nextHead;
    _splitTriggered = true;
}

bool TelemetryEngine::popSplitEvent(SplitEvent& event) {
    if (_splitQueueHead == _splitQueueTail) {
        return false;
    }
    event = _splitEventQueue[_splitQueueTail];
    _splitQueueTail = (_splitQueueTail + 1) % MAX_SPLIT_EVENTS;
    return true;
}

bool TelemetryEngine::popCompletedRun(RunRecord& record) {
    if (_hasCompletedRun) {
        record = _lastRun;
        _hasCompletedRun = false;
        return true;
    }
    return false;
}

bool TelemetryEngine::checkAndClearSplitTrigger() {
    if (_splitTriggered) {
        _splitTriggered = false;
        return true;
    }
    return false;
}

void TelemetryEngine::_handleLaunch(uint64_t launchUs, float startAltM, uint32_t epochSec) {
    _launchTimeUs = launchUs;
    _rolloutOffsetUs = 0;
    _rolloutCrossingUs = 0;
    _rolloutPassed = !_settings.use1FootRollout;

    _currentRun.timestampUtc = epochSec;
    _currentRun.startAltM = startAltM;
    _currentRun.finishAltM = startAltM;
    _currentRun.discipline = _discipline;
    _currentRun.rolloutUsed = _settings.use1FootRollout;
    _liveDistanceM = 0.0f;
}

void TelemetryEngine::processImuSample(const ImuSample& imu) {
    _liveGLong = imu.data.gLongitudinal;

    // Взведен: детект старта по перегрузке (>0.15G)
    if (_state == RaceState::ARMED) {
        if (imu.data.isLaunchDetected()) {
            _handleLaunch(imu.sampleUs, _prevEpochAltM, 0);
            _state = _settings.use1FootRollout ? RaceState::LAUNCH_DETECTED : RaceState::MEASURING;
            _currentRun.maxAccelG = imu.data.gLongitudinal;
        }
    } else if (_state == RaceState::LAUNCH_DETECTED || _state == RaceState::MEASURING) {
        if (imu.data.gLongitudinal > _currentRun.maxAccelG) {
            _currentRun.maxAccelG = imu.data.gLongitudinal;
        }
        // Контроль таймаута GPS по таймеру выборок IMU (200 Гц)
        checkStaleTimeout(imu.sampleUs);
    }
}

void TelemetryEngine::processGpsEpoch(const GpsEpoch& epoch) {
    uint64_t arrivalUs = (epoch.arrivalUs > 0) ? epoch.arrivalUs : (uint64_t)micros();

    // Дедупликация: игнорируем повторный разбор одного и того же пакета
    if (_hasPrevEpoch && epoch.towMs != 0 && epoch.towMs == _lastGpsTowMs && epoch.sequence == _lastGpsSequence) {
        return;
    }

    _lastGpsArrivalUs = arrivalUs;
    _lastGpsTowMs = epoch.towMs;
    _lastGpsSequence = epoch.sequence;
    _liveSpeedKmh = epoch.data.speedKmh;

    // Режим ARMED: детект старта по скорости (>0.8 км/ч)
    if (_state == RaceState::ARMED) {
        if (epoch.data.speedKmh >= LAUNCH_SPEED_KMH) {
            _handleLaunch(arrivalUs, epoch.data.altMSL, epoch.data.epochSeconds);
            _state = _settings.use1FootRollout ? RaceState::LAUNCH_DETECTED : RaceState::MEASURING;
            _currentRun.maxSpeedKmh = epoch.data.speedKmh;
        }
        _prevEpochArrivalUs = arrivalUs;
        _prevEpochSpeedMs = epoch.data.speedMs;
        _prevEpochSpeedKmh = epoch.data.speedKmh;
        _prevEpochDistM = 0.0f;
        _prevEpochAltM = epoch.data.altMSL;
        _hasPrevEpoch = true;
        return;
    }

    // Режимы LAUNCH_DETECTED и MEASURING
    if (_state == RaceState::LAUNCH_DETECTED || _state == RaceState::MEASURING) {
        // Контроль качества спутникового решения во время заезда
        if (!epoch.data.validFix || epoch.data.fixType < 3) {
            _finalizeRun(RaceState::ABORTED, RaceAbortReason::GPS_FIX_LOST, _calcElapsedSec(arrivalUs), epoch.data.altMSL);
            return;
        }
        if (epoch.data.sAccKmh > GPS_MAX_SACC_KMH) {
            _finalizeRun(RaceState::ABORTED, RaceAbortReason::GPS_ACCURACY_BAD, _calcElapsedSec(arrivalUs), epoch.data.altMSL);
            return;
        }

        if (epoch.data.speedKmh > _currentRun.maxSpeedKmh) {
            _currentRun.maxSpeedKmh = epoch.data.speedKmh;
        }

        if (!_hasPrevEpoch) {
            _prevEpochArrivalUs = arrivalUs;
            _prevEpochSpeedMs = epoch.data.speedMs;
            _prevEpochSpeedKmh = epoch.data.speedKmh;
            _prevEpochDistM = _liveDistanceM;
            _prevEpochAltM = epoch.data.altMSL;
            _hasPrevEpoch = true;
            return;
        }

        float dtSec = (float)(arrivalUs - _prevEpochArrivalUs) / 1000000.0f;
        if (dtSec <= 0.0001f || dtSec > 1.0f) {
            // Аномальный или отрицательный скачок времени
            return;
        }

        // 1. Непрерывное интегрирование дистанции методом трапеций
        float avgSpeedMs = (epoch.data.speedMs + _prevEpochSpeedMs) * 0.5f;
        float s1 = _prevEpochDistM;
        _liveDistanceM += avgSpeedMs * dtSec;
        float s2 = _liveDistanceM;

        // 2. Детект пересечения 1-Foot Rollout (0.3048 м) БЕЗ сброса интегратора дистанции
        if (_settings.use1FootRollout && !_rolloutPassed) {
            if (_liveDistanceM >= ONE_FOOT_METERS) {
                float deltaS = s2 - s1;
                float factor = (deltaS > 0.0001f) ? ((ONE_FOOT_METERS - s1) / deltaS) : 1.0f;
                factor = fmaxf(0.0f, fminf(1.0f, factor));
                _rolloutCrossingUs = _prevEpochArrivalUs + (uint64_t)(factor * (arrivalUs - _prevEpochArrivalUs));
                _rolloutOffsetUs = (_rolloutCrossingUs > _launchTimeUs) ? (_rolloutCrossingUs - _launchTimeUs) : 0;
                _rolloutPassed = true;
                _state = RaceState::MEASURING;
            }
        } else if (!_settings.use1FootRollout && !_rolloutPassed) {
            _rolloutPassed = true;
            _rolloutOffsetUs = 0;
            _state = RaceState::MEASURING;
        }

        // Вычисление меток времени для интерполяции отсечек
        float t1 = _calcElapsedSec(_prevEpochArrivalUs);
        float t2 = _calcElapsedSec(arrivalUs);
        float v1Kmh = _prevEpochSpeedKmh;
        float v2Kmh = epoch.data.speedKmh;
        float v1Ms = _prevEpochSpeedMs;
        float v2Ms = epoch.data.speedMs;
        float h1 = _prevEpochAltM;
        float h2 = epoch.data.altMSL;

        // 3. Проверка скоростных и дистанционных отсечек
        _checkSpeedSplits(epoch, t1, v1Kmh, t2, v2Kmh);
        _checkDistanceSplits(epoch, t1, v1Ms, s1, h1, t2, v2Ms, s2, h2);

        // 4. Расчет живого уклона трассы
        if (_liveDistanceM > 5.0f) {
            _liveSlopePct = ((epoch.data.altMSL - _currentRun.startAltM) / _liveDistanceM) * 100.0f;
        }

        // 5. Проверка выполнения условий финиша дисциплины
        _checkRunCompletion();

        // Сохранение текущей эпохи как предыдущей
        _prevEpochArrivalUs = arrivalUs;
        _prevEpochSpeedMs = epoch.data.speedMs;
        _prevEpochSpeedKmh = epoch.data.speedKmh;
        _prevEpochDistM = _liveDistanceM;
        _prevEpochAltM = epoch.data.altMSL;
        _prevGpsTowMs = epoch.towMs;
    }
}

void TelemetryEngine::checkStaleTimeout(uint64_t currentUs) {
    if (_state != RaceState::MEASURING && _state != RaceState::LAUNCH_DETECTED) {
        return;
    }
    if (_lastGpsArrivalUs > 0) {
        uint64_t diffUs = currentUs - _lastGpsArrivalUs;
        if (diffUs > ((uint64_t)GPS_STALE_TIMEOUT_MS * 1000ULL)) {
            _finalizeRun(RaceState::ABORTED, RaceAbortReason::GPS_STALE, _calcElapsedSec(_lastGpsArrivalUs), _prevEpochAltM);
        }
    }
}

void TelemetryEngine::process(const GpsData& gps, const ImuData& imu, uint64_t currentUs) {
    if (currentUs == 0) currentUs = (uint64_t)micros();

    ImuSample imuSample;
    imuSample.data = imu;
    imuSample.sampleUs = currentUs;
    processImuSample(imuSample);

    GpsEpoch epoch;
    epoch.data = gps;
    epoch.towMs = gps.towMs;
    epoch.arrivalUs = currentUs;
    epoch.sequence = 0;
    processGpsEpoch(epoch);
}

void TelemetryEngine::_interpolateSpeedSplit(float targetKmh, SplitType type, const char* name, SplitTime& split, 
                                             float t1, float v1, float t2, float v2) {
    if (v2 <= v1) {
        split.timeSec = t2;
        split.trapSpeedKmh = targetKmh;
        split.achieved = true;
        _pushSplitEvent(type, name, split.timeSec, split.trapSpeedKmh);
        return;
    }

    float factor = (targetKmh - v1) / (v2 - v1);
    factor = fmaxf(0.0f, fminf(1.0f, factor));
    
    split.timeSec = t1 + factor * (t2 - t1);
    split.trapSpeedKmh = targetKmh;
    split.achieved = true;
    _pushSplitEvent(type, name, split.timeSec, split.trapSpeedKmh);
}

void TelemetryEngine::_interpolateDistanceSplit(float targetM, SplitType type, const char* name, SplitTime& split, 
                                                 float t1, float v1Ms, float s1, float h1, 
                                                 float t2, float v2Ms, float s2, float h2) {
    float deltaS = targetM - s1;
    float dt = t2 - t1;

    if (dt <= 0.0001f || (s2 <= s1)) {
        split.timeSec = t2;
        split.trapSpeedKmh = v2Ms * 3.6f;
        split.achieved = true;
        _pushSplitEvent(type, name, split.timeSec, split.trapSpeedKmh);
        return;
    }

    float accel = (v2Ms - v1Ms) / dt;

    if (fabsf(accel) < 0.001f) {
        float tau = (v1Ms > 0.1f) ? (deltaS / v1Ms) : 0.0f;
        split.timeSec = t1 + tau;
        split.trapSpeedKmh = v1Ms * 3.6f;
    } else {
        float discriminant = v1Ms * v1Ms + 2.0f * accel * deltaS;
        if (discriminant >= 0.0f) {
            float tau = (-v1Ms + sqrtf(discriminant)) / accel;
            if (tau >= 0.0f && tau <= dt) {
                split.timeSec = t1 + tau;
                float trapSpeedMs = v1Ms + accel * tau;
                split.trapSpeedKmh = trapSpeedMs * 3.6f;
            } else {
                split.timeSec = t2;
                split.trapSpeedKmh = v2Ms * 3.6f;
            }
        } else {
            split.timeSec = t2;
            split.trapSpeedKmh = v2Ms * 3.6f;
        }
    }

    split.achieved = true;
    _pushSplitEvent(type, name, split.timeSec, split.trapSpeedKmh);
}

void TelemetryEngine::_checkSpeedSplits(const GpsEpoch& epoch, float t1, float v1, float t2, float v2) {
    // 0 - 60 км/ч
    if (!_currentRun.split0_60.achieved && v2 >= 60.0f) {
        _interpolateSpeedSplit(60.0f, SplitType::SPLIT_0_60, "0-60", _currentRun.split0_60, t1, v1, t2, v2);
    }

    // 0 - 100 км/ч
    if (!_currentRun.split0_100.achieved && v2 >= 100.0f) {
        _interpolateSpeedSplit(100.0f, SplitType::SPLIT_0_100, "0-100", _currentRun.split0_100, t1, v1, t2, v2);
        _timeAt100KmhSec = _currentRun.split0_100.timeSec;
        _passed100Kmh = true;
    }

    // 100 - 150 км/ч
    if (!_currentRun.split100_150.achieved && v2 >= 150.0f && _passed100Kmh) {
        SplitTime split150;
        _interpolateSpeedSplit(150.0f, SplitType::SPLIT_100_150, "100-150", split150, t1, v1, t2, v2);
        _currentRun.split100_150.timeSec = split150.timeSec - _timeAt100KmhSec;
        _currentRun.split100_150.trapSpeedKmh = 150.0f;
        _currentRun.split100_150.achieved = true;
    }

    // 0 - 200 км/ч и 100 - 200 км/ч
    if (!_currentRun.split0_200.achieved && v2 >= 200.0f) {
        _interpolateSpeedSplit(200.0f, SplitType::SPLIT_0_200, "0-200", _currentRun.split0_200, t1, v1, t2, v2);
        _timeAt200KmhSec = _currentRun.split0_200.timeSec;
        _passed200Kmh = true;

        if (_passed100Kmh && !_currentRun.split100_200.achieved) {
            _currentRun.split100_200.timeSec = _currentRun.split0_200.timeSec - _timeAt100KmhSec;
            _currentRun.split100_200.trapSpeedKmh = 200.0f;
            _currentRun.split100_200.achieved = true;
            _pushSplitEvent(SplitType::SPLIT_100_200, "100-200", _currentRun.split100_200.timeSec, 200.0f);
        }
    }

    // 200 - 300 км/ч
    if (!_currentRun.split200_300.achieved && v2 >= 300.0f && _passed200Kmh) {
        SplitTime split300;
        _interpolateSpeedSplit(300.0f, SplitType::SPLIT_200_300, "200-300", split300, t1, v1, t2, v2);
        _currentRun.split200_300.timeSec = split300.timeSec - _timeAt200KmhSec;
        _currentRun.split200_300.trapSpeedKmh = 300.0f;
        _currentRun.split200_300.achieved = true;
    }

    // 60 - 120 км/ч
    if (!_currentRun.split60_120.achieved && v2 >= 120.0f && _currentRun.split0_60.achieved) {
        SplitTime split120;
        _interpolateSpeedSplit(120.0f, SplitType::SPLIT_60_120, "60-120", split120, t1, v1, t2, v2);
        _currentRun.split60_120.timeSec = split120.timeSec - _currentRun.split0_60.timeSec;
        _currentRun.split60_120.trapSpeedKmh = 120.0f;
        _currentRun.split60_120.achieved = true;
    }

    // 80 - 120 км/ч
    if (!_currentRun.split80_120.achieved && v2 >= 120.0f && v1 <= 80.0f) {
        SplitTime split80, split120;
        _interpolateSpeedSplit(80.0f, SplitType::SPLIT_80_120, "80", split80, t1, v1, t2, v2);
        _interpolateSpeedSplit(120.0f, SplitType::SPLIT_80_120, "80-120", split120, t1, v1, t2, v2);
        _currentRun.split80_120.timeSec = split120.timeSec - split80.timeSec;
        _currentRun.split80_120.trapSpeedKmh = 120.0f;
        _currentRun.split80_120.achieved = true;
    }
}

void TelemetryEngine::_checkDistanceSplits(const GpsEpoch& epoch, float t1, float v1Ms, float s1, float h1, 
                                           float t2, float v2Ms, float s2, float h2) {
    // 60 футов (18.288 м)
    if (!_currentRun.split60ft.achieved && s2 >= 18.288f) {
        _interpolateDistanceSplit(18.288f, SplitType::SPLIT_60FT, "60ft", _currentRun.split60ft, t1, v1Ms, s1, h1, t2, v2Ms, s2, h2);
    }

    // 330 футов (100.584 м)
    if (!_currentRun.split330ft.achieved && s2 >= 100.584f) {
        _interpolateDistanceSplit(100.584f, SplitType::SPLIT_330FT, "330ft", _currentRun.split330ft, t1, v1Ms, s1, h1, t2, v2Ms, s2, h2);
    }

    // 1/8 мили (201.168 м)
    if (!_currentRun.split1_8mi.achieved && s2 >= 201.168f) {
        _interpolateDistanceSplit(201.168f, SplitType::SPLIT_1_8MI, "1/8mi", _currentRun.split1_8mi, t1, v1Ms, s1, h1, t2, v2Ms, s2, h2);
    }

    // 1000 футов (304.800 м)
    if (!_currentRun.split1000ft.achieved && s2 >= 304.800f) {
        _interpolateDistanceSplit(304.800f, SplitType::SPLIT_1000FT, "1000ft", _currentRun.split1000ft, t1, v1Ms, s1, h1, t2, v2Ms, s2, h2);
    }

    // 1/4 мили (402.336 м)
    if (!_currentRun.split1_4mi.achieved && s2 >= 402.336f) {
        _interpolateDistanceSplit(402.336f, SplitType::SPLIT_1_4MI, "1/4mi", _currentRun.split1_4mi, t1, v1Ms, s1, h1, t2, v2Ms, s2, h2);
    }

    // 1/2 мили (804.672 м)
    if (!_currentRun.split1_2mi.achieved && s2 >= 804.672f) {
        _interpolateDistanceSplit(804.672f, SplitType::SPLIT_1_2MI, "1/2mi", _currentRun.split1_2mi, t1, v1Ms, s1, h1, t2, v2Ms, s2, h2);
    }

    // 1 миля (1609.344 м)
    if (!_currentRun.split1mi.achieved && s2 >= 1609.344f) {
        _interpolateDistanceSplit(1609.344f, SplitType::SPLIT_1MI, "1mi", _currentRun.split1mi, t1, v1Ms, s1, h1, t2, v2Ms, s2, h2);
    }

    // Торможение 100 - 0 км/ч
    if (_discipline == RaceDiscipline::BRAKE_100_0) {
        if (!_brakeActive && epoch.data.speedKmh >= 100.0f) {
            _brakeActive = true;
            _brakeStartTimeUs = epoch.arrivalUs;
            _brakeStartDistM = _liveDistanceM;
        } else if (_brakeActive && epoch.data.speedKmh <= 0.8f) {
            float brakeTimeSec = (float)(epoch.arrivalUs - _brakeStartTimeUs) / 1000000.0f;
            _currentRun.split100_0.timeSec = brakeTimeSec;
            _currentRun.brakeDist100_0M = _liveDistanceM - _brakeStartDistM;
            _currentRun.split100_0.achieved = true;
            _pushSplitEvent(SplitType::SPLIT_BRAKE_100_0, "100-0", brakeTimeSec, 0.0f);
            _finalizeRun(RaceState::FINISHED, RaceAbortReason::NONE, brakeTimeSec, epoch.data.altMSL);
        }
    }
}

void TelemetryEngine::_checkRunCompletion() {
    bool completed = false;
    float finishTime = 0.0f;
    float finishAlt = _prevEpochAltM;

    switch (_discipline) {
        case RaceDiscipline::ALL_IN_ONE_DRAG:
            if (_currentRun.split1_4mi.achieved) {
                completed = true;
                finishTime = _currentRun.split1_4mi.timeSec;
            } else if (_currentRun.split0_200.achieved) {
                completed = true;
                finishTime = _currentRun.split0_200.timeSec;
            } else if (_currentRun.split0_100.achieved && (_liveSpeedKmh < (_currentRun.maxSpeedKmh - 6.0f) || _liveSpeedKmh < 60.0f)) {
                completed = true;
                finishTime = _currentRun.split0_100.timeSec;
            } else if (_currentRun.split0_60.achieved && (_liveSpeedKmh < (_currentRun.maxSpeedKmh - 6.0f) || _liveSpeedKmh < 30.0f)) {
                completed = true;
                finishTime = _currentRun.split0_60.timeSec;
            } else if (_liveSpeedKmh <= STOP_SPEED_THRESHOLD && _currentRun.maxSpeedKmh >= 20.0f) {
                completed = true;
                finishTime = getCurrentTimeSec(_lastGpsArrivalUs);
            }
            break;

        case RaceDiscipline::SPEED_0_60:
            if (_currentRun.split0_60.achieved) {
                completed = true;
                finishTime = _currentRun.split0_60.timeSec;
            }
            break;

        case RaceDiscipline::SPEED_0_100:
            if (_currentRun.split0_100.achieved) {
                completed = true;
                finishTime = _currentRun.split0_100.timeSec;
            }
            break;

        case RaceDiscipline::SPEED_100_200:
            if (_currentRun.split100_200.achieved) {
                completed = true;
                finishTime = _currentRun.split100_200.timeSec;
            }
            break;

        case RaceDiscipline::SPEED_0_200:
            if (_currentRun.split0_200.achieved) {
                completed = true;
                finishTime = _currentRun.split0_200.timeSec;
            }
            break;

        case RaceDiscipline::QUARTER_MILE_402M:
            if (_currentRun.split1_4mi.achieved) {
                completed = true;
                finishTime = _currentRun.split1_4mi.timeSec;
            }
            break;

        case RaceDiscipline::HALF_MILE_804M:
            if (_currentRun.split1_2mi.achieved) {
                completed = true;
                finishTime = _currentRun.split1_2mi.timeSec;
            }
            break;

        default:
            break;
    }

    if (completed) {
        _finalizeRun(RaceState::FINISHED, RaceAbortReason::NONE, finishTime, finishAlt);
    }
}

void TelemetryEngine::_finalizeRun(RaceState finalState, RaceAbortReason abortReason, float finishTimeSec, float finishAltM) {
    if (finishTimeSec <= 0.0001f) {
        finishTimeSec = getCurrentTimeSec(_lastGpsArrivalUs);
    }

    _currentRun.totalDurationSec = finishTimeSec;
    _currentRun.totalDistanceM = _liveDistanceM;
    _currentRun.finishAltM = finishAltM;
    _currentRun.abortReason = abortReason;

    // Расчет уклона трассы
    if (_currentRun.totalDistanceM > 10.0f && isfinite(_currentRun.finishAltM) && isfinite(_currentRun.startAltM)) {
        _currentRun.slopePct = ((_currentRun.finishAltM - _currentRun.startAltM) / _currentRun.totalDistanceM) * 100.0f;
    } else {
        _currentRun.slopePct = 0.0f;
    }

    if (!isfinite(_currentRun.slopePct) || fabsf(_currentRun.slopePct) > 50.0f) {
        _currentRun.slopePct = 0.0f;
    }

    _currentRun.isValidSlope = (_currentRun.slopePct >= _settings.slopeTolerancePct);

    // Сохранение итогового результата
    _lastRun = _currentRun;
    _hasCompletedRun = (finalState == RaceState::FINISHED);
    _state = finalState;
    _lastAbortReason = abortReason;
}

