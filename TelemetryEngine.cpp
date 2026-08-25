#include "TelemetryEngine.h"
#include <math.h>

TelemetryEngine::TelemetryEngine()
    : _state(RaceState::IDLE_WAIT_STOP),
      _discipline(RaceDiscipline::ALL_IN_ONE_DRAG),
      _liveSpeedKmh(0.0f),
      _liveDistanceM(0.0f),
      _liveGLong(0.0f),
      _liveSlopePct(0.0f),
      _stopStartMs(0),
      _launchTimeUs(0),
      _rolloutTimeUs(0),
      _rolloutPassed(false),
      _prevSpeedMs(0.0f),
      _prevSpeedKmh(0.0f),
      _prevDistM(0.0f),
      _prevTimeSec(0.0f),
      _prevAltM(0.0f),
      _hasPrevSample(false),
      _timeAt100KmhSec(0.0f),
      _passed100Kmh(false),
      _passed200Kmh(false),
      _brakeActive(false),
      _brakeStartDistM(0.0f),
      _brakeStartTimeUs(0)
{
    memset(&_currentRun, 0, sizeof(RunRecord));
    memset(&_lastRun, 0, sizeof(RunRecord));
    _settings.use1FootRollout = true;
    _settings.metricUnits = true;
    _settings.slopeTolerancePct = MAX_VALID_SLOPE_PCT;
}

void TelemetryEngine::begin(const DeviceSettings& settings) {
    _settings = settings;
    reset();
}

void TelemetryEngine::updateSettings(const DeviceSettings& settings) {
    _settings = settings;
}

void TelemetryEngine::setDiscipline(RaceDiscipline discipline) {
    _discipline = discipline;
    reset();
}

void TelemetryEngine::arm() {
    _state = RaceState::ARMED;
    _rolloutPassed = !_settings.use1FootRollout;
    _liveDistanceM = 0.0f;
    _liveSpeedKmh = 0.0f;
    _hasPrevSample = false;
    memset(&_currentRun, 0, sizeof(RunRecord));
    _currentRun.discipline = _discipline;
    _currentRun.rolloutUsed = _settings.use1FootRollout;
    _passed100Kmh = false;
    _passed200Kmh = false;
}

void TelemetryEngine::reset() {
    _state = RaceState::IDLE_WAIT_STOP;
    _stopStartMs = millis();
    _liveDistanceM = 0.0f;
    _liveSpeedKmh = 0.0f;
    _liveGLong = 0.0f;
    _liveSlopePct = 0.0f;
    _hasPrevSample = false;
    _brakeActive = false;
    memset(&_currentRun, 0, sizeof(RunRecord));
}

float TelemetryEngine::getCurrentTimeSec() const {
    if (_state != RaceState::MEASURING && _state != RaceState::LAUNCH_DETECTED) {
        return 0.0f;
    }
    uint32_t refTime = (_settings.use1FootRollout && _rolloutPassed) ? _rolloutTimeUs : _launchTimeUs;
    if (refTime == 0) return 0.0f;
    return (float)(micros() - refTime) / 1000000.0f;
}

void TelemetryEngine::process(const GpsData& gps, const ImuData& imu) {
    _liveSpeedKmh = gps.speedKmh;
    _liveGLong = imu.gLongitudinal;

    _handleStateTransitions(gps, imu);

    if (_state == RaceState::MEASURING || _state == RaceState::LAUNCH_DETECTED) {
        _updateMeasuring(gps, imu);
    }
}

void TelemetryEngine::_handleStateTransitions(const GpsData& gps, const ImuData& imu) {
    uint32_t nowMs = millis();

    switch (_state) {
        case RaceState::IDLE_WAIT_STOP:
            if (gps.validFix && gps.speedKmh <= STOP_SPEED_THRESHOLD) {
                if (_stopStartMs == 0) {
                    _stopStartMs = nowMs;
                } else if ((nowMs - _stopStartMs) >= STOP_TIME_STABLE_MS) {
                    // Автомобиль неподвижен более 1.2 с — переходим в режим готовности (ARMED)
                    arm();
                }
            } else {
                _stopStartMs = 0;
            }
            break;

        case RaceState::ARMED:
            // Детекция старта: либо по рывку акселерометра (>0.15G), либо по скорости GPS (>0.8 км/ч)
            if (imu.isLaunchDetected() || gps.speedKmh >= LAUNCH_SPEED_KMH) {
                _state = RaceState::LAUNCH_DETECTED;
                _launchTimeUs = micros();
                _currentRun.timestampUtc = (uint32_t)(gps.towMs / 1000);
                _currentRun.startAltM = gps.altMSL;
                _currentRun.maxAccelG = imu.gLongitudinal;
                _currentRun.maxSpeedKmh = gps.speedKmh;
                
                _liveDistanceM = 0.0f;
                _prevSpeedMs = gps.speedMs;
                _prevSpeedKmh = gps.speedKmh;
                _prevDistM = 0.0f;
                _prevTimeSec = 0.0f;
                _prevAltM = gps.altMSL;
                _hasPrevSample = true;

                if (!_settings.use1FootRollout) {
                    _rolloutPassed = true;
                    _rolloutTimeUs = _launchTimeUs;
                    _state = RaceState::MEASURING;
                }
            }
            break;

        case RaceState::LAUNCH_DETECTED:
            // Ожидание преодоления 1 фута (0.3048 м) раската
            if (_settings.use1FootRollout && !_rolloutPassed) {
                if (_liveDistanceM >= ONE_FOOT_METERS) {
                    _rolloutPassed = true;
                    _rolloutTimeUs = micros();
                    _state = RaceState::MEASURING;
                }
            } else {
                _state = RaceState::MEASURING;
            }
            break;

        case RaceState::MEASURING:
            // Заезд завершается при достижении всех целей или при полной остановке/замедлении
            _checkRunCompletion();
            break;

        case RaceState::FINISHED:
        case RaceState::ABORTED:
            // Ожидание повторной остановки авто для нового заезда
            if (gps.speedKmh <= STOP_SPEED_THRESHOLD) {
                if (_stopStartMs == 0) _stopStartMs = nowMs;
                else if ((nowMs - _stopStartMs) >= (STOP_TIME_STABLE_MS + 1000)) {
                    arm();
                }
            } else {
                _stopStartMs = 0;
            }
            break;
    }
}

void TelemetryEngine::_updateMeasuring(const GpsData& gps, const ImuData& imu) {
    if (!_hasPrevSample) {
        _prevSpeedMs = gps.speedMs;
        _prevSpeedKmh = gps.speedKmh;
        _prevDistM = _liveDistanceM;
        _prevTimeSec = getCurrentTimeSec();
        _prevAltM = gps.altMSL;
        _hasPrevSample = true;
        return;
    }

    float currTimeSec = getCurrentTimeSec();
    float dt = currTimeSec - _prevTimeSec;
    if (dt <= 0.0001f) return;

    // Приращение дистанции по методу трапеций
    float avgSpeedMs = (gps.speedMs + _prevSpeedMs) * 0.5f;
    float deltaDistM = avgSpeedMs * dt;
    _liveDistanceM += deltaDistM;

    // Обновление пиковых параметров
    if (gps.speedKmh > _currentRun.maxSpeedKmh) {
        _currentRun.maxSpeedKmh = gps.speedKmh;
    }
    if (imu.gLongitudinal > _currentRun.maxAccelG) {
        _currentRun.maxAccelG = imu.gLongitudinal;
    }

    // Расчет текущего уклона (Slope %)
    if (_liveDistanceM > 5.0f) {
        _liveSlopePct = ((gps.altMSL - _currentRun.startAltM) / _liveDistanceM) * 100.0f;
    }

    // ------------------------------------------------------------------------
    // СУБВЫБОРОЧНАЯ ИНТЕРПОЛЯЦИЯ СКОРОСТНЫХ ОТСЕЧЕК (Hermite Cubic / Linear)
    // ------------------------------------------------------------------------
    // 0 - 60 км/ч
    if (!_currentRun.split0_60.achieved && gps.speedKmh >= 60.0f) {
        _interpolateSpeedSplit(60.0f, _currentRun.split0_60, 
                               _prevTimeSec, _prevSpeedKmh, currTimeSec, gps.speedKmh, 
                               _prevDistM, _liveDistanceM);
    }

    // 0 - 100 км/ч
    if (!_currentRun.split0_100.achieved && gps.speedKmh >= 100.0f) {
        _interpolateSpeedSplit(100.0f, _currentRun.split0_100, 
                               _prevTimeSec, _prevSpeedKmh, currTimeSec, gps.speedKmh, 
                               _prevDistM, _liveDistanceM);
        _timeAt100KmhSec = _currentRun.split0_100.timeSec;
        _passed100Kmh = true;
    }

    // 100 - 150 км/ч
    if (!_currentRun.split100_150.achieved && gps.speedKmh >= 150.0f && _passed100Kmh) {
        SplitTime split150;
        _interpolateSpeedSplit(150.0f, split150, _prevTimeSec, _prevSpeedKmh, currTimeSec, gps.speedKmh, _prevDistM, _liveDistanceM);
        _currentRun.split100_150.timeSec = split150.timeSec - _timeAt100KmhSec;
        _currentRun.split100_150.trapSpeedKmh = 150.0f;
        _currentRun.split100_150.achieved = true;
    }

    // 0 - 200 км/ч и 100 - 200 км/ч
    if (!_currentRun.split0_200.achieved && gps.speedKmh >= 200.0f) {
        _interpolateSpeedSplit(200.0f, _currentRun.split0_200, 
                               _prevTimeSec, _prevSpeedKmh, currTimeSec, gps.speedKmh, 
                               _prevDistM, _liveDistanceM);
        _timeAt200KmhSec = _currentRun.split0_200.timeSec;
        _passed200Kmh = true;

        if (_passed100Kmh && !_currentRun.split100_200.achieved) {
            _currentRun.split100_200.timeSec = _currentRun.split0_200.timeSec - _timeAt100KmhSec;
            _currentRun.split100_200.trapSpeedKmh = 200.0f;
            _currentRun.split100_200.achieved = true;
        }
    }

    // 200 - 300 км/ч
    if (!_currentRun.split200_300.achieved && gps.speedKmh >= 300.0f && _passed200Kmh) {
        SplitTime split300;
        _interpolateSpeedSplit(300.0f, split300, _prevTimeSec, _prevSpeedKmh, currTimeSec, gps.speedKmh, _prevDistM, _liveDistanceM);
        _currentRun.split200_300.timeSec = split300.timeSec - _timeAt200KmhSec;
        _currentRun.split200_300.trapSpeedKmh = 300.0f;
        _currentRun.split200_300.achieved = true;
    }

    // 60 - 120 км/ч
    if (!_currentRun.split60_120.achieved && gps.speedKmh >= 120.0f && _currentRun.split0_60.achieved) {
        SplitTime split120;
        _interpolateSpeedSplit(120.0f, split120, _prevTimeSec, _prevSpeedKmh, currTimeSec, gps.speedKmh, _prevDistM, _liveDistanceM);
        _currentRun.split60_120.timeSec = split120.timeSec - _currentRun.split0_60.timeSec;
        _currentRun.split60_120.trapSpeedKmh = 120.0f;
        _currentRun.split60_120.achieved = true;
    }

    // ------------------------------------------------------------------------
    // СУБВЫБОРОЧНАЯ КВАДРАТИЧНАЯ ИНТЕРПОЛЯЦИЯ ДИСТАНЦИОННЫХ ОТСЕЧЕК (DRAG)
    // ------------------------------------------------------------------------
    // 60 футов (18.288 м)
    if (!_currentRun.split60ft.achieved && _liveDistanceM >= 18.288f) {
        _interpolateDistanceSplit(18.288f, _currentRun.split60ft, 
                                  _prevTimeSec, _prevSpeedMs, currTimeSec, gps.speedMs, 
                                  _prevDistM, _liveDistanceM);
    }

    // 330 футов (100.584 м)
    if (!_currentRun.split330ft.achieved && _liveDistanceM >= 100.584f) {
        _interpolateDistanceSplit(100.584f, _currentRun.split330ft, 
                                  _prevTimeSec, _prevSpeedMs, currTimeSec, gps.speedMs, 
                                  _prevDistM, _liveDistanceM);
    }

    // 1/8 мили (201.168 м)
    if (!_currentRun.split1_8mi.achieved && _liveDistanceM >= 201.168f) {
        _interpolateDistanceSplit(201.168f, _currentRun.split1_8mi, 
                                  _prevTimeSec, _prevSpeedMs, currTimeSec, gps.speedMs, 
                                  _prevDistM, _liveDistanceM);
    }

    // 1000 футов (304.800 м)
    if (!_currentRun.split1000ft.achieved && _liveDistanceM >= 304.800f) {
        _interpolateDistanceSplit(304.800f, _currentRun.split1000ft, 
                                  _prevTimeSec, _prevSpeedMs, currTimeSec, gps.speedMs, 
                                  _prevDistM, _liveDistanceM);
    }

    // 1/4 мили (402.336 м) — Главная драг-отсечка!
    if (!_currentRun.split1_4mi.achieved && _liveDistanceM >= 402.336f) {
        _interpolateDistanceSplit(402.336f, _currentRun.split1_4mi, 
                                  _prevTimeSec, _prevSpeedMs, currTimeSec, gps.speedMs, 
                                  _prevDistM, _liveDistanceM);
    }

    // 1/2 мили (804.672 м)
    if (!_currentRun.split1_2mi.achieved && _liveDistanceM >= 804.672f) {
        _interpolateDistanceSplit(804.672f, _currentRun.split1_2mi, 
                                  _prevTimeSec, _prevSpeedMs, currTimeSec, gps.speedMs, 
                                  _prevDistM, _liveDistanceM);
    }

    // 1 миля (1609.344 м)
    if (!_currentRun.split1mi.achieved && _liveDistanceM >= 1609.344f) {
        _interpolateDistanceSplit(1609.344f, _currentRun.split1mi, 
                                  _prevTimeSec, _prevSpeedMs, currTimeSec, gps.speedMs, 
                                  _prevDistM, _liveDistanceM);
    }

    // Торможение 100 - 0 км/ч
    if (_discipline == RaceDiscipline::BRAKE_100_0) {
        if (!_brakeActive && gps.speedKmh >= 100.0f) {
            _brakeActive = true;
            _brakeStartTimeUs = micros();
            _brakeStartDistM = _liveDistanceM;
        } else if (_brakeActive && gps.speedKmh <= 0.8f) {
            _currentRun.split100_0.timeSec = (float)(micros() - _brakeStartTimeUs) / 1000000.0f;
            _currentRun.brakeDist100_0M = _liveDistanceM - _brakeStartDistM;
            _currentRun.split100_0.achieved = true;
            _finalizeRun();
            return;
        }
    }

    // Сохраняем предыдущую точку для следующего шага интерполяции
    _prevTimeSec = currTimeSec;
    _prevSpeedMs = gps.speedMs;
    _prevSpeedKmh = gps.speedKmh;
    _prevDistM = _liveDistanceM;
    _prevAltM = gps.altMSL;
}

void TelemetryEngine::_interpolateSpeedSplit(float targetKmh, SplitTime& split, 
                                             float t1, float v1, float t2, float v2, 
                                             float s1, float s2) {
    if (v2 <= v1) {
        split.timeSec = t2;
        split.trapSpeedKmh = targetKmh;
        split.achieved = true;
        return;
    }

    // Точная субмиллисекундная интерполяция
    float factor = (targetKmh - v1) / (v2 - v1);
    factor = fmaxf(0.0f, fminf(1.0f, factor));
    
    split.timeSec = t1 + factor * (t2 - t1);
    split.trapSpeedKmh = targetKmh;
    split.achieved = true;
    _splitTriggered = true;
}

void TelemetryEngine::_interpolateDistanceSplit(float targetM, SplitTime& split, 
                                                float t1, float v1Ms, float t2, float v2Ms, 
                                                float s1, float s2) {
    float deltaS = targetM - s1;
    float dt = t2 - t1;

    if (dt <= 0.0001f || (s2 <= s1)) {
        split.timeSec = t2;
        split.trapSpeedKmh = v2Ms * 3.6f;
        split.achieved = true;
        _splitTriggered = true;
        return;
    }

    float accel = (v2Ms - v1Ms) / dt;

    if (fabsf(accel) < 0.001f) {
        // Равномерное движение
        float tau = (v1Ms > 0.1f) ? (deltaS / v1Ms) : 0.0f;
        split.timeSec = t1 + tau;
        split.trapSpeedKmh = v1Ms * 3.6f;
    } else {
        // Равноускоренное движение: s = v1*t + 0.5*a*t^2 -> 0.5*a*t^2 + v1*t - deltaS = 0
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
    _splitTriggered = true;
}

bool TelemetryEngine::checkAndClearSplitTrigger() {
    if (_splitTriggered) {
        _splitTriggered = false;
        return true;
    }
    return false;
}

void TelemetryEngine::_checkRunCompletion() {
    bool completed = false;

    switch (_discipline) {
        case RaceDiscipline::ALL_IN_ONE_DRAG:
            // В универсальном режиме заезд считается завершенным при проезде 1/4 мили (402м) 
            // или достижении 200 км/ч, либо если пилот сбросил газ (скорость упала ниже 40 км/ч после разгона)
            if (_currentRun.split1_4mi.achieved || _currentRun.split0_200.achieved) {
                completed = true;
            } else if (_currentRun.split0_100.achieved && _liveSpeedKmh < 40.0f) {
                completed = true;
            }
            break;

        case RaceDiscipline::SPEED_0_100:
            if (_currentRun.split0_100.achieved) completed = true;
            break;

        case RaceDiscipline::SPEED_100_200:
            if (_currentRun.split100_200.achieved) completed = true;
            break;

        case RaceDiscipline::SPEED_0_200:
            if (_currentRun.split0_200.achieved) completed = true;
            break;

        case RaceDiscipline::QUARTER_MILE_402M:
            if (_currentRun.split1_4mi.achieved) completed = true;
            break;

        case RaceDiscipline::HALF_MILE_804M:
            if (_currentRun.split1_2mi.achieved) completed = true;
            break;

        default:
            break;
    }

    if (completed) {
        _finalizeRun();
    }
}

void TelemetryEngine::_finalizeRun() {
    _state = RaceState::FINISHED;
    _currentRun.finishAltM = _prevAltM;
    _currentRun.totalDistanceM = _liveDistanceM;
    _currentRun.totalDurationSec = getCurrentTimeSec();

    // Расчет уклона трассы за заезд
    if (_currentRun.totalDistanceM > 10.0f) {
        _currentRun.slopePct = ((_currentRun.finishAltM - _currentRun.startAltM) / _currentRun.totalDistanceM) * 100.0f;
    } else {
        _currentRun.slopePct = 0.0f;
    }

    // Проверка валидности уклона по регламенту Dragy / NHRA (уклон вниз не более 1.0%)
    _currentRun.isValidSlope = (_currentRun.slopePct >= _settings.slopeTolerancePct);

    // Сохраняем как последний результат
    _lastRun = _currentRun;
    _stopStartMs = 0;
}
