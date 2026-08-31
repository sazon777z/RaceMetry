#include "LedController.h"

LedController::LedController()
    : _pin(PIN_WS2812),
      _mode(LedMode::GPS_SEARCH),
      _lastUpdateMs(0),
      _animStep(0),
      _curR(0), _curG(0), _curB(0),
      _splitFlashUntilMs(0),
      _fixAcquiredUntilMs(0),
      _batteryAnimActive(false),
      _batteryAnimUntilMs(0),
      _batteryPct(100),
      _batteryMode(0)
{
}

void LedController::begin(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    setMode(LedMode::GPS_SEARCH);
}

void LedController::setMode(LedMode mode) {
    if (_mode != mode) {
        _mode = mode;
        _animStep = 0;
        _lastUpdateMs = 0;
    }
}

void LedController::triggerSplitFlash() {
    // Яркая белая/изумрудная вспышка на 110 мс при старте или взятии отсечки (60, 100, 200, 402м)
    _splitFlashUntilMs = millis() + 110;
}

void LedController::notifyFixAcquired() {
    // Запуск двойной подтверждающей зеленой вспышки на 500 мс при первом захвате спутников
    _mode = LedMode::GPS_FIX_ACQUIRED;
    _animStep = 0;
    _fixAcquiredUntilMs = millis() + 500;
}

void LedController::setRgb(uint8_t r, uint8_t g, uint8_t b) {
    _curR = r;
    _curG = g;
    _curB = b;
    _writeLed(r, g, b);
}

void LedController::showBatteryStatus(uint8_t percentage, uint8_t mode) {
    _batteryAnimActive = true;
    _batteryPct = percentage;
    _batteryMode = mode;
    
    if (mode == 0) {
        // Режим 1: Световая градация по цветам (непрерывное свечение 2.5 сек)
        _batteryAnimUntilMs = millis() + 2500;
    } else {
        // Режим 2: Серия вспышек (450 мс на цикл)
        uint8_t totalBlinks = (_batteryPct >= 80) ? 4 : ((_batteryPct >= 50) ? 3 : ((_batteryPct >= 25) ? 2 : 1));
        _batteryAnimUntilMs = millis() + (totalBlinks * 450) + 350;
    }
}

void LedController::update() {
    uint32_t now = millis();

    // 1. Приоритетная моментальная яркая вспышка старта и отсечек (0-60, 0-100, 100-200, 402м)
    if (now < _splitFlashUntilMs) {
        _writeLed(220, 255, 220); // Супер-яркий строб-импульс
        return;
    }

    // 2. Индикация уровня заряда аккумулятора (по двойному клику)
    if (_batteryAnimActive) {
        if (now >= _batteryAnimUntilMs) {
            _batteryAnimActive = false;
        } else {
            if (_batteryMode == 0) {
                // Градация по цветам (яркое свечение 2.5 сек):
                if (_batteryPct >= 80) {
                    _writeLed(0, 255, 60);
                } else if (_batteryPct >= 40) {
                    _writeLed(255, 180, 0);
                } else if (_batteryPct >= 20) {
                    _writeLed(255, 60, 0);
                } else {
                    if ((now / 130) % 2 == 0) _writeLed(255, 0, 0);
                    else _writeLed(0, 0, 0);
                }
            } else {
                // Серия вспышек (Blink Count):
                uint8_t totalBlinks = (_batteryPct >= 80) ? 4 : ((_batteryPct >= 50) ? 3 : ((_batteryPct >= 25) ? 2 : 1));
                uint8_t r = (_batteryPct >= 50) ? 0 : 255;
                uint8_t g = (_batteryPct >= 50) ? 255 : ((_batteryPct >= 25) ? 180 : 0);
                uint8_t b = (_batteryPct >= 80) ? 60 : 0;

                uint32_t remaining = _batteryAnimUntilMs - now;
                uint32_t totalDur = (totalBlinks * 450) + 350;
                uint32_t elapsed = (totalDur > remaining) ? (totalDur - remaining) : 0;

                uint8_t curCycle = elapsed / 450;
                uint16_t inCycle = elapsed % 450;

                if (curCycle < totalBlinks && inCycle < 240) {
                    _writeLed(r, g, b);
                } else {
                    _writeLed(0, 0, 0);
                }
            }
            return;
        }
    }

    // 3. Обработка стандартных режимов гонки и навигации
    switch (_mode) {
        case LedMode::OFF:
            _writeLed(0, 0, 0);
            break;

        case LedMode::GPS_SEARCH:
            // Мягкое тусклое мерцание синим цветом в поиске GPS (комфортно для глаз)
            if (now - _lastUpdateMs >= 20) {
                _lastUpdateMs = now;
                _animStep = (_animStep + 1) % 90;
                float breath = (sinf((float)_animStep * 0.0698f) + 1.0f) * 0.5f;
                uint8_t b = (uint8_t)(15.0f + breath * 65.0f);
                uint8_t g = (uint8_t)(breath * 10.0f);
                _writeLed(0, g, b);
            }
            break;

        case LedMode::GPS_FIX_ACQUIRED:
            // Двойная яркая зеленая вспышка при захвате спутников
            if (now - _lastUpdateMs >= 75) {
                _lastUpdateMs = now;
                _animStep++;
                if (_animStep == 1 || _animStep == 3) {
                    _writeLed(0, 255, 60); // Яркая вспышка ВКЛ
                } else {
                    _writeLed(0, 0, 0);   // Вспышка ВЫКЛ
                }

                if (now >= _fixAcquiredUntilMs) {
                    setMode(LedMode::ARMED_READY);
                }
            }
            break;

        case LedMode::ARMED_READY:
        case LedMode::MEASURING:
        case LedMode::LAUNCH_DETECTED:
            // Комфортное тусклое ровное свечение зеленым цветом (GPS пойман)
            _writeLed(0, 40, 10);
            break;

        case LedMode::BRAKING_ACTIVE:
            // Агрессивный красный стробоскоп торможения
            if (now - _lastUpdateMs >= 50) {
                _lastUpdateMs = now;
                _animStep = (_animStep + 1) % 2;
                if (_animStep == 0) _writeLed(255, 0, 0);
                else _writeLed(0, 0, 0);
            }
            break;

        case LedMode::FINISHED_VALID:
            // Тройная победная зеленая вспышка, затем возврат в тусклый зеленый
            if (now - _lastUpdateMs >= 75) {
                _lastUpdateMs = now;
                if (_animStep < 6) {
                    if (_animStep % 2 == 0) _writeLed(0, 255, 60);
                    else _writeLed(0, 0, 0);
                    _animStep++;
                } else {
                    _writeLed(0, 40, 10); // Возврат в тусклый зеленый
                }
            }
            break;

        case LedMode::FINISHED_SLOPE:
            // Предупреждающая тройная янтарно-оранжевая вспышка (уклон > 1%)
            if (now - _lastUpdateMs >= 75) {
                _lastUpdateMs = now;
                if (_animStep < 6) {
                    if (_animStep % 2 == 0) _writeLed(255, 120, 0);
                    else _writeLed(0, 0, 0);
                    _animStep++;
                } else {
                    _writeLed(0, 40, 10); // Возврат в тусклый зеленый
                }
            }
            break;

        case LedMode::CALIBRATING:
            // Пурпурный стробоскоп калибровки акселерометра
            if (now - _lastUpdateMs >= 60) {
                _lastUpdateMs = now;
                _animStep = (_animStep + 1) % 2;
                if (_animStep == 0) _writeLed(220, 0, 255);
                else _writeLed(0, 0, 0);
            }
            break;
    }
}

extern "C" void neopixelWrite(uint8_t pin, uint8_t red_val, uint8_t green_val, uint8_t blue_val);

void LedController::_writeLed(uint8_t r, uint8_t g, uint8_t b) {
    neopixelWrite(_pin, r, g, b);
}

void LedController::turnOff() {
    neopixelWrite(_pin, 0, 0, 0);
}

void LedController::showPowerOnAnimation() {
    // Плавное нарастание изумрудно-зеленого свечения (fade-in), затем подтверждающий импульс
    for (int i = 0; i <= 255; i += 15) {
        _writeLed(0, i, (uint8_t)(i * 0.3f));
        delay(15);
    }
    delay(100);
    _writeLed(0, 0, 0);
    delay(60);
    _writeLed(0, 255, 100);
    delay(120);
    _writeLed(0, 40, 10);
}

void LedController::showPowerOffAnimation() {
    // 2 четкие предупреждающие красные вспышки перед глубоким сном
    for (int k = 0; k < 2; k++) {
        _writeLed(255, 0, 0);
        delay(110);
        _writeLed(0, 0, 0);
        delay(80);
    }
    _writeLed(0, 0, 0);
}

void LedController::showPowerOffHolding(uint8_t progressPct) {
    // Плавный обратный отсчет удержания кнопки (от золотого к ярко-красному)
    uint8_t r = 255;
    uint8_t g = (progressPct < 100) ? (uint8_t)((100 - progressPct) * 1.6f) : 0;
    _writeLed(r, g, 0);
}
