#include "ButtonManager.h"

ButtonManager::ButtonManager()
    : _pin(255),
      _rawState(false),
      _debouncedPressed(false),
      _lastDebounceTimeMs(0),
      _pressStartMs(0),
      _lastReleaseMs(0),
      _clickCount(0),
      _longPressTriggered(false),
      _pendingEvent(ButtonEvent::NONE)
{
}

void ButtonManager::begin(uint8_t pin) {
    _pin = pin;
    if (_pin == 255) return;
    pinMode(_pin, INPUT_PULLUP);
    _rawState = (digitalRead(_pin) == LOW);
    _debouncedPressed = _rawState;
    _lastDebounceTimeMs = millis();
    _pressStartMs = _rawState ? millis() : 0;
    _lastReleaseMs = 0;
    _clickCount = 0;
    _longPressTriggered = false;
    _pendingEvent = ButtonEvent::NONE;
}

void ButtonManager::update() {
    if (_pin == 255) return;
    uint32_t now = millis();
    bool rawNow = (digitalRead(_pin) == LOW); // LOW = нажата к GND

    // 1. Аппаратно-программный антидребезг (35 мс фильтрации)
    if (rawNow != _rawState) {
        _rawState = rawNow;
        _lastDebounceTimeMs = now;
    }

    if ((now - _lastDebounceTimeMs) >= 35) {
        if (_rawState != _debouncedPressed) {
            _debouncedPressed = _rawState;

            if (_debouncedPressed) {
                // Событие: Кнопка ЧЕТКО нажата
                _pressStartMs = now;
                _longPressTriggered = false;
            } else {
                // Событие: Кнопка ЧЕТКО отпущена
                uint32_t pressDuration = now - _pressStartMs;
                if (!_longPressTriggered && pressDuration >= 40 && pressDuration < BTN_HOLD_POWER_OFF_MS) {
                    _clickCount++;
                    _lastReleaseMs = now;
                }
            }
        }
    }

    // 2. Обработка длительного удержания (Power OFF: 1500 мс)
    if (_debouncedPressed && !_longPressTriggered) {
        uint32_t holdDuration = now - _pressStartMs;
        if (holdDuration >= BTN_HOLD_POWER_OFF_MS) {
            _longPressTriggered = true;
            _clickCount = 0;
            _pendingEvent = ButtonEvent::LONG_PRESS;
        }
    }

    // 3. Обработка окна завершения кликов (Single vs Double click: 320 мс)
    if (!_debouncedPressed && _clickCount > 0) {
        if (now - _lastReleaseMs >= 320) {
            if (_clickCount >= 2) {
                _pendingEvent = ButtonEvent::DOUBLE_CLICK;
            } else if (_clickCount == 1) {
                _pendingEvent = ButtonEvent::CLICK;
            }
            _clickCount = 0;
        }
    }
}

uint32_t ButtonManager::getPressDurationMs() const {
    if (!_debouncedPressed) return 0;
    return (millis() - _pressStartMs);
}

uint8_t ButtonManager::getPowerOffProgressPct() const {
    if (!_debouncedPressed) return 0;
    uint32_t dur = millis() - _pressStartMs;
    if (dur >= BTN_HOLD_POWER_OFF_MS) return 100;
    if (dur < 350) return 0;
    return (uint8_t)(((dur - 350) * 100) / (BTN_HOLD_POWER_OFF_MS - 350));
}

ButtonEvent ButtonManager::popEvent() {
    ButtonEvent evt = _pendingEvent;
    _pendingEvent = ButtonEvent::NONE;
    return evt;
}
