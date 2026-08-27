#include "ButtonManager.h"

ButtonManager::ButtonManager()
    : _pendingEvent(ButtonEvent::NONE)
{
    memset(&_btn, 0, sizeof(ButtonState));
    _btn.pin = 255;
}

void ButtonManager::begin(uint8_t pin) {
    _btn.pin = pin;
    if (_btn.pin == 255) return;
    pinMode(_btn.pin, INPUT_PULLUP);
    _btn.isPressed = (digitalRead(_btn.pin) == LOW);
    _btn.pressStartMs = millis();
    _btn.lastReleaseMs = 0;
    _btn.clickCount = 0;
    _btn.longPressFired = false;
    _pendingEvent = ButtonEvent::NONE;
}

void ButtonManager::update() {
    if (_btn.pin == 255) return;
    bool rawPressed = (digitalRead(_btn.pin) == LOW); // LOW = нажата к GND
    uint32_t now = millis();

    if (rawPressed) {
        if (!_btn.isPressed) {
            // Переход: Кнопка только что нажата
            _btn.isPressed = true;
            _btn.pressStartMs = now;
            _btn.longPressFired = false;
        } else {
            // Кнопка удерживается: проверка длительного нажатия (выключение)
            if (!_btn.longPressFired && (now - _btn.pressStartMs >= BTN_HOLD_POWER_OFF_MS)) {
                _btn.longPressFired = true;
                _btn.clickCount = 0;
                _pendingEvent = ButtonEvent::LONG_PRESS;
            }
        }
    } else {
        if (_btn.isPressed) {
            // Переход: Кнопка отпущена
            _btn.isPressed = false;
            if (!_btn.longPressFired) {
                // Если было короткое нажатие (< 1.8с)
                _btn.clickCount++;
                _btn.lastReleaseMs = now;
            }
        } else {
            // Кнопка в отпущенном состоянии: ожидаем завершения окна мульти-клика (300 мс)
            if (_btn.clickCount > 0 && (now - _btn.lastReleaseMs >= 300)) {
                if (_btn.clickCount >= 2) {
                    _pendingEvent = ButtonEvent::DOUBLE_CLICK;
                } else if (_btn.clickCount == 1) {
                    _pendingEvent = ButtonEvent::CLICK;
                }
                _btn.clickCount = 0;
            }
        }
    }
}

uint32_t ButtonManager::getPressDurationMs() const {
    if (!_btn.isPressed) return 0;
    return (millis() - _btn.pressStartMs);
}

uint8_t ButtonManager::getPowerOffProgressPct() const {
    if (!_btn.isPressed) return 0;
    uint32_t dur = millis() - _btn.pressStartMs;
    if (dur >= BTN_HOLD_POWER_OFF_MS) return 100;
    return (uint8_t)((dur * 100) / BTN_HOLD_POWER_OFF_MS);
}

ButtonEvent ButtonManager::popEvent() {
    ButtonEvent evt = _pendingEvent;
    _pendingEvent = ButtonEvent::NONE;
    return evt;
}
