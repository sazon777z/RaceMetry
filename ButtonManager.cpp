#include "ButtonManager.h"

ButtonManager::ButtonManager() {
    memset(&_btn, 0, sizeof(ButtonState));
}

void ButtonManager::begin(uint8_t pin) {
    _btn.pin = pin;
    if (_btn.pin == 255) return;
    pinMode(_btn.pin, INPUT_PULLUP);
    _btn.isPressed = (digitalRead(_btn.pin) == LOW);
    _btn.pressStartMs = millis();
    _btn.powerOffFired = false;
}

void ButtonManager::update() {
    if (_btn.pin == 255) return;
    bool rawReading = (digitalRead(_btn.pin) == LOW); // LOW = нажата к GND
    uint32_t now = millis();

    if (rawReading && !_btn.isPressed) {
        // Начало нажатия
        _btn.isPressed = true;
        _btn.pressStartMs = now;
        _btn.powerOffFired = false;
    } else if (!rawReading && _btn.isPressed) {
        // Отпускание
        _btn.isPressed = false;
        _btn.powerOffFired = false;
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

bool ButtonManager::isPowerOffTriggered() {
    if (_btn.isPressed && !_btn.powerOffFired) {
        if (getPressDurationMs() >= BTN_HOLD_POWER_OFF_MS) {
            _btn.powerOffFired = true;
            return true;
        }
    }
    return false;
}
