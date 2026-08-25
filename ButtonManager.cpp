#include "ButtonManager.h"

ButtonManager::ButtonManager() {
    memset(&_btnLeft, 0, sizeof(ButtonState));
    memset(&_btnRight, 0, sizeof(ButtonState));
}

void ButtonManager::begin(uint8_t pinLeft, uint8_t pinRight) {
    _btnLeft.pin = pinLeft;
    _btnRight.pin = pinRight;

    pinMode(_btnLeft.pin, INPUT_PULLUP);
    pinMode(_btnRight.pin, INPUT_PULLUP);

    _btnLeft.lastRawState = HIGH;
    _btnRight.lastRawState = HIGH;
}

void ButtonManager::update() {
    _updateButton(_btnLeft);
    _updateButton(_btnRight);
}

void ButtonManager::_updateButton(ButtonState& btn) {
    bool rawReading = (digitalRead(btn.pin) == LOW); // LOW = нажата
    uint32_t now = millis();

    if (rawReading && !btn.isPressed) {
        // Начало нажатия
        btn.isPressed = true;
        btn.pressStartMs = now;
        btn.longPressFired = false;
    } else if (rawReading && btn.isPressed) {
        // Удержание
        if (!btn.longPressFired && (now - btn.pressStartMs >= BTN_LONG_PRESS_MS)) {
            btn.longPressFired = true;
            btn.pendingEvent = ButtonEvent::LONG_PRESS;
        }
    } else if (!rawReading && btn.isPressed) {
        // Отпускание
        btn.isPressed = false;
        if (!btn.longPressFired && (now - btn.pressStartMs >= BTN_DEBOUNCE_MS)) {
            btn.pendingEvent = ButtonEvent::CLICK;
        }
    }
}

ButtonEvent ButtonManager::getLeftEvent() {
    ButtonEvent ev = _btnLeft.pendingEvent;
    _btnLeft.pendingEvent = ButtonEvent::NONE;
    return ev;
}

ButtonEvent ButtonManager::getRightEvent() {
    ButtonEvent ev = _btnRight.pendingEvent;
    _btnRight.pendingEvent = ButtonEvent::NONE;
    return ev;
}
