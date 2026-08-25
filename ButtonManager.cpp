#include "ButtonManager.h"

ButtonManager::ButtonManager() {
    memset(&_btn, 0, sizeof(ButtonState));
}

void ButtonManager::begin(uint8_t pin) {
    _btn.pin = pin;
    pinMode(_btn.pin, INPUT_PULLUP);
    _btn.lastRawState = HIGH;
    _btn.isPressed = false;
    _btn.clickCount = 0;
    _btn.longPressFired = false;
    _btn.pendingEvent = ButtonEvent::NONE;
}

void ButtonManager::update() {
    _updateButton(_btn);
}

void ButtonManager::_updateButton(ButtonState& btn) {
    bool rawReading = (digitalRead(btn.pin) == LOW); // LOW = нажата к GND
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
            btn.clickCount = 0;
        }
    } else if (!rawReading && btn.isPressed) {
        // Отпускание
        btn.isPressed = false;
        if (!btn.longPressFired && (now - btn.pressStartMs >= BTN_DEBOUNCE_MS)) {
            btn.clickCount++;
            btn.lastReleaseMs = now;
        }
    }

    // Обработка одиночного или двойного клика по тайм-ауту после отпускания
    if (btn.clickCount > 0 && !btn.isPressed) {
        if (btn.clickCount >= 2) {
            btn.pendingEvent = ButtonEvent::DOUBLE_CLICK;
            btn.clickCount = 0;
        } else if (now - btn.lastReleaseMs >= 280) { // Окно ожидания второго клика 280мс
            btn.pendingEvent = ButtonEvent::CLICK;
            btn.clickCount = 0;
        }
    }
}

ButtonEvent ButtonManager::getEvent() {
    ButtonEvent ev = _btn.pendingEvent;
    _btn.pendingEvent = ButtonEvent::NONE;
    return ev;
}
