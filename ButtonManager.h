#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                    BUTTON MANAGER (2 TACTILE BUTTONS)
 * ============================================================================
 * Неблокирующая обработка двух кнопок с программным антидребезгом
 * и поддержкой коротких и длинных нажатий (Long Press).
 */

class ButtonManager {
public:
    ButtonManager();

    void begin(uint8_t pinLeft = PIN_BTN_LEFT, uint8_t pinRight = PIN_BTN_RIGHT);
    void update();

    ButtonEvent getLeftEvent();
    ButtonEvent getRightEvent();

private:
    struct ButtonState {
        uint8_t pin;
        bool lastRawState;
        bool isPressed;
        uint32_t pressStartMs;
        bool longPressFired;
        ButtonEvent pendingEvent;
    };

    ButtonState _btnLeft;
    ButtonState _btnRight;

    void _updateButton(ButtonState& btn);
};
