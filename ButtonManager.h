#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                    BUTTON MANAGER (SINGLE BUTTON)
 * ============================================================================
 * Неблокирующая обработка единственной кнопки с программным антидребезгом
 * и поддержкой событий:
 * - Одиночный клик (CLICK): Взведение (ARM) / Сброс
 * - Двойной клик (DOUBLE_CLICK): Переключение дисциплины по кругу
 * - Длинное нажатие (LONG_PRESS): Калибровка нуля акселерометра IMU
 */

class ButtonManager {
public:
    ButtonManager();

    void begin(uint8_t pin = PIN_BTN);
    void update();

    ButtonEvent getEvent();

private:
    struct ButtonState {
        uint8_t pin;
        bool lastRawState;
        bool isPressed;
        uint32_t pressStartMs;
        uint32_t lastReleaseMs;
        uint8_t clickCount;
        bool longPressFired;
        ButtonEvent pendingEvent;
    };

    ButtonState _btn;

    void _updateButton(ButtonState& btn);
};
