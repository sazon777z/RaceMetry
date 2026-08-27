#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                      POWER BUTTON MANAGER (GPIO 11)
 * ============================================================================
 * Неблокирующая обработка кнопки включения / выключения питания:
 * - Включение: пробуждение из Deep Sleep по нажатию кнопки
 * - Выключение: удержание кнопки в течение 1.8 сек с световой индикацией
 */

class ButtonManager {
public:
    ButtonManager();

    void begin(uint8_t pin = PIN_BTN);
    void update();

    // Проверка нажатия кнопки
    bool isPressed() const { return _btn.isPressed; }

    // Длительность текущего удержания кнопки (мс)
    uint32_t getPressDurationMs() const;

    // Прогресс удержания кнопки для выключения (0..100%)
    uint8_t getPowerOffProgressPct() const;

    // Извлечь и сбросить накопленное событие кнопки
    ButtonEvent popEvent();

private:
    struct ButtonState {
        uint8_t pin;
        bool isPressed;
        uint32_t pressStartMs;
        uint32_t lastReleaseMs;
        uint8_t clickCount;
        bool longPressFired;
    };

    ButtonState _btn;
    ButtonEvent _pendingEvent;
};
