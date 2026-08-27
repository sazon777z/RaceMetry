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

    // Текущее физическое состояние кнопки (с учетом антидребезга)
    bool isPressed() const { return _debouncedPressed; }

    // Длительность текущего удержания кнопки (мс)
    uint32_t getPressDurationMs() const;

    // Прогресс удержания кнопки для выключения (0..100%)
    uint8_t getPowerOffProgressPct() const;

    // Извлечь и сбросить накопленное событие кнопки
    ButtonEvent popEvent();

private:
    uint8_t _pin;
    bool _rawState;
    bool _debouncedPressed;
    uint32_t _lastDebounceTimeMs;

    uint32_t _pressStartMs;
    uint32_t _lastReleaseMs;
    uint8_t _clickCount;
    bool _longPressTriggered;

    ButtonEvent _pendingEvent;
};
