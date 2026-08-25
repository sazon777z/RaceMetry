#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                    LED CONTROLLER (WS2812B RGB RMT)
 * ============================================================================
 * Управление адресным светодиодом WS2812B для визуализации состояния прибора:
 * - Красный мигающий: Поиск спутников GPS / Плохой 3D Fix
 * - Зеленый постоянный: ARMED (Готов к старту)
 * - Голубой пульсирующий: Идет заезд (RUNNING)
 * - Зеленый стробоскоп: Заезд завершен (Валидный результат)
 * - Оранжевый стробоскоп: Заезд завершен (Уклон не валиден)
 */

enum class LedMode : uint8_t {
    OFF = 0,
    GPS_SEARCH,      // Медленное мигание оранжево-красным
    ARMED_READY,     // Яркий постоянный зеленый (готов к старту)
    RUNNING,         // Пульсирующий сине-голубой
    FINISHED_VALID,  // 3 быстрых вспышки зеленым
    FINISHED_SLOPE,  // 3 быстрых вспышки желтым
    CALIBRATING      // Фиолетовый
};

class LedController {
public:
    LedController();

    void begin(uint8_t pin = PIN_WS2812);
    void update();

    void setMode(LedMode mode);
    void setRgb(uint8_t r, uint8_t g, uint8_t b);

private:
    uint8_t _pin;
    LedMode _mode;
    uint32_t _lastUpdateMs;
    uint8_t _animStep;
    uint8_t _curR, _curG, _curB;

    void _writeLed(uint8_t r, uint8_t g, uint8_t b);
};
