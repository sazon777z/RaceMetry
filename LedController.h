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
    GPS_SEARCH,          // Плавная пульсация синим цветом (поиск спутников)
    GPS_FIX_ACQUIRED,    // Двойная вспышка зеленым при захвате 3D-фикса
    ARMED_READY,         // Постоянный сочный зеленый (готов к старту)
    LAUNCH_DETECTED,     // Быстрое мерцание неоновым синим/цианом (старт)
    MEASURING,           // Динамическая скоростная пульсация синим в заезде
    RUNNING = MEASURING, // Псевдоним для совместимости
    BRAKING_ACTIVE,      // Яркий красный стробоскоп при торможении
    FINISHED_VALID,      // Тройная победная зеленая вспышка (валидный заезд)
    FINISHED_SLOPE,      // Янтарно-оранжевая вспышка (предупреждение об уклоне)
    CALIBRATING          // Фиолетовый стробоскоп (калибровка IMU)
};

class LedController {
public:
    LedController();

    void begin(uint8_t pin = PIN_WS2812);
    void update();

    void setMode(LedMode mode);
    void triggerSplitFlash();
    void notifyFixAcquired();
    void setRgb(uint8_t r, uint8_t g, uint8_t b);

    // Анимации включения и выключения прибора
    void showPowerOnAnimation();
    void showPowerOffAnimation();
    void showPowerOffHolding(uint8_t progressPct);
    void turnOff();

private:
    uint8_t _pin;
    LedMode _mode;
    uint32_t _lastUpdateMs;
    uint8_t _animStep;
    uint8_t _curR, _curG, _curB;
    uint32_t _splitFlashUntilMs;
    uint32_t _fixAcquiredUntilMs;

    void _writeLed(uint8_t r, uint8_t g, uint8_t b);
};
