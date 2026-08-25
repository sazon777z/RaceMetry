#include "LedController.h"

LedController::LedController()
    : _pin(PIN_WS2812),
      _mode(LedMode::GPS_SEARCH),
      _lastUpdateMs(0),
      _animStep(0),
      _curR(0), _curG(0), _curB(0)
{
}

void LedController::begin(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    setMode(LedMode::GPS_SEARCH);
}

void LedController::setMode(LedMode mode) {
    if (_mode != mode) {
        _mode = mode;
        _animStep = 0;
        _lastUpdateMs = 0;
    }
}

void LedController::setRgb(uint8_t r, uint8_t g, uint8_t b) {
    _curR = r;
    _curG = g;
    _curB = b;
    _writeLed(r, g, b);
}

void LedController::update() {
    uint32_t now = millis();

    switch (_mode) {
        case LedMode::OFF:
            _writeLed(0, 0, 0);
            break;

        case LedMode::GPS_SEARCH:
            // Плавное дыхание красным (период 1.5 сек)
            if (now - _lastUpdateMs >= 30) {
                _lastUpdateMs = now;
                _animStep = (_animStep + 1) % 100;
                float breath = (sinf((float)_animStep * 0.0628f) + 1.0f) * 0.5f;
                uint8_t r = (uint8_t)(breath * 180.0f);
                uint8_t g = (uint8_t)(breath * 20.0f);
                _writeLed(r, g, 0);
            }
            break;

        case LedMode::ARMED_READY:
            // Сочный постоянный зеленый
            _writeLed(0, 255, 0);
            break;

        case LedMode::RUNNING:
            // Быстрое дыхание цианом
            if (now - _lastUpdateMs >= 20) {
                _lastUpdateMs = now;
                _animStep = (_animStep + 1) % 60;
                float val = (sinf((float)_animStep * 0.1047f) + 1.0f) * 0.5f;
                uint8_t b = (uint8_t)(100.0f + val * 155.0f);
                uint8_t g = (uint8_t)(val * 120.0f);
                _writeLed(0, g, b);
            }
            break;

        case LedMode::FINISHED_VALID:
            // Стробоскоп зеленым цветом (3 быстрых вспышки, затем постоянный)
            if (now - _lastUpdateMs >= 80) {
                _lastUpdateMs = now;
                if (_animStep < 6) {
                    if (_animStep % 2 == 0) _writeLed(0, 255, 0);
                    else _writeLed(0, 0, 0);
                    _animStep++;
                } else {
                    _writeLed(0, 200, 0);
                }
            }
            break;

        case LedMode::FINISHED_SLOPE:
            // Стробоскоп оранжевым цветом (предупреждение об уклоне)
            if (now - _lastUpdateMs >= 80) {
                _lastUpdateMs = now;
                if (_animStep < 6) {
                    if (_animStep % 2 == 0) _writeLed(255, 120, 0);
                    else _writeLed(0, 0, 0);
                    _animStep++;
                } else {
                    _writeLed(200, 80, 0);
                }
            }
            break;

        case LedMode::CALIBRATING:
            _writeLed(200, 0, 255); // Пурпурный
            break;
    }
}

void LedController::_writeLed(uint8_t r, uint8_t g, uint8_t b) {
#ifdef neopixelWrite
    neopixelWrite(_pin, r, g, b);
#else
    // Заглушка, если neopixelWrite не объявлен на старых ядрах
    (void)_pin; (void)r; (void)g; (void)b;
#endif
}
