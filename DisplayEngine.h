#pragma once
#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                 DISPLAY ENGINE (1.47" IPS ST7789 320x172)
 * ============================================================================
 * Графический движок на базе LovyanGFX с двойной буферизацией (Sprite DMA).
 * Обеспечивает отображение 60 FPS без мерцания.
 */

// Конфигурация драйвера LovyanGFX для 1.47" ST7789 SPI на ESP32-S3
class LGFX_DRAGon : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789  _panel_instance;
    lgfx::Bus_SPI       _bus_instance;
    lgfx::Light_PWM     _light_instance;

public:
    LGFX_DRAGon() {
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host = SPI2_HOST; // FSPI
            cfg.spi_mode = 0;
            cfg.freq_write = 40000000; // 40 МГц
            cfg.freq_read  = 16000000;
            cfg.pin_sclk = PIN_LCD_SCLK;
            cfg.pin_mosi = PIN_LCD_MOSI;
            cfg.pin_miso = PIN_LCD_MISO;
            cfg.pin_dc   = PIN_LCD_DC;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs           = PIN_LCD_CS;
            cfg.pin_rst          = PIN_LCD_RST;
            cfg.pin_busy         = -1;
            cfg.memory_width     = 172;
            cfg.memory_height    = 320;
            cfg.panel_width      = 172;
            cfg.panel_height     = 320;
            cfg.offset_x         = 34;  // Смещение для 1.47" ST7789 (172x320)
            cfg.offset_y         = 0;
            cfg.offset_rotation  = 1;   // Альбомная ориентация (320x172)
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits  = 1;
            cfg.readable         = false;
            cfg.invert           = true; // Инверсия цветов для IPS
            cfg.rgb_order        = false;
            cfg.dlen_16bit       = false;
            cfg.bus_shared       = false;
            _panel_instance.config(cfg);
        }

        {
            auto cfg = _light_instance.config();
            cfg.pin_bl = PIN_LCD_BLK;
            cfg.invert = false;
            cfg.freq   = 44100;
            cfg.pwm_channel = 7;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        setPanel(&_panel_instance);
    }
};

class DisplayEngine {
public:
    DisplayEngine();

    bool begin();
    void setBrightness(uint8_t brightness);

    // Основной цикл перерисовки экрана (вызывается на Core 1)
    void render(AppScreen screen, 
                const GpsData& gps, 
                const ImuData& imu, 
                RaceState raceState,
                RaceDiscipline discipline,
                const RunRecord& currentRun,
                const RunRecord& lastRun,
                const DeviceSettings& settings,
                float liveTimeSec);

    // Навигация
    void nextScreen();
    void prevScreen();
    void setScreen(AppScreen screen);
    AppScreen getScreen() const { return _currentScreen; }

private:
    LGFX_DRAGon _tft;
    LGFX_Sprite _canvas; // Полноэкранный спрайт для двойной буферизации
    AppScreen _currentScreen;
    uint32_t _lastRenderMs;

    // Отрисовка конкретных экранов
    void _drawTopStatusBar(const GpsData& gps, RaceState raceState);
    void _renderDashboardLive(const GpsData& gps, const ImuData& imu, RaceState raceState);
    void _renderDragRace(const GpsData& gps, const ImuData& imu, RaceState raceState, RaceDiscipline discipline, const RunRecord& currentRun, float liveTimeSec);
    void _renderBrakeTest(const GpsData& gps, const ImuData& imu, RaceState raceState, const RunRecord& currentRun);
    void _renderGMeterScreen(const ImuData& imu);
    void _renderRunResults(const RunRecord& run);
    void _renderGpsInfo(const GpsData& gps);
    void _renderSettings(const DeviceSettings& settings);

    // Вспомогательные методы рисования графики
    void _drawGBar(int x, int y, int w, int h, float gVal, float maxG);
    void _drawPillBadge(int x, int y, int w, int h, const char* text, uint16_t bgColor, uint16_t textColor);
};
