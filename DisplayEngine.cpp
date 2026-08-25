#include "DisplayEngine.h"

// Цветовая палитра гоночного интерфейса (RGB565)
#define COLOR_BG            0x0841 // Глубокий темно-серый / черный
#define COLOR_CARD_BG       0x18C3 // Темно-серый для карточек
#define COLOR_CARD_BORDER   0x3186 // Граница карточек
#define COLOR_CYAN          0x07FF // Неоновый циан
#define COLOR_GREEN         0x07E0 // Сочный гоночный зеленый
#define COLOR_RED           0xF800 // Яркий красный
#define COLOR_YELLOW        0xFFE0 // Золотисто-желтый
#define COLOR_ORANGE        0xFD20 // Оранжевый
#define COLOR_WHITE         0xFFFF
#define COLOR_GRAY          0x8410
#define COLOR_LIGHT_GRAY    0xC618

DisplayEngine::DisplayEngine()
    : _canvas(&_tft),
      _currentScreen(AppScreen::DASHBOARD_LIVE),
      _lastRenderMs(0)
{
}

bool DisplayEngine::begin() {
    _tft.init();
    _tft.setRotation(LCD_ROTATION);
    _tft.setBrightness(220);

    // Создаем полноэкранный 16-битный спрайт для исключения мерцания
    _canvas.setColorDepth(16);
    _canvas.createSprite(LCD_WIDTH, LCD_HEIGHT);
    _canvas.setTextWrap(false);
    return true;
}

void DisplayEngine::setBrightness(uint8_t brightness) {
    _tft.setBrightness(brightness);
}

void DisplayEngine::setScreen(AppScreen screen) {
    _currentScreen = screen;
}

void DisplayEngine::nextScreen() {
    uint8_t next = ((uint8_t)_currentScreen + 1) % 7;
    _currentScreen = (AppScreen)next;
}

void DisplayEngine::prevScreen() {
    int prev = (int)_currentScreen - 1;
    if (prev < 0) prev = 6;
    _currentScreen = (AppScreen)prev;
}

void DisplayEngine::render(AppScreen screen, 
                           const GpsData& gps, 
                           const ImuData& imu, 
                           RaceState raceState,
                           RaceDiscipline discipline,
                           const RunRecord& currentRun,
                           const RunRecord& lastRun,
                           const DeviceSettings& settings,
                           float liveTimeSec) 
{
    _currentScreen = screen;
    _canvas.fillScreen(COLOR_BG);

    // Верхняя информационная статус-строка
    _drawTopStatusBar(gps, raceState);

    // Отрисовка активного экрана
    switch (_currentScreen) {
        case AppScreen::DASHBOARD_LIVE:
            _renderDashboardLive(gps, imu, raceState);
            break;

        case AppScreen::DRAG_RACE:
            _renderDragRace(gps, imu, raceState, discipline, currentRun, liveTimeSec);
            break;

        case AppScreen::BRAKE_TEST:
            _renderBrakeTest(gps, imu, raceState, currentRun);
            break;

        case AppScreen::G_METER_SCREEN:
            _renderGMeterScreen(imu);
            break;

        case AppScreen::RUN_RESULTS:
            _renderRunResults(lastRun);
            break;

        case AppScreen::GPS_INFO:
            _renderGpsInfo(gps);
            break;

        case AppScreen::SETTINGS:
            _renderSettings(settings);
            break;
    }

    // Выталкиваем буфер на дисплей по SPI DMA
    _canvas.pushSprite(0, 0);
}

void DisplayEngine::_drawTopStatusBar(const GpsData& gps, RaceState raceState) {
    // Фоновая полоса
    _canvas.fillRect(0, 0, LCD_WIDTH, 24, 0x1082);
    _canvas.drawFastHLine(0, 24, LCD_WIDTH, COLOR_CARD_BORDER);

    // Спутники
    _canvas.setTextSize(1);
    _canvas.setTextColor(gps.validFix ? COLOR_GREEN : COLOR_RED);
    _canvas.setCursor(6, 7);
    _canvas.printf("SAT:%02d", gps.numSats);

    // Точность
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.setCursor(62, 7);
    if (gps.validFix) {
        _canvas.printf("+/-%.1fm", gps.hAccM);
    } else {
        _canvas.print("NO FIX");
    }

    // Статус прибора (Pill Badge справа)
    const char* statusText = "WAIT";
    uint16_t statusBg = COLOR_YELLOW;
    uint16_t statusFg = COLOR_BG;

    switch (raceState) {
        case RaceState::IDLE_WAIT_STOP:
            statusText = "WAIT STOP";
            statusBg = COLOR_YELLOW;
            break;
        case RaceState::ARMED:
            statusText = "READY";
            statusBg = COLOR_GREEN;
            break;
        case RaceState::LAUNCH_DETECTED:
        case RaceState::MEASURING:
            statusText = "LAUNCH";
            statusBg = COLOR_CYAN;
            break;
        case RaceState::FINISHED:
            statusText = "DONE";
            statusBg = COLOR_GREEN;
            break;
        case RaceState::ABORTED:
            statusText = "ABORT";
            statusBg = COLOR_RED;
            break;
    }

    _drawPillBadge(LCD_WIDTH - 85, 3, 78, 18, statusText, statusBg, statusFg);
}

void DisplayEngine::_renderDashboardLive(const GpsData& gps, const ImuData& imu, RaceState raceState) {
    // Цифровой спидометр (огромный шрифт)
    int speedVal = (int)(gps.speedKmh + 0.5f);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.setTextSize(1);
    _canvas.setFont(&fonts::Font7); // Большие цифры
    _canvas.setTextDatum(textdatum_t::middle_center);
    _canvas.drawNumber(speedVal, LCD_WIDTH / 2 - 20, 85);

    // Единица измерения
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.drawString("KM/H", LCD_WIDTH / 2 + 75, 95);

    // Нижняя плашка: G-Force и Макс. скорость
    _canvas.fillRoundRect(8, 130, LCD_WIDTH - 16, 36, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(8, 130, LCD_WIDTH - 16, 36, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("G-LONG:", 20, 142);
    
    _canvas.setTextColor((imu.gLongitudinal >= 0) ? COLOR_GREEN : COLOR_RED);
    _canvas.printf("%+.2f G", imu.gLongitudinal);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("PEAK G:", 180, 142);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.printf("%.2f G", imu.gPeakAccel);
}

void DisplayEngine::_renderDragRace(const GpsData& gps, const ImuData& imu, 
                                    RaceState raceState, RaceDiscipline discipline, 
                                    const RunRecord& currentRun, float liveTimeSec) 
{
    // Карточка таймера (левая половина)
    _canvas.fillRoundRect(8, 30, 150, 134, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(8, 30, 150, 134, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.setTextDatum(textdatum_t::top_left);
    _canvas.drawString("DRAG TIMER", 16, 36);

    _canvas.setFont(&fonts::Font6); // Крупный таймер
    _canvas.setTextColor((raceState == RaceState::MEASURING) ? COLOR_GREEN : COLOR_WHITE);
    _canvas.setCursor(16, 60);
    _canvas.printf("%04.2f", liveTimeSec);
    _canvas.setFont(&fonts::Font2);
    _canvas.print(" s");

    // Скорость и дистанция заезда
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_LIGHT_GRAY);
    _canvas.setCursor(16, 115);
    _canvas.printf("Spd: %.1f km/h", gps.speedKmh);
    _canvas.setCursor(16, 138);
    _canvas.printf("Dst: %.1f m", currentRun.totalDistanceM);

    // Правая половина: Живые отсечки (Splits)
    _canvas.fillRoundRect(164, 30, 148, 134, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(164, 30, 148, 134, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.drawString("SPLITS", 172, 36);

    // 0 - 60
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("0-60:", 172, 58);
    if (currentRun.split0_60.achieved) {
        _canvas.setTextColor(COLOR_GREEN);
        _canvas.printf("%.2fs", currentRun.split0_60.timeSec);
    } else {
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
        _canvas.print("---");
    }

    // 0 - 100
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("0-100:", 172, 80);
    if (currentRun.split0_100.achieved) {
        _canvas.setTextColor(COLOR_GREEN);
        _canvas.printf("%.2fs", currentRun.split0_100.timeSec);
    } else {
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
        _canvas.print("---");
    }

    // 100 - 200
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("100-200:", 172, 102);
    if (currentRun.split100_200.achieved) {
        _canvas.setTextColor(COLOR_GREEN);
        _canvas.printf("%.2fs", currentRun.split100_200.timeSec);
    } else {
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
        _canvas.print("---");
    }

    // 1/4 Mile (402m)
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("1/4mi:", 172, 124);
    if (currentRun.split1_4mi.achieved) {
        _canvas.setTextColor(COLOR_GREEN);
        _canvas.printf("%.2fs", currentRun.split1_4mi.timeSec);
    } else {
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
        _canvas.print("---");
    }
}

void DisplayEngine::_renderBrakeTest(const GpsData& gps, const ImuData& imu, RaceState raceState, const RunRecord& currentRun) {
    _canvas.fillRoundRect(8, 30, LCD_WIDTH - 16, 134, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(8, 30, LCD_WIDTH - 16, 134, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_RED);
    _canvas.drawString("BRAKING TEST (100-0 KM/H)", 16, 36);

    _canvas.setFont(&fonts::Font4);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.setCursor(16, 65);
    if (currentRun.split100_0.achieved) {
        _canvas.printf("DIST: %.2f m", currentRun.brakeDist100_0M);
        _canvas.setCursor(16, 100);
        _canvas.printf("TIME: %.2f s", currentRun.split100_0.timeSec);
    } else {
        _canvas.printf("LIVE SPEED: %.1f", gps.speedKmh);
        _canvas.setCursor(16, 100);
        _canvas.printf("MAX BRAKE: %.2f G", imu.gPeakBrake);
    }
}

void DisplayEngine::_renderGMeterScreen(const ImuData& imu) {
    int cx = LCD_WIDTH / 2 - 40;
    int cy = 100;
    int radius = 55;

    // Сетка G-Bowl
    _canvas.drawCircle(cx, cy, radius, COLOR_CARD_BORDER);
    _canvas.drawCircle(cx, cy, radius / 2, COLOR_CARD_BORDER);
    _canvas.drawFastHLine(cx - radius, cy, radius * 2, COLOR_CARD_BORDER);
    _canvas.drawFastVLine(cx, cy - radius, radius * 2, COLOR_CARD_BORDER);

    // Точка текущей перегрузки
    int dotX = cx + (int)(imu.gLateral * 45.0f);
    int dotY = cy - (int)(imu.gLongitudinal * 45.0f);
    dotX = constrain(dotX, cx - radius, cx + radius);
    dotY = constrain(dotY, cy - radius, cy + radius);

    _canvas.fillCircle(dotX, dotY, 6, COLOR_GREEN);
    _canvas.drawCircle(dotX, dotY, 7, COLOR_WHITE);

    // Числовые показатели справа
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("LONG G:", LCD_WIDTH - 120, 45);
    _canvas.setTextColor(COLOR_GREEN);
    _canvas.printf("%+.2f", imu.gLongitudinal);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("LAT G:", LCD_WIDTH - 120, 75);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.printf("%+.2f", imu.gLateral);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("PEAK G:", LCD_WIDTH - 120, 105);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.printf("%.2f", imu.gPeakAccel);
}

void DisplayEngine::_renderRunResults(const RunRecord& run) {
    // Заголовок экрана результатов
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.drawString("LAST RUN SUMMARY", 10, 30);

    // Бейдж валидности уклона (Dragy Style)
    if (run.isValidSlope) {
        _drawPillBadge(LCD_WIDTH - 110, 28, 100, 18, "SLOPE VALID", COLOR_GREEN, COLOR_BG);
    } else {
        _drawPillBadge(LCD_WIDTH - 110, 28, 100, 18, "INVALID SLOPE", COLOR_RED, COLOR_WHITE);
    }

    // Таблица результатов
    _canvas.fillRoundRect(8, 52, LCD_WIDTH - 16, 112, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(8, 52, LCD_WIDTH - 16, 112, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_GRAY);

    // Колонка 1: Скорости
    _canvas.drawString("0-100:", 16, 60);
    _canvas.setTextColor(COLOR_GREEN);
    if (run.split0_100.achieved) _canvas.printf("%.2fs", run.split0_100.timeSec); else _canvas.print("---");

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("100-200:", 16, 82);
    _canvas.setTextColor(COLOR_GREEN);
    if (run.split100_200.achieved) _canvas.printf("%.2fs", run.split100_200.timeSec); else _canvas.print("---");

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("0-60:", 16, 104);
    _canvas.setTextColor(COLOR_GREEN);
    if (run.split0_60.achieved) _canvas.printf("%.2fs", run.split0_60.timeSec); else _canvas.print("---");

    // Колонка 2: Драг (1/4 мили, 60ft)
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("1/4mi:", 160, 60);
    _canvas.setTextColor(COLOR_YELLOW);
    if (run.split1_4mi.achieved) _canvas.printf("%.2fs @ %.0f", run.split1_4mi.timeSec, run.split1_4mi.trapSpeedKmh); else _canvas.print("---");

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("60ft:", 160, 82);
    _canvas.setTextColor(COLOR_WHITE);
    if (run.split60ft.achieved) _canvas.printf("%.2fs", run.split60ft.timeSec); else _canvas.print("---");

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Slope:", 160, 104);
    _canvas.setTextColor(run.isValidSlope ? COLOR_GREEN : COLOR_RED);
    _canvas.printf("%+.1f%%", run.slopePct);

    // Нижняя строка
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("1-Foot:", 16, 132);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.print(run.rolloutUsed ? "ON" : "OFF");

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("MaxG:", 160, 132);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.printf("%.2fG", run.maxAccelG);
}

void DisplayEngine::_renderGpsInfo(const GpsData& gps) {
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.drawString("GNSS u-blox M10Q STATUS", 10, 30);

    _canvas.fillRoundRect(8, 52, LCD_WIDTH - 16, 112, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(8, 52, LCD_WIDTH - 16, 112, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_GRAY);
    
    _canvas.drawString("Lat:", 16, 60);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.printf("%.6f", gps.lat);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Lon:", 16, 82);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.printf("%.6f", gps.lon);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Alt MSL:", 16, 104);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.printf("%.1f m", gps.altMSL);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("hAcc / vAcc:", 16, 126);
    _canvas.setTextColor(COLOR_GREEN);
    _canvas.printf("%.2fm / %.2fm", gps.hAccM, gps.vAccM);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Speed Acc:", 180, 60);
    _canvas.setTextColor(COLOR_GREEN);
    _canvas.printf("+/-%.2f km/h", gps.sAccKmh);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Rate:", 180, 82);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.printf("%d Hz UBX", GPS_UPDATE_RATE_HZ);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Fix Type:", 180, 104);
    _canvas.setTextColor(gps.validFix ? COLOR_GREEN : COLOR_RED);
    _canvas.printf("%s", (gps.fixType == 3) ? "3D FIX" : (gps.fixType == 4) ? "3D+DR" : "NO FIX");
}

void DisplayEngine::_renderSettings(const DeviceSettings& settings) {
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.drawString("DEVICE SETTINGS", 10, 30);

    _canvas.fillRoundRect(8, 52, LCD_WIDTH - 16, 112, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(8, 52, LCD_WIDTH - 16, 112, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_GRAY);
    
    _canvas.drawString("1-Foot Rollout:", 16, 62);
    _canvas.setTextColor(settings.use1FootRollout ? COLOR_GREEN : COLOR_RED);
    _canvas.print(settings.use1FootRollout ? "ENABLED (Dragy)" : "DISABLED (Real 0)");

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Max Slope Tol:", 16, 88);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.printf("%.1f%% (NHRA)", settings.slopeTolerancePct);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Firmware:", 16, 114);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.print(DRAGON_FW_VERSION);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Btn R: Calibrate IMU", 16, 138);
}

void DisplayEngine::_drawPillBadge(int x, int y, int w, int h, const char* text, uint16_t bgColor, uint16_t textColor) {
    _canvas.fillRoundRect(x, y, w, h, h / 2, bgColor);
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(textColor);
    _canvas.setTextDatum(textdatum_t::middle_center);
    _canvas.drawString(text, x + w / 2, y + h / 2);
    _canvas.setTextDatum(textdatum_t::top_left);
}
