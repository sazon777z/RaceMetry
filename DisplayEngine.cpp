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
    _tft.fillScreen(0x0000); // Очистка экрана от случайного шума VRAM
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

    // 1. Верхняя статус-строка
    _drawTopStatusBar(gps, raceState);

    // 2. Отрисовка активного экрана
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

    // Выталкиваем кадр на дисплей по SPI DMA
    _canvas.pushSprite(0, 0);
}

void DisplayEngine::_drawTopStatusBar(const GpsData& gps, RaceState raceState) {
    // Фоновая полоса
    _canvas.fillRect(0, 0, LCD_WIDTH, 24, 0x1082);
    _canvas.drawFastHLine(0, 24, LCD_WIDTH, COLOR_CARD_BORDER);

    // Спутники
    char satStr[16];
    snprintf(satStr, sizeof(satStr), "SAT: %02d", gps.numSats);
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(gps.validFix ? COLOR_GREEN : COLOR_RED);
    _canvas.setTextDatum(textdatum_t::top_left);
    _canvas.drawString(satStr, 16, 4);

    // Точность
    char accStr[20];
    if (gps.validFix) {
        snprintf(accStr, sizeof(accStr), "+/-%.1fm", gps.hAccM);
    } else {
        snprintf(accStr, sizeof(accStr), "NO FIX");
    }
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString(accStr, 92, 4);

    // Статус прибора (Pill Badge справа)
    const char* statusText = "WAIT";
    uint16_t statusBg = COLOR_YELLOW;
    uint16_t statusFg = COLOR_BG;

    switch (raceState) {
        case RaceState::IDLE_WAIT_STOP:
            statusText = "WAIT STOP";
            statusBg = COLOR_YELLOW;
            statusFg = COLOR_BG;
            break;
        case RaceState::ARMED:
            statusText = "READY";
            statusBg = COLOR_GREEN;
            statusFg = COLOR_BG;
            break;
        case RaceState::LAUNCH_DETECTED:
        case RaceState::MEASURING:
            statusText = "LAUNCH";
            statusBg = COLOR_CYAN;
            statusFg = COLOR_BG;
            break;
        case RaceState::FINISHED:
            statusText = "DONE";
            statusBg = COLOR_GREEN;
            statusFg = COLOR_BG;
            break;
        case RaceState::ABORTED:
            statusText = "ABORT";
            statusBg = COLOR_RED;
            statusFg = COLOR_WHITE;
            break;
    }

    _drawPillBadge(226, 3, 78, 18, statusText, statusBg, statusFg);
}

void DisplayEngine::_renderDashboardLive(const GpsData& gps, const ImuData& imu, RaceState raceState) {
    // Цифровой спидометр (огромные цифры)
    int speedVal = (int)(gps.speedKmh + 0.5f);
    char spdStr[16];
    snprintf(spdStr, sizeof(spdStr), "%d", speedVal);

    _canvas.setTextColor(COLOR_WHITE);
    _canvas.setFont(&fonts::Font7);
    _canvas.setTextDatum(textdatum_t::middle_right);
    _canvas.drawString(spdStr, 175, 75);

    // Единица измерения
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.setTextDatum(textdatum_t::middle_left);
    _canvas.drawString("KM/H", 185, 82);

    // Нижняя плашка: G-Force и Пиковое G
    _canvas.fillRoundRect(16, 122, 288, 42, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(16, 122, 288, 42, 6, COLOR_CARD_BORDER);

    // G-LONG
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.setTextDatum(textdatum_t::middle_left);
    _canvas.drawString("G-LONG:", 28, 143);

    char gStr[16];
    snprintf(gStr, sizeof(gStr), "%+.2f G", imu.gLongitudinal);
    _canvas.setTextColor((imu.gLongitudinal >= 0) ? COLOR_GREEN : COLOR_RED);
    _canvas.drawString(gStr, 96, 143);

    // PEAK G
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("PEAK G:", 178, 143);

    char pkStr[16];
    snprintf(pkStr, sizeof(pkStr), "%.2f G", imu.gPeakAccel);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.drawString(pkStr, 242, 143);
}

void DisplayEngine::_renderDragRace(const GpsData& gps, const ImuData& imu, 
                                    RaceState raceState, RaceDiscipline discipline, 
                                    const RunRecord& currentRun, float liveTimeSec) 
{
    // Карточка таймера (левая половина)
    _canvas.fillRoundRect(16, 28, 140, 138, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(16, 28, 140, 138, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.setTextDatum(textdatum_t::top_left);
    _canvas.drawString("DRAG TIMER", 24, 34);

    // Крупный таймер
    char tBuf[16];
    snprintf(tBuf, sizeof(tBuf), "%04.2f s", liveTimeSec);
    _canvas.setFont(&fonts::Font4);
    _canvas.setTextColor((raceState == RaceState::MEASURING) ? COLOR_GREEN : COLOR_WHITE);
    _canvas.drawString(tBuf, 24, 62);

    // Скорость и дистанция заезда
    char spBuf[24];
    snprintf(spBuf, sizeof(spBuf), "Spd: %.1f km/h", gps.speedKmh);
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_LIGHT_GRAY);
    _canvas.drawString(spBuf, 24, 102);

    char dstBuf[24];
    snprintf(dstBuf, sizeof(dstBuf), "Dst: %.1f m", currentRun.totalDistanceM);
    _canvas.drawString(dstBuf, 24, 126);

    // Правая половина: Живые отсечки (Splits)
    _canvas.fillRoundRect(164, 28, 140, 138, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(164, 28, 140, 138, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.drawString("SPLITS", 172, 34);

    char valBuf[16];

    // 0 - 60
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("0-60:", 172, 58);
    if (currentRun.split0_60.achieved) {
        snprintf(valBuf, sizeof(valBuf), "%.2fs", currentRun.split0_60.timeSec);
        _canvas.setTextColor(COLOR_GREEN);
    } else {
        snprintf(valBuf, sizeof(valBuf), "---");
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
    }
    _canvas.drawString(valBuf, 240, 58);

    // 0 - 100
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("0-100:", 172, 80);
    if (currentRun.split0_100.achieved) {
        snprintf(valBuf, sizeof(valBuf), "%.2fs", currentRun.split0_100.timeSec);
        _canvas.setTextColor(COLOR_GREEN);
    } else {
        snprintf(valBuf, sizeof(valBuf), "---");
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
    }
    _canvas.drawString(valBuf, 240, 80);

    // 100 - 200
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("100-200:", 172, 102);
    if (currentRun.split100_200.achieved) {
        snprintf(valBuf, sizeof(valBuf), "%.2fs", currentRun.split100_200.timeSec);
        _canvas.setTextColor(COLOR_GREEN);
    } else {
        snprintf(valBuf, sizeof(valBuf), "---");
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
    }
    _canvas.drawString(valBuf, 240, 102);

    // 1/4 Mile (402m)
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("1/4mi:", 172, 124);
    if (currentRun.split1_4mi.achieved) {
        snprintf(valBuf, sizeof(valBuf), "%.2fs", currentRun.split1_4mi.timeSec);
        _canvas.setTextColor(COLOR_GREEN);
    } else {
        snprintf(valBuf, sizeof(valBuf), "---");
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
    }
    _canvas.drawString(valBuf, 240, 124);
}

void DisplayEngine::_renderBrakeTest(const GpsData& gps, const ImuData& imu, RaceState raceState, const RunRecord& currentRun) {
    _canvas.fillRoundRect(16, 28, 288, 138, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(16, 28, 288, 138, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_RED);
    _canvas.setTextDatum(textdatum_t::top_left);
    _canvas.drawString("BRAKE TEST (100 - 0 KM/H)", 26, 34);

    char buf[40];
    if (currentRun.split100_0.achieved) {
        snprintf(buf, sizeof(buf), "DIST: %.2f m", currentRun.brakeDist100_0M);
        _canvas.setFont(&fonts::Font4);
        _canvas.setTextColor(COLOR_WHITE);
        _canvas.drawString(buf, 26, 65);

        snprintf(buf, sizeof(buf), "TIME: %.2f s", currentRun.split100_0.timeSec);
        _canvas.drawString(buf, 26, 100);
    } else {
        snprintf(buf, sizeof(buf), "LIVE SPD: %.1f km/h", gps.speedKmh);
        _canvas.setFont(&fonts::Font4);
        _canvas.setTextColor(COLOR_WHITE);
        _canvas.drawString(buf, 26, 65);

        snprintf(buf, sizeof(buf), "MAX DECEL: %.2f G", imu.gPeakBrake);
        _canvas.drawString(buf, 26, 100);
    }
}

void DisplayEngine::_renderGMeterScreen(const ImuData& imu) {
    int cx = 82;
    int cy = 96;
    int radius = 48;

    // Сетка G-Bowl
    _canvas.drawCircle(cx, cy, radius, COLOR_CARD_BORDER);
    _canvas.drawCircle(cx, cy, radius / 2, COLOR_CARD_BORDER);
    _canvas.drawFastHLine(cx - radius, cy, radius * 2, COLOR_CARD_BORDER);
    _canvas.drawFastVLine(cx, cy - radius, radius * 2, COLOR_CARD_BORDER);

    // Точка текущей перегрузки
    int dotX = cx + (int)(imu.gLateral * 40.0f);
    int dotY = cy - (int)(imu.gLongitudinal * 40.0f);
    dotX = constrain(dotX, cx - radius + 4, cx + radius - 4);
    dotY = constrain(dotY, cy - radius + 4, cy + radius - 4);

    _canvas.fillCircle(dotX, dotY, 5, COLOR_GREEN);
    _canvas.drawCircle(dotX, dotY, 6, COLOR_WHITE);

    // Числовые показатели справа
    _canvas.fillRoundRect(156, 28, 148, 138, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(156, 28, 148, 138, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.setTextDatum(textdatum_t::top_left);
    _canvas.drawString("G-FORCE", 166, 34);

    char valBuf[16];

    // LONG G
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("LONG G:", 166, 60);
    snprintf(valBuf, sizeof(valBuf), "%+.2f", imu.gLongitudinal);
    _canvas.setTextColor(COLOR_GREEN);
    _canvas.drawString(valBuf, 240, 60);

    // LAT G
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("LAT G:", 166, 86);
    snprintf(valBuf, sizeof(valBuf), "%+.2f", imu.gLateral);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.drawString(valBuf, 240, 86);

    // PEAK G
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("PEAK G:", 166, 112);
    snprintf(valBuf, sizeof(valBuf), "%.2f", imu.gPeakAccel);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.drawString(valBuf, 240, 112);
}

void DisplayEngine::_renderRunResults(const RunRecord& run) {
    // Заголовок экрана результатов
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.setTextDatum(textdatum_t::top_left);
    _canvas.drawString("LAST RUN SUMMARY", 18, 28);

    // Бейдж валидности уклона (Dragy Style)
    if (run.isValidSlope) {
        _drawPillBadge(188, 26, 114, 18, "SLOPE VALID", COLOR_GREEN, COLOR_BG);
    } else {
        _drawPillBadge(188, 26, 114, 18, "INVALID SLOPE", COLOR_RED, COLOR_WHITE);
    }

    // Таблица результатов
    _canvas.fillRoundRect(16, 48, 288, 118, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(16, 48, 288, 118, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    char buf[24];

    // Колонка 1: Скорости
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("0-100:", 24, 54);
    if (run.split0_100.achieved) {
        snprintf(buf, sizeof(buf), "%.2fs", run.split0_100.timeSec);
        _canvas.setTextColor(COLOR_GREEN);
    } else {
        snprintf(buf, sizeof(buf), "---");
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
    }
    _canvas.drawString(buf, 86, 54);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("100-200:", 24, 76);
    if (run.split100_200.achieved) {
        snprintf(buf, sizeof(buf), "%.2fs", run.split100_200.timeSec);
        _canvas.setTextColor(COLOR_GREEN);
    } else {
        snprintf(buf, sizeof(buf), "---");
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
    }
    _canvas.drawString(buf, 86, 76);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("0-60:", 24, 98);
    if (run.split0_60.achieved) {
        snprintf(buf, sizeof(buf), "%.2fs", run.split0_60.timeSec);
        _canvas.setTextColor(COLOR_GREEN);
    } else {
        snprintf(buf, sizeof(buf), "---");
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
    }
    _canvas.drawString(buf, 86, 98);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("1-Foot:", 24, 120);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.drawString(run.rolloutUsed ? "ON" : "OFF", 86, 120);

    // Колонка 2: Драг (1/4 мили, 60ft, Slope, MaxG)
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("1/4mi:", 152, 54);
    if (run.split1_4mi.achieved) {
        snprintf(buf, sizeof(buf), "%.2fs @ %.0f", run.split1_4mi.timeSec, run.split1_4mi.trapSpeedKmh);
        _canvas.setTextColor(COLOR_YELLOW);
    } else {
        snprintf(buf, sizeof(buf), "---");
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
    }
    _canvas.drawString(buf, 208, 54);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("60ft:", 152, 76);
    if (run.split60ft.achieved) {
        snprintf(buf, sizeof(buf), "%.2fs", run.split60ft.timeSec);
        _canvas.setTextColor(COLOR_WHITE);
    } else {
        snprintf(buf, sizeof(buf), "---");
        _canvas.setTextColor(COLOR_LIGHT_GRAY);
    }
    _canvas.drawString(buf, 208, 76);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Slope:", 152, 98);
    snprintf(buf, sizeof(buf), "%+.1f%%", run.slopePct);
    _canvas.setTextColor(run.isValidSlope ? COLOR_GREEN : COLOR_RED);
    _canvas.drawString(buf, 208, 98);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("MaxG:", 152, 120);
    snprintf(buf, sizeof(buf), "%.2f G", run.maxAccelG);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.drawString(buf, 208, 120);
}

void DisplayEngine::_renderGpsInfo(const GpsData& gps) {
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.setTextDatum(textdatum_t::top_left);
    _canvas.drawString("GNSS u-blox M10Q STATUS", 18, 28);

    _canvas.fillRoundRect(16, 48, 288, 118, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(16, 48, 288, 118, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    char buf[28];

    // Колонка 1: Координаты и высоты
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Lat:", 24, 54);
    snprintf(buf, sizeof(buf), "%.5f", gps.lat);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.drawString(buf, 65, 54);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Lon:", 24, 76);
    snprintf(buf, sizeof(buf), "%.5f", gps.lon);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.drawString(buf, 65, 76);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Alt:", 24, 98);
    snprintf(buf, sizeof(buf), "%.1f m", gps.altMSL);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.drawString(buf, 65, 98);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("hAcc:", 24, 120);
    snprintf(buf, sizeof(buf), "+/-%.1fm", gps.hAccM);
    _canvas.setTextColor(COLOR_GREEN);
    _canvas.drawString(buf, 65, 120);

    // Колонка 2: Точность скорости, частота, фикс
    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("SpdAcc:", 155, 54);
    snprintf(buf, sizeof(buf), "+/-%.2f km/h", gps.sAccKmh);
    _canvas.setTextColor(COLOR_GREEN);
    _canvas.drawString(buf, 220, 54);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Rate:", 155, 76);
    snprintf(buf, sizeof(buf), "%d Hz UBX", GPS_UPDATE_RATE_HZ);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.drawString(buf, 220, 76);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Fix:", 155, 98);
    const char* fixStr = (gps.fixType == 3) ? "3D FIX" : (gps.fixType == 4) ? "3D+DR" : "NO FIX";
    _canvas.setTextColor(gps.validFix ? COLOR_GREEN : COLOR_RED);
    _canvas.drawString(fixStr, 220, 98);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("pDOP:", 155, 120);
    snprintf(buf, sizeof(buf), "%.2f", gps.pDOP);
    _canvas.setTextColor(COLOR_WHITE);
    _canvas.drawString(buf, 220, 120);
}

void DisplayEngine::_renderSettings(const DeviceSettings& settings) {
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.setTextDatum(textdatum_t::top_left);
    _canvas.drawString("DEVICE SETTINGS", 18, 28);

    _canvas.fillRoundRect(16, 48, 288, 118, 6, COLOR_CARD_BG);
    _canvas.drawRoundRect(16, 48, 288, 118, 6, COLOR_CARD_BORDER);

    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(COLOR_GRAY);

    _canvas.drawString("1-Foot Rollout:", 24, 56);
    _canvas.setTextColor(settings.use1FootRollout ? COLOR_GREEN : COLOR_RED);
    _canvas.drawString(settings.use1FootRollout ? "ON (Dragy)" : "OFF", 145, 56);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Max Slope Tol:", 24, 80);
    char buf[24];
    snprintf(buf, sizeof(buf), "%.1f%% (NHRA)", settings.slopeTolerancePct);
    _canvas.setTextColor(COLOR_YELLOW);
    _canvas.drawString(buf, 145, 80);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Firmware:", 24, 104);
    _canvas.setTextColor(COLOR_CYAN);
    _canvas.drawString(DRAGON_FW_VERSION, 145, 104);

    _canvas.setTextColor(COLOR_GRAY);
    _canvas.drawString("Btn 9 (Hold): Calibrate IMU", 24, 126);
}

void DisplayEngine::_drawPillBadge(int x, int y, int w, int h, const char* text, uint16_t bgColor, uint16_t textColor) {
    _canvas.fillRoundRect(x, y, w, h, h / 2, bgColor);
    _canvas.setFont(&fonts::Font2);
    _canvas.setTextColor(textColor);
    _canvas.setTextDatum(textdatum_t::middle_center);
    _canvas.drawString(text, x + w / 2, y + h / 2);
    _canvas.setTextDatum(textdatum_t::top_left);
}
