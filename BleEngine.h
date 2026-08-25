#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <functional>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                 BLE ENGINE (NORDIC UART SERVICE - NUS)
 * ============================================================================
 * Высокоскоростной BLE GATT сервер для связи со смартфоном:
 * - Сервис Nordic UART (NUS) для совместимости с Web Bluetooth (Chrome/Edge/iOS)
 * - Потоковая передача телеметрии 15 Гц (скорость, $G_x/G_y/G_z$, спутники, статус)
 * - Мгновенные уведомления о взятии отсечек (0-100, 100-200, 402м)
 * - Передача полных протоколов заездов (RunRecord) с расчетом уклона
 * - Прием команд управления (взведение, сброс, смена дисциплин, калибровка IMU)
 * ============================================================================
 */

// Тип функции обратного вызова для обработки команд со смартфона
typedef std::function<void(const String& cmd, const String& val)> BleCommandHandler;

class BleEngine : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
    BleEngine();

    // Инициализация BLE стека и запуск рекламы (Advertising)
    bool begin(const char* deviceName = BLE_DEVICE_NAME);

    // Проверка статуса подключения смартфона
    bool isConnected() const { return _deviceConnected; }

    // Периодическое обновление (проверка повторного запуска рекламы при дисконнекте)
    void update();

    // Установка обработчика входящих команд
    void setCommandHandler(BleCommandHandler handler) { _cmdHandler = handler; }

    // Отправка живой телеметрии на смартфон
    void sendLiveTelemetry(
        const GpsData& gps,
        const ImuData& imu,
        RaceState state,
        RaceDiscipline disc,
        float liveTimeSec,
        float liveDistanceM,
        float liveSpeedKmh,
        float liveSlopePct,
        float batVolts = 0.0f,
        uint8_t batPct = 0
    );

    // Отправка события моментального взятия отсечки
    void sendSplitEvent(const char* splitName, float timeSec, float trapSpeedKmh);

    // Отправка полного протокола завершенного заезда
    void sendRunRecord(const RunRecord& run);

    // Отправка личных рекордов (Personal Bests)
    void sendPersonalBests(const PersonalBests& pb);

    // Отправка системной информации и настроек прибора
    void sendDeviceInfo(const DeviceSettings& settings, uint8_t runsCount, bool gpsReady, uint8_t sats, float batVolts = 0.0f, uint8_t batPct = 0);

    // Отправка отчета самодиагностики при подключении смартфона
    void sendDiagnostics(
        bool imuOk,
        const char* imuMsg,
        bool gpsOk,
        const char* gpsMsg,
        uint8_t gpsRateHz,
        uint32_t gpsBaud,
        bool storageOk,
        bool batOk,
        float batVolts,
        uint8_t batPct
    );

    // Прямая отправка произвольной JSON-строки
    void sendJson(const char* jsonStr);
    void sendJson(const String& jsonStr);

    // Callbacks BLEServerCallbacks
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;

    // Callbacks BLECharacteristicCallbacks
    void onWrite(BLECharacteristic* pCharacteristic) override;

private:
    BLEServer*          _pServer;
    BLEService*         _pService;
    BLECharacteristic*  _pTxCharacteristic;
    BLECharacteristic*  _pRxCharacteristic;

    bool                _deviceConnected;
    bool                _oldDeviceConnected;
    uint32_t            _lastTxTimeMs;
    BleCommandHandler   _cmdHandler;

    char                _txBuffer[512];
    String              _rxAccumulator;

    void _parseIncomingLine(const String& line);
};
