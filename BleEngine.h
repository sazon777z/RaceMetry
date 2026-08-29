#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                 BLE ENGINE (NORDIC UART SERVICE - NUS)
 * ============================================================================
 * Высокоскоростной BLE GATT сервер для связи со смартфоном:
 * - Сервис Nordic UART (NUS) для совместимости с Web Bluetooth
 * - Потоковая передача телеметрии 15 Гц
 * - Неблокирующая очередь входящих команд (изоляция контекста BLE стека)
 * - Потокобезопасная отправка с защитой буфера передачи (TX Mutex)
 */

struct BleCommand {
    char cmd[24];
    char val[32];
};

class BleEngine : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
    BleEngine();

    // Инициализация BLE стека и запуск рекламы
    bool begin(const char* deviceName = BLE_DEVICE_NAME);

    // Проверка статуса подключения смартфона
    bool isConnected() const { return _deviceConnected; }

    // Двусторонний канал подтверждён командой ping после подписки клиента
    bool isClientReady() const { return _clientReady; }
    void setClientReady(bool ready) { _clientReady = ready; }

    // Периодическое обновление (обслуживание рекламы при дисконнекте)
    void update();

    // Неблокирующая выборка поступившей команды
    bool popCommand(BleCommand& cmd);

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

    // Отправка события отсечки
    void sendSplitEvent(const char* splitName, float timeSec, float trapSpeedKmh);
    void sendSplitEvent(const SplitEvent& evt);

    // Отправка полного протокола завершенного заезда
    void sendRunRecord(const RunRecord& run);

    // Отправка личных рекордов (Personal Bests)
    void sendPersonalBests(const PersonalBests& pb);

    // Отправка системной информации и настроек прибора
    void sendDeviceInfo(const DeviceSettings& settings, uint8_t runsCount, bool gpsReady, uint8_t sats, float batVolts = 0.0f, uint8_t batPct = 0);

    // Отправка отчета самодиагностики
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

    // Обратные вызовы BLE сервера (подключение/отключение)
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;

    // Обратный вызов на запись в RX характеристику
    void onWrite(BLECharacteristic* pCharacteristic) override;

private:
    BLEServer*          _pServer;
    BLEService*         _pService;
    BLECharacteristic*  _pTxCharacteristic;
    BLECharacteristic*  _pRxCharacteristic;

    volatile bool       _deviceConnected;
    bool                _oldDeviceConnected;
    volatile bool       _clientReady;

    QueueHandle_t       _cmdQueue;
    SemaphoreHandle_t   _txMutex;

    char                _txBuffer[512];
    char                _rxAccumulator[256];
    size_t              _rxAccumulatorLen;

    void _parseIncomingLine(const char* line);
};
