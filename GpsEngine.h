#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *              GPS ENGINE (u-blox M10Q UBX + NMEA HYBRID PARSER)
 * ============================================================================
 * Универсальный высокопроизводительный драйвер для u-blox M10Q / M9N / M8N:
 * - Автоматический поиск скорости порта (38400, 115200, 9600, 460800)
 * - Поддержка конфигурации u-blox Gen 10 (UBX-CFG-VALSET) и Gen 8/9 (UBX-CFG)
 * - Парсинг бинарного протокола UBX-NAV-PVT (10-18 Гц, Doppler 3D)
 * - Резервный встроенный парсер NMEA ($GNRMC, $GNGGA, $GNGSA)
 * - Диагностика связи и подсчет принятых байт в реальном времени
 */

class GpsEngine {
public:
    GpsEngine();
    
    // Инициализация, автоопределение скорости и отправка конфигурации
    bool begin(HardwareSerial& serialPort, uint32_t targetBaud = 115200);
    
    // Потоковый разбор входящих байт (UBX + NMEA)
    bool update();
    
    // Получить последние разобранные данные
    const GpsData& getData() const { return _data; }
    
    // Проверить качество связи (готовность к замерам)
    bool isReadyForRace() const;
    
    // Диагностика
    uint32_t getRxByteCount() const { return _rxByteCount; }
    uint32_t getPacketCount() const { return _packetCount; }
    uint32_t getDetectedBaud() const { return _detectedBaud; }
    bool isReceivingBytes() const { return (millis() - _lastByteTimeMs < 1000) && (_rxByteCount > 10); }

    // Принудительная отправка конфигурации UBX M10
    void configureUblox();

private:
    HardwareSerial* _serial;
    GpsData _data;
    uint32_t _detectedBaud;
    uint32_t _rxByteCount;
    uint32_t _packetCount;
    uint32_t _lastByteTimeMs;

    // Состояния парсера UBX
    enum UbxParserState {
        WAIT_SYNC1,
        WAIT_SYNC2,
        WAIT_CLASS,
        WAIT_ID,
        WAIT_LEN_L,
        WAIT_LEN_H,
        PAYLOAD,
        WAIT_CKA,
        WAIT_CKB
    };

    UbxParserState _ubxState;
    uint8_t _msgClass;
    uint8_t _msgId;
    uint16_t _payloadLen;
    uint16_t _payloadCounter;
    uint8_t _ckA, _ckB;
    uint8_t _calcCkA, _calcCkB;
    uint8_t _payloadBuf[128];

    // Буфер парсера NMEA
    char _nmeaBuf[120];
    uint8_t _nmeaIdx;
    bool _inNmea;

    void _processUbxPayload();
    void _processNmeaSentence(const char* sentence);
    void _sendUbxCommand(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);
    void _calculateChecksum(uint8_t byte, uint8_t& cka, uint8_t& ckb);
};
