#pragma once
#include <Arduino.h>
#include "Config.h"
#include "Types.h"

/**
 * ============================================================================
 *                          DRAGon TELEMETRY METER
 *                    GPS ENGINE (u-blox M10Q UBX PARSER)
 * ============================================================================
 * Драйвер скоростного бинарного протокола u-blox UBX.
 * Осуществляет автоматическую настройку чипа M10 на частоту 10-18 Гц,
 * установку динамической модели "Automotive", отключение NMEA и
 * прямое извлечение 3D Doppler скорости и геодезической высоты.
 */

class GpsEngine {
public:
    GpsEngine();
    
    // Инициализация UART и отправка конфигурации u-blox
    bool begin(HardwareSerial& serialPort, uint32_t targetBaud = GPS_BAUDRATE_TARGET);
    
    // Потоковый разбор входящих байт (вызывать в скоростном цикле Core 0)
    // Возвращает true, когда разобран новый полный навигационный пакет UBX-NAV-PVT
    bool update();
    
    // Получить последние разобранные данные
    const GpsData& getData() const { return _data; }
    
    // Проверить качество связи (достаточно ли спутников и 3D-фикса для замера)
    bool isReadyForRace() const;
    
    // Принудительная отправка конфигурации UBX
    void configureUblox();

private:
    HardwareSerial* _serial;
    GpsData _data;
    
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

    UbxParserState _state;
    uint8_t _msgClass;
    uint8_t _msgId;
    uint16_t _payloadLen;
    uint16_t _payloadCounter;
    uint8_t _ckA, _ckB;
    uint8_t _calcCkA, _calcCkB;
    
    uint8_t _payloadBuf[128]; // Буфер полезной нагрузки UBX-NAV-PVT (92 байта)

    void _processPayload();
    void _sendUbxCommand(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);
    void _calculateChecksum(uint8_t byte, uint8_t& cka, uint8_t& ckb);
};
