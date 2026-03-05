#pragma once
#include "Particle.h"

// 2024 Jeep Gladiator (JT) CAN-C bus runs at 500 kbps.
// The serial CAN module (e.g. Longan Labs Serial CAN Bus Module)
// connects via UART and exchanges ASCII frames in the format:
//   Receive: "T<id><dlc><data>\r"  (extended) or "t<id><dlc><data>\r" (std)
//   Send:    same format written to serial
//
// Proprietary CAN IDs reverse-engineered by the JL/JT community:
//   0x322  bytes 0-1: engine RPM (big-endian, raw = RPM)
//          bytes 6-7: vehicle speed

// Proprietary CAN-C arbitration IDs (Jeep JL/JT platform)
static const uint16_t CAN_ID_RPM_SPEED = 0x322;

// Engine state thresholds
static const uint16_t ENGINE_RUNNING_RPM   = 200;

enum class EngineState : uint8_t {
    OFF,
    RUNNING
};

struct CanTelemetry {
    uint16_t rpm;
    uint16_t speedRaw;
    EngineState engineState;
    uint32_t lastRpmMs;
};

class CanBus {
public:
    // canSerial: the HardwareSerial port connected to the CAN module
    // (e.g. Serial2 on Particle Boron — GPS is on Serial1)
    void begin(USARTSerial &canSerial, uint32_t baud = 115200);
    void poll();

    const CanTelemetry &telemetry() const { return _tel; }

private:
    USARTSerial *_serial = nullptr;
    CanTelemetry _tel = {};

    char _rxBuf[64];
    uint8_t _rxLen = 0;

    void processLine(const char *line, uint8_t len);
    bool parseFrame(const char *line, uint8_t len,
                    uint16_t &id, uint8_t &dlc, uint8_t *data);
    void handleRpmSpeed(const uint8_t *data, uint8_t dlc);

    static uint8_t hexNibble(char c);
    static uint8_t hexByte(const char *p);
};
