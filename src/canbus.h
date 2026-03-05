#pragma once
#include "Particle.h"

// 2024 Jeep Gladiator (JT) CAN-C bus runs at 500 kbps.
// The MCP2515 serial CAN module (e.g. Longan Labs Serial CAN Bus Module)
// connects via UART and exchanges ASCII frames in the format:
//   Receive: "T<id><dlc><data>\r"  (extended) or "t<id><dlc><data>\r" (std)
//   Send:    same format written to serial
//
// Proprietary CAN IDs reverse-engineered by the JL/JT community:
//   0x322  bytes 0-1: engine RPM (big-endian, raw = RPM)
//          bytes 6-7: vehicle speed
//
// Standard OBD-II (request 0x7DF, response 0x7E8):
//   Mode 01 PID 0x11: throttle position (byte A * 100/255 = %)

// Proprietary CAN-C arbitration IDs (Jeep JL/JT platform)
static const uint16_t CAN_ID_RPM_SPEED = 0x322;

// Standard OBD-II arbitration IDs
static const uint16_t CAN_ID_OBD_REQUEST   = 0x7DF;
static const uint16_t CAN_ID_OBD_RESPONSE  = 0x7E8;

// OBD-II Mode 01 PIDs
static const uint8_t OBD_MODE_CURRENT      = 0x01;
static const uint8_t OBD_PID_RPM           = 0x0C;
static const uint8_t OBD_PID_THROTTLE      = 0x11;

// Engine state thresholds
static const uint16_t ENGINE_RUNNING_RPM   = 200;

enum class EngineState : uint8_t {
    OFF,
    RUNNING
};

struct CanTelemetry {
    uint16_t rpm;
    uint16_t speedRaw;
    float    throttlePct;
    EngineState engineState;
    uint32_t lastRpmMs;
    uint32_t lastThrottleMs;
};

class CanBus {
public:
    // canSerial: the HardwareSerial port connected to the CAN module
    // (e.g. Serial2 on Particle — GPS is on Serial1)
    void begin(USARTSerial &canSerial, uint32_t baud = 115200);
    void poll();

    const CanTelemetry &telemetry() const { return _tel; }

private:
    USARTSerial *_serial = nullptr;
    CanTelemetry _tel = {};

    char _rxBuf[64];
    uint8_t _rxLen = 0;

    uint32_t _lastThrottleReqMs = 0;
    static const uint32_t THROTTLE_POLL_MS = 1000;

    void processLine(const char *line, uint8_t len);
    bool parseFrame(const char *line, uint8_t len,
                    uint16_t &id, uint8_t &dlc, uint8_t *data);
    void handleRpmSpeed(const uint8_t *data, uint8_t dlc);
    void handleObdResponse(const uint8_t *data, uint8_t dlc);
    void requestThrottle();
    void sendFrame(uint16_t id, uint8_t dlc, const uint8_t *data);

    static uint8_t hexNibble(char c);
    static uint8_t hexByte(const char *p);
};
