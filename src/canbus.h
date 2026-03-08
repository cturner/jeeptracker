#pragma once
#include "Particle.h"
#include <mcp_can.h>

// 2024 Jeep Gladiator (JT) CAN-C bus runs at 500 kbps.
// MCP2515 SPI CAN controller wired to:
//   CS  = A2
//   INT = A1
//   SPI bus: MOSI/MISO/SCK (default Particle SPI pins)
//
// Proprietary CAN IDs reverse-engineered by the JL/JT community:
//   0x322  bytes 0-1: engine RPM (big-endian, raw = RPM)
//          bytes 6-7: vehicle speed

#define CAN_CS_PIN   A2
#define CAN_INT_PIN  A1

// Proprietary CAN-C arbitration IDs (Jeep JL/JT platform)
static const uint16_t CAN_ID_RPM_SPEED = 0x322;

// Engine state thresholds
static const uint16_t ENGINE_RUNNING_RPM = 200;

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
    void begin();
    void poll();

    const CanTelemetry &telemetry() const { return _tel; }

private:
    MCP_CAN _can{CAN_CS_PIN};
    CanTelemetry _tel = {};
    bool _initialized = false;

    void handleRpmSpeed(const uint8_t *data, uint8_t dlc);
};
