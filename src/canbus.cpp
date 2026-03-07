#include "canbus.h"

void CanBus::begin() {
    pinMode(CAN_INT_PIN, INPUT);
    memset(&_tel, 0, sizeof(_tel));
    _tel.engineState = EngineState::OFF;

    // Initialize MCP2515: accept all IDs, 500 kbps, 8 MHz crystal
    if (_can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("[CAN] MCP2515 initialized at 500 kbps");
    } else {
        Serial.println("[CAN] MCP2515 init FAILED");
    }

    // Must switch from default loopback mode to normal mode
    _can.setMode(MCP_NORMAL);
}

void CanBus::poll() {
    // Check INT pin (active low) for pending messages
    if (digitalRead(CAN_INT_PIN) == LOW) {
        long unsigned int rxId;
        unsigned char dlc = 0;
        unsigned char data[8];

        _can.readMsgBuf(&rxId, &dlc, data);

        // Mask off extended/RTR flag bits to get the raw arbitration ID
        uint16_t id = (uint16_t)(rxId & 0x7FF);

        if (id == CAN_ID_RPM_SPEED) {
            handleRpmSpeed(data, dlc);
        }
    }
}

void CanBus::handleRpmSpeed(const uint8_t *data, uint8_t dlc) {
    if (dlc < 2) return;

    uint16_t newRpm = ((uint16_t)data[0] << 8) | data[1];
    EngineState prevState = _tel.engineState;
    _tel.rpm = newRpm;
    _tel.lastRpmMs = millis();

    if (dlc >= 8) {
        _tel.speedRaw = ((uint16_t)data[6] << 8) | data[7];
    }

    // Detect engine start/stop transitions
    if (newRpm >= ENGINE_RUNNING_RPM) {
        _tel.engineState = EngineState::RUNNING;
    } else {
        _tel.engineState = EngineState::OFF;
    }

    if (prevState != _tel.engineState) {
        if (_tel.engineState == EngineState::RUNNING) {
            Serial.printlnf("[CAN] ENGINE START detected (RPM: %u)", newRpm);
        } else {
            Serial.printlnf("[CAN] ENGINE STOP detected (RPM: %u)", newRpm);
        }
    }
}
