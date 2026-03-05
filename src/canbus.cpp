#include "canbus.h"

void CanBus::begin(USARTSerial &canSerial, uint32_t baud) {
    _serial = &canSerial;
    _serial->begin(baud);
    _rxLen = 0;
    memset(&_tel, 0, sizeof(_tel));
    _tel.engineState = EngineState::OFF;

    // Configure the serial CAN module for 500 kbps (Jeep CAN-C bus speed).
    // Longan Labs Serial CAN Bus Module AT commands:
    //   "AT+C=[rate]\r\n"  where rate: 0=5k,6=250k,7=500k,8=1M
    delay(100);
    _serial->print("AT+C=7\r\n");
    delay(100);
    // Open the CAN channel
    _serial->print("AT+O\r\n");
    delay(100);

    Serial.println("[CAN] initialized at 500 kbps");
}

void CanBus::poll() {
    // Read incoming bytes and assemble lines
    while (_serial->available()) {
        char c = _serial->read();
        if (c == '\r' || c == '\n') {
            if (_rxLen > 0) {
                _rxBuf[_rxLen] = '\0';
                processLine(_rxBuf, _rxLen);
                _rxLen = 0;
            }
        } else if (_rxLen < sizeof(_rxBuf) - 1) {
            _rxBuf[_rxLen++] = c;
        } else {
            _rxLen = 0; // overflow, discard
        }
    }

    // Periodically request throttle position via OBD-II
    if (millis() - _lastThrottleReqMs >= THROTTLE_POLL_MS) {
        _lastThrottleReqMs = millis();
        requestThrottle();
    }
}

// Parse a completed line from the serial CAN module.
// Standard frame format: "t<3-char hex id><1-char dlc><hex data bytes>"
// Example:  "t3228044304430000AABB"
//            t 322 8 04430443...
void CanBus::processLine(const char *line, uint8_t len) {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];

    if (!parseFrame(line, len, id, dlc, data)) {
        return;
    }

    if (id == CAN_ID_RPM_SPEED) {
        handleRpmSpeed(data, dlc);
    } else if (id == CAN_ID_OBD_RESPONSE) {
        handleObdResponse(data, dlc);
    }
}

bool CanBus::parseFrame(const char *line, uint8_t len,
                        uint16_t &id, uint8_t &dlc, uint8_t *data) {
    // Minimum: 't' + 3 id chars + 1 dlc char = 5
    if (len < 5) return false;

    char type = line[0];
    if (type != 't' && type != 'T') return false;

    bool extended = (type == 'T');
    uint8_t idChars = extended ? 8 : 3;

    if (len < (uint8_t)(1 + idChars + 1)) return false;

    // Parse arbitration ID
    id = 0;
    for (uint8_t i = 0; i < idChars; i++) {
        id = (id << 4) | hexNibble(line[1 + i]);
    }

    // Parse DLC
    dlc = hexNibble(line[1 + idChars]);
    if (dlc > 8) return false;

    // Check we have enough hex chars for the data
    uint8_t dataStart = 1 + idChars + 1;
    if (len < dataStart + dlc * 2) return false;

    for (uint8_t i = 0; i < dlc; i++) {
        data[i] = hexByte(&line[dataStart + i * 2]);
    }
    return true;
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

void CanBus::handleObdResponse(const uint8_t *data, uint8_t dlc) {
    // OBD-II response format: [numBytes, 0x41, PID, A, B, ...]
    if (dlc < 4) return;
    if (data[1] != 0x41) return; // not a Mode 01 response

    uint8_t pid = data[2];

    if (pid == OBD_PID_THROTTLE && dlc >= 4) {
        // Throttle position: A * 100 / 255
        _tel.throttlePct = data[3] * 100.0f / 255.0f;
        _tel.lastThrottleMs = millis();
        Serial.printlnf("[CAN] Throttle: %.1f%%", (double)_tel.throttlePct);
    } else if (pid == OBD_PID_RPM && dlc >= 5) {
        // RPM from OBD: ((A * 256) + B) / 4
        uint16_t obdRpm = (((uint16_t)data[3] << 8) | data[4]) / 4;
        Serial.printlnf("[CAN] OBD RPM: %u", obdRpm);
        // Only use OBD RPM if we haven't seen 0x322 recently
        if (millis() - _tel.lastRpmMs > 2000) {
            _tel.rpm = obdRpm;
            _tel.lastRpmMs = millis();
        }
    }
}

void CanBus::requestThrottle() {
    // OBD-II request: 2 bytes payload, Mode 01, PID 0x11
    uint8_t frame[8] = {0x02, OBD_MODE_CURRENT, OBD_PID_THROTTLE,
                        0x00, 0x00, 0x00, 0x00, 0x00};
    sendFrame(CAN_ID_OBD_REQUEST, 8, frame);
}

void CanBus::sendFrame(uint16_t id, uint8_t dlc, const uint8_t *data) {
    if (!_serial) return;

    // Format: "t<3 hex id><dlc><hex data>\r"
    char buf[32];
    int pos = snprintf(buf, sizeof(buf), "t%03X%01X", id, dlc);
    for (uint8_t i = 0; i < dlc; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X", data[i]);
    }
    buf[pos++] = '\r';
    buf[pos] = '\0';
    _serial->print(buf);
}

uint8_t CanBus::hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

uint8_t CanBus::hexByte(const char *p) {
    return (hexNibble(p[0]) << 4) | hexNibble(p[1]);
}
