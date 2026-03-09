/* 
 * JeepTracker — GPS tracker with power management + stolen mode
 * 
 * Modes:
 *   DRIVING  — engine on or USB power present. GPS always on, publishes
 *              every 10s in motion / 60s idle. CAN polling active.
 *   SENTRY   — engine off, on battery, server mode "normal". Sleeps for
 *              server-specified interval (default 30 min), wakes to grab
 *              a fix, publish, poll server config, then sleeps again.
 *   STOLEN   — server mode "stolen". Same wake cycle but short interval
 *              (default 60s), adapts based on battery level to maximize
 *              tracking duration.
 *
 * Config polling: after each sentry/stolen publish the device publishes a
 * "config_request" event. A Particle webhook GETs our server, and the
 * response comes back via "hook-response/config_request" with JSON:
 *   {"mode":"normal|stolen","interval_s":1800}
 */

#include "Particle.h"
#include "Adafruit_GPS.h"
#include "canbus.h"

// ── GPS ─────────────────────────────────────────────────────────────────
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

// ── CAN ─────────────────────────────────────────────────────────────────
CanBus canbus;

// ── Timing (driving mode) ───────────────────────────────────────────────
uint32_t timer = 0;
uint32_t lastPublish = 0;
uint32_t lastCanPublish = 0;
bool firstFixPublished = false;

const uint32_t PUBLISH_INTERVAL_MS      = 10000;  // 10s when moving
const uint32_t IDLE_PUBLISH_INTERVAL_MS = 60000;  // 60s when stationary
const uint32_t CAN_PUBLISH_INTERVAL_MS  = 10000;
const float    MOTION_THRESHOLD_MPH     = 2.0f;

// ── Power management ────────────────────────────────────────────────────
enum class TrackerMode : uint8_t { DRIVING, SENTRY, STOLEN };
TrackerMode currentMode = TrackerMode::DRIVING;

const uint32_t ENGINE_OFF_DEBOUNCE_MS = 120000;  // 2 min debounce before sleeping
uint32_t       engineOffSince = 0;               // millis() when engine stopped + no USB
bool           engineOffPending = false;

// Sentry/stolen defaults (overridden by server)
int      sentryIntervalS = 1800;   // 30 min
int      stolenIntervalS = 60;     // 1 min
bool     serverSaysStolen = false;

// Config response handling
volatile bool   configReceived = false;
volatile int    configIntervalS = 1800;
volatile bool   configStolen = false;
const uint32_t  CONFIG_WAIT_MS = 10000;  // max wait for webhook response

// GPS fix timeout during sentry wake
const uint32_t GPS_FIX_TIMEOUT_MS = 90000;

// Battery critical threshold — hibernate and stop publishing
const float BATTERY_CRITICAL_PCT = 8.0f;

// Last known good position (for stale publishes)
float lastLat = 0, lastLon = 0, lastAlt = 0, lastHdg = 0;
bool  hasLastPosition = false;

// ── Cell tower info ─────────────────────────────────────────────────────
struct CellInfo {
    int mcc;
    int mnc;
    int lac;
    int cid;
    int rssi;
    bool valid;
};
CellInfo cellInfo = {0, 0, 0, 0, 0, false};

// ── System config ───────────────────────────────────────────────────────
SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);

// ── Forward declarations ────────────────────────────────────────────────
float metersToFeet(float meters);
float knotsToMph(float knots);
bool  isInMotion();
bool  hasPower();
bool  isEngineRunning();
bool  publishLocation(bool stale = false);
bool  publishCanTelemetry();
void  pollServerConfig();
void  configResponseHandler(const char *event, const char *data);
void  enterSentry();
void  sentryWakeCycle();
int   getSleepInterval();
void  gpsStandby();
void  gpsWake();
void  readCellInfo();


// ── Helpers ─────────────────────────────────────────────────────────────

float metersToFeet(float meters) {
    return meters * 3.28084f;
}

float knotsToMph(float knots) {
    return knots * 1.150779f;
}

bool isInMotion() {
    return GPS.fix && knotsToMph(GPS.speed) >= MOTION_THRESHOLD_MPH;
}

bool hasPower() {
    int ps = System.powerSource();
    return (ps != POWER_SOURCE_BATTERY && ps != POWER_SOURCE_UNKNOWN);
}

bool isEngineRunning() {
    return canbus.telemetry().engineState == EngineState::RUNNING;
}


// ── GPS power management ────────────────────────────────────────────────

void gpsStandby() {
    // PMTK161,0 = standby mode (~50µA, retains almanac for warm fix)
    // GPS wakes on any serial input
    GPSSerial.println("$PMTK161,0*28");
    delay(100);
    Serial.println("[GPS] Entered standby mode");
}

void gpsWake() {
    // Any serial data wakes the GPS from standby
    // Send a dummy byte then reconfigure
    GPSSerial.println("");
    delay(250);  // give it a moment to wake
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    Serial.println("[GPS] Woke from standby");
}


// ── Cell tower info ─────────────────────────────────────────────────────

// Callback buffer for AT command response
static char atResponseBuf[256];
static int atResponseLen = 0;

int atCallback(int type, const char* buf, int len, char* out) {
    if (type == TYPE_PLUS) {
        // Copy response data, leave room for null
        int copyLen = (len < 255) ? len : 255;
        memcpy(out + atResponseLen, buf, copyLen);
        atResponseLen += copyLen;
        out[atResponseLen] = '\0';
    }
    return WAIT;
}

void readCellInfo() {
    cellInfo.valid = false;
    atResponseLen = 0;
    memset(atResponseBuf, 0, sizeof(atResponseBuf));

    // AT+COPS? returns current operator: +COPS: <mode>,<format>,<oper>,<AcT>
    // AT+CREG? returns registration: +CREG: <n>,<stat>[,<lac>,<ci>]
    // We use CREG for LAC+CID, and COPS for MCC/MNC

    // Get EPS registration info (LAC + Cell ID)
    // LTE Cat M1 uses AT+CEREG, not AT+CREG
    int ret = Cellular.command(atCallback, atResponseBuf, 5000, "AT+CEREG=2\r\n");
    if (ret != RESP_OK) {
        Serial.println("[CELL] CEREG=2 failed");
    }
    delay(100);

    atResponseLen = 0;
    memset(atResponseBuf, 0, sizeof(atResponseBuf));
    ret = Cellular.command(atCallback, atResponseBuf, 5000, "AT+CEREG?\r\n");

    if (ret == RESP_OK && atResponseLen > 0) {
        // Parse: +CEREG: 2,1,"XXXX","XXXXXXXX",7
        char *p = strstr(atResponseBuf, "+CEREG:");
        if (p) {
            int n, stat;
            char lacStr[8] = {0}, cidStr[16] = {0};
            int parsed = sscanf(p, "+CEREG: %d,%d,\"%7[^\"]\",\"%15[^\"]\"", &n, &stat, lacStr, cidStr);
            if (parsed >= 4) {
                cellInfo.lac = (int)strtol(lacStr, NULL, 16);
                cellInfo.cid = (int)strtol(cidStr, NULL, 16);
                Serial.printlnf("[CELL] LAC=%d CID=%d", cellInfo.lac, cellInfo.cid);
            }
        }
    }

    // Fallback to CREG if CEREG didn't return LAC/CID (2G/3G fallback)
    if (cellInfo.lac == 0 && cellInfo.cid == 0) {
        atResponseLen = 0;
        memset(atResponseBuf, 0, sizeof(atResponseBuf));
        Cellular.command(atCallback, atResponseBuf, 5000, "AT+CREG=2\r\n");
        delay(100);
        atResponseLen = 0;
        memset(atResponseBuf, 0, sizeof(atResponseBuf));
        ret = Cellular.command(atCallback, atResponseBuf, 5000, "AT+CREG?\r\n");
        if (ret == RESP_OK && atResponseLen > 0) {
            char *p = strstr(atResponseBuf, "+CREG:");
            if (p) {
                int n, stat;
                char lacStr[8] = {0}, cidStr[16] = {0};
                int parsed = sscanf(p, "+CREG: %d,%d,\"%7[^\"]\",\"%15[^\"]\"", &n, &stat, lacStr, cidStr);
                if (parsed >= 4) {
                    cellInfo.lac = (int)strtol(lacStr, NULL, 16);
                    cellInfo.cid = (int)strtol(cidStr, NULL, 16);
                    Serial.printlnf("[CELL] LAC=%d CID=%d (via CREG)", cellInfo.lac, cellInfo.cid);
                }
            }
        }
    }

    // Get operator info (MCC+MNC)
    atResponseLen = 0;
    memset(atResponseBuf, 0, sizeof(atResponseBuf));
    ret = Cellular.command(atCallback, atResponseBuf, 5000, "AT+COPS=3,2\r\n");
    delay(100);

    atResponseLen = 0;
    memset(atResponseBuf, 0, sizeof(atResponseBuf));
    ret = Cellular.command(atCallback, atResponseBuf, 5000, "AT+COPS?\r\n");

    if (ret == RESP_OK && atResponseLen > 0) {
        // Parse: +COPS: 0,2,"310410",7
        char *p = strstr(atResponseBuf, "+COPS:");
        if (p) {
            char operStr[16] = {0};
            int mode, fmt;
            int parsed = sscanf(p, "+COPS: %d,%d,\"%15[^\"]\"", &mode, &fmt, operStr);
            if (parsed >= 3 && strlen(operStr) >= 5) {
                // MCC is first 3 digits, MNC is rest
                char mccStr[4] = {0};
                strncpy(mccStr, operStr, 3);
                cellInfo.mcc = atoi(mccStr);
                cellInfo.mnc = atoi(operStr + 3);
                Serial.printlnf("[CELL] MCC=%d MNC=%d", cellInfo.mcc, cellInfo.mnc);
            }
        }
    }

    // Get signal strength
    atResponseLen = 0;
    memset(atResponseBuf, 0, sizeof(atResponseBuf));
    ret = Cellular.command(atCallback, atResponseBuf, 5000, "AT+CSQ\r\n");

    if (ret == RESP_OK && atResponseLen > 0) {
        char *p = strstr(atResponseBuf, "+CSQ:");
        if (p) {
            int csq, ber;
            if (sscanf(p, "+CSQ: %d,%d", &csq, &ber) >= 1 && csq != 99) {
                // Convert CSQ to approximate dBm: dBm = -113 + 2*csq
                cellInfo.rssi = -113 + 2 * csq;
                Serial.printlnf("[CELL] RSSI=%d dBm (CSQ=%d)", cellInfo.rssi, csq);
            }
        }
    }

    // Valid if we got at least LAC and CID
    cellInfo.valid = (cellInfo.lac > 0 && cellInfo.cid > 0);
    if (cellInfo.valid) {
        Serial.printlnf("[CELL] Info: MCC=%d MNC=%d LAC=%d CID=%d RSSI=%d",
            cellInfo.mcc, cellInfo.mnc, cellInfo.lac, cellInfo.cid, cellInfo.rssi);
    } else {
        Serial.println("[CELL] Could not read cell info");
    }
}


// ── Publish ─────────────────────────────────────────────────────────────

bool publishLocation(bool stale) {
    if (!Particle.connected()) return false;

    float lat, lon, alt, hdg, spd;
    int sat;

    if (GPS.fix && !stale) {
        lat = GPS.latitudeDegrees;
        lon = GPS.longitudeDegrees;
        alt = metersToFeet(GPS.altitude);
        hdg = GPS.angle;
        spd = knotsToMph(GPS.speed);
        sat = GPS.satellites;
        // Save as last known
        lastLat = lat; lastLon = lon; lastAlt = alt; lastHdg = hdg;
        hasLastPosition = true;
    } else if (hasLastPosition) {
        lat = lastLat; lon = lastLon; alt = lastAlt; hdg = lastHdg;
        spd = 0; sat = 0;
    } else {
        return false;  // no fix ever, nothing to publish
    }

    const CanTelemetry &can = canbus.telemetry();
    float battPct = System.batteryCharge();
    FuelGauge fuel;
    float battV = fuel.getVCell();

    // Read cell tower info (modem is already connected, ~10ms)
    readCellInfo();

    // Build payload — keep under 1024 byte Particle.publish limit
    char buf[512];
    int len;

    if (cellInfo.valid) {
        len = snprintf(buf, sizeof(buf),
            "{\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,\"alt\":%.1f,\"hdg\":%.1f,"
            "\"sat\":%d,\"rpm\":%u,\"bat\":%.0f,\"batv\":%.2f,\"pwr\":%d,"
            "\"cell\":{\"mcc\":%d,\"mnc\":%d,\"lac\":%d,\"cid\":%d,\"rssi\":%d}"
            "%s}",
            lat, lon, spd, alt, hdg, sat,
            (unsigned)can.rpm,
            battPct, battV,
            hasPower() ? 1 : 0,
            cellInfo.mcc, cellInfo.mnc, cellInfo.lac, cellInfo.cid, cellInfo.rssi,
            stale ? ",\"stale\":true" : "");
    } else {
        len = snprintf(buf, sizeof(buf),
            "{\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,\"alt\":%.1f,\"hdg\":%.1f,"
            "\"sat\":%d,\"rpm\":%u,\"bat\":%.0f,\"batv\":%.2f,\"pwr\":%d%s}",
            lat, lon, spd, alt, hdg, sat,
            (unsigned)can.rpm,
            battPct, battV,
            hasPower() ? 1 : 0,
            stale ? ",\"stale\":true" : "");
    }

    Particle.publish("location", buf, PRIVATE);
    Serial.printlnf("[PUB] %s", buf);
    return true;
}

bool publishCanTelemetry() {
    if (!Particle.connected()) return false;
    const CanTelemetry &can = canbus.telemetry();
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"rpm\":%u,\"eng\":\"%s\"}",
        (unsigned)can.rpm,
        can.engineState == EngineState::RUNNING ? "on" : "off");
    Particle.publish("canbus", buf, PRIVATE);
    Serial.printlnf("[CAN] %s", buf);
    return true;
}


// ── Server config polling ───────────────────────────────────────────────

void configResponseHandler(const char *event, const char *data) {
    String s(data);
    Serial.printlnf("[CFG] Response: %s", data);

    configStolen = (s.indexOf("\"stolen\"") >= 0);

    int idx = s.indexOf("\"interval_s\":");
    if (idx >= 0) {
        int val = atoi(s.c_str() + idx + 13);
        if (val > 0) configIntervalS = val;
    }
    configReceived = true;
}

void pollServerConfig() {
    configReceived = false;
    float battPct = System.batteryCharge();
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"bat\":%.0f}", battPct);
    Particle.publish("config_request", buf, PRIVATE);
    Serial.printlnf("[CFG] Requesting config (bat=%.0f)", battPct);

    uint32_t start = millis();
    while (!configReceived && (millis() - start < CONFIG_WAIT_MS)) {
        Particle.process();
        delay(100);
    }

    if (configReceived) {
        serverSaysStolen = configStolen;
        if (serverSaysStolen) {
            stolenIntervalS = configIntervalS;
            Serial.printlnf("[CFG] STOLEN mode, interval=%ds", stolenIntervalS);
        } else {
            sentryIntervalS = configIntervalS;
            Serial.printlnf("[CFG] Normal mode, interval=%ds", sentryIntervalS);
        }
    } else {
        Serial.println("[CFG] No response, using previous config");
    }
}


// ── Sleep / sentry ──────────────────────────────────────────────────────

int getSleepInterval() {
    if (serverSaysStolen) return stolenIntervalS;
    return sentryIntervalS;
}

void sentryWakeCycle() {
    Serial.println("[SENTRY] Wake cycle starting");

    // Wake GPS from standby (warm fix, almanac retained)
    gpsWake();

    // Try to get a GPS fix (up to 90s)
    bool gotFix = false;
    uint32_t fixStart = millis();
    while (millis() - fixStart < GPS_FIX_TIMEOUT_MS) {
        GPS.read();
        if (GPS.newNMEAreceived()) {
            GPS.parse(GPS.lastNMEA());
        }
        if (GPS.fix) {
            gotFix = true;
            Serial.printlnf("[SENTRY] GPS fix in %lums, %d sats",
                millis() - fixStart, GPS.satellites);
            break;
        }
        Particle.process();
        delay(10);
    }

    // Wait for cloud connection if not already
    if (!Particle.connected()) {
        Serial.println("[SENTRY] Waiting for cloud...");
        uint32_t cloudStart = millis();
        while (!Particle.connected() && (millis() - cloudStart < 30000)) {
            Particle.process();
            delay(100);
        }
    }

    if (Particle.connected()) {
        publishLocation(!gotFix);
        delay(1100);  // rate limit: 1 publish/sec
        pollServerConfig();
    } else {
        Serial.println("[SENTRY] Cloud connect failed, sleeping with defaults");
    }

    // Check if we should return to driving mode
    if (hasPower() || isEngineRunning()) {
        Serial.println("[SENTRY] Power detected, returning to DRIVING");
        currentMode = TrackerMode::DRIVING;
        engineOffPending = false;
        firstFixPublished = false;
        canbus.begin();
        return;  // back to loop(), GPS stays awake
    }

    // Check critical battery
    float batt = System.batteryCharge();
    if (batt >= 0 && batt < BATTERY_CRITICAL_PCT) {
        Serial.printlnf("[SENTRY] Battery critical (%.0f%%), hibernating", batt);
        gpsStandby();
        SystemSleepConfiguration hibernate;
        hibernate.mode(SystemSleepMode::HIBERNATE);
        System.sleep(hibernate);
        return;  // after hibernate wake, device resets (runs setup)
    }

    // Put GPS into standby before sleeping
    gpsStandby();

    int sleepSec = getSleepInterval();
    currentMode = serverSaysStolen ? TrackerMode::STOLEN : TrackerMode::SENTRY;
    Serial.printlnf("[SENTRY] Sleeping %ds (mode=%s, bat=%.0f%%)",
        sleepSec,
        serverSaysStolen ? "STOLEN" : "normal",
        batt);

    SystemSleepConfiguration sleepConfig;
    sleepConfig.mode(SystemSleepMode::STOP)
               .duration(sleepSec * 1000L);
    // TODO: Add .gpio(D8, RISING) once VUSB voltage divider is wired

    System.sleep(sleepConfig);

    Serial.println("[SENTRY] Woke from sleep");
}

void enterSentry() {
    Serial.println("[MODE] Entering SENTRY mode");
    currentMode = TrackerMode::SENTRY;
    engineOffPending = false;

    // Publish one last position before sleeping
    if (Particle.connected() && GPS.fix) {
        publishLocation();
        delay(1100);
    }

    // Start the sentry wake/sleep cycle
    sentryWakeCycle();
}


// ── Setup ───────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("=========> JeepTracker starting");

    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    GPSSerial.println(PMTK_Q_RELEASE);

    canbus.begin();

    Particle.subscribe("hook-response/config_request", configResponseHandler);

    if (!hasPower() && !isEngineRunning()) {
        Serial.println("[BOOT] No power detected, starting sentry cycle");
        sentryWakeCycle();
    } else {
        Serial.println("[BOOT] Power detected, entering DRIVING mode");
        currentMode = TrackerMode::DRIVING;
    }

    Serial.println("Setup complete!");
}


// ── Main loop (DRIVING mode) ────────────────────────────────────────────

void loop() {
    if (currentMode != TrackerMode::DRIVING) {
        sentryWakeCycle();
        return;
    }

    // ── GPS parsing ──
    GPS.read();
    if (GPS.newNMEAreceived()) {
        if (!GPS.parse(GPS.lastNMEA()))
            return;
    }

    // ── Serial debug output ──
    static unsigned nextInterval = 2000;
    if (millis() - timer > nextInterval) {
        timer = millis();
        nextInterval = 1500 + random(1000);

        float s = GPS.seconds + GPS.milliseconds / 1000.0f + GPS.secondsSinceTime();
        int m = GPS.minute;
        int h = GPS.hour;
        int d = GPS.day;
        while (s >= 60) { s -= 60; m++; }
        while (m >= 60) { m -= 60; h++; }
        while (h >= 24) { h -= 24; d++; }

        Serial.printf("\nDate: %d-%02d-%02d   Time: %02d:%02d:%06.3f\n",
            GPS.year + 2000, GPS.month, d, h, m, s);
        Serial.printlnf("Fix: %d  quality: %d  sats: %d",
            (int)GPS.fix, (int)GPS.fixquality, (int)GPS.satellites);

        if (GPS.fix) {
            Serial.printlnf("Loc: %.6f, %.6f  Spd: %.1f mph  Hdg: %.1f  Alt: %.0f ft",
                GPS.latitudeDegrees, GPS.longitudeDegrees,
                knotsToMph(GPS.speed), GPS.angle, metersToFeet(GPS.altitude));
        }

        Serial.printlnf("Batt: %.0f%%  Power: %s  Engine: %s",
            System.batteryCharge(),
            hasPower() ? "USB/VIN" : "BATTERY",
            isEngineRunning() ? "RUNNING" : "OFF");
    }

    // ── CAN polling ──
    canbus.poll();

    // ── Location publishing ──
    if (!firstFixPublished && GPS.fix && publishLocation()) {
        firstFixPublished = true;
        lastPublish = millis();
    }

    uint32_t publishInterval = isInMotion() ? PUBLISH_INTERVAL_MS : IDLE_PUBLISH_INTERVAL_MS;
    if (millis() - lastPublish >= publishInterval && publishLocation()) {
        lastPublish = millis();
    }

    // ── CAN telemetry publishing ──
    if ((millis() - lastCanPublish >= CAN_PUBLISH_INTERVAL_MS) &&
        isEngineRunning() && publishCanTelemetry()) {
        lastCanPublish = millis();
    }

    // ── Power loss detection → sentry transition ──
    bool powered = hasPower() || isEngineRunning();

    if (powered) {
        engineOffPending = false;
    } else if (!engineOffPending) {
        engineOffPending = true;
        engineOffSince = millis();
        Serial.println("[PWR] Power lost, starting 2-min debounce");
    } else if (millis() - engineOffSince >= ENGINE_OFF_DEBOUNCE_MS) {
        Serial.println("[PWR] Debounce complete, entering sentry mode");
        enterSentry();
    }
}
