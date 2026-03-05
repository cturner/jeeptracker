/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_GPS.h"

#define GPSSerial Serial1

Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = 0;
uint32_t lastPublish = 0;

const uint32_t PUBLISH_INTERVAL_MS = 30000;
const float MOTION_THRESHOLD_MPH = 2.0f;

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("=========> running setup()");
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
  Serial.println("setup complete!");
}

float metersToFeet(float meters) {
    return meters * 3.28084f;
}

float knotsToMph(float knots) {
    return knots * 1.150779f;
}

bool isInMotion() {
    return GPS.fix && knotsToMph(GPS.speed) >= MOTION_THRESHOLD_MPH;
}

void publishLocation() {
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,\"alt\":%.1f,\"hdg\":%.1f,\"sat\":%d}",
        (float)GPS.latitudeDegrees,
        (float)GPS.longitudeDegrees,
        knotsToMph(GPS.speed),
        metersToFeet(GPS.altitude),
        (float)GPS.angle,
        (int)GPS.satellites);
    if (Particle.connected()) {
        Particle.publish("location", buf, PRIVATE);
        Serial.printlnf("Published: %s", buf);
    } else {
        Serial.printlnf("Cloud offline, skipped publish: %s", buf);
    }
}


// loop() runs over and over again, as quickly as it can execute.
void loop() {
  GPS.read();
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    // Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived()
    // flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag
                                    // to false
      return; // we can fail to parse a sentence in which case we should just
              // wait for another
  }

  static unsigned nextInterval = 2000;
  if (millis() - timer > nextInterval) {
    timer = millis(); // reset the timer
    nextInterval = 1500 + random(1000);
    // Time in seconds keeps increasing after we get the NMEA sentence.
    // This estimate will lag real time due to transmission and parsing delays,
    // but the lag should be small and should also be consistent.
    float s = GPS.seconds + GPS.milliseconds / 1000.0f + GPS.secondsSinceTime();
    int m = GPS.minute;
    int h = GPS.hour;
    int d = GPS.day;
    // Adjust time and day forward to account for elapsed time.
    // This will break at month boundaries!!! Humans will have to cope with
    // April 31,32 etc.
    while (s >= 60) {
      s -= 60;
      m++;
    }
    while (m >= 60) {
      m -= 60;
      h++;
    }
    while (h >= 24) {
      h -= 24;
      d++;
    }
    // ISO Standard Date Format, with leading zeros https://xkcd.com/1179/
    Serial.print("\nDate: ");
    Serial.print(GPS.year + 2000, DEC);
    Serial.print("-");
    if (GPS.month < 10)
      Serial.print("0");
    Serial.print(GPS.month, DEC);
    Serial.print("-");
    if (d < 10)
      Serial.print("0");
    Serial.print(d, DEC);
    Serial.print("   Time: ");
    if (h < 10)
      Serial.print("0");
    Serial.print(h, DEC);
    Serial.print(':');
    if (m < 10)
      Serial.print("0");
    Serial.print(m, DEC);
    Serial.print(':');
    if (s < 10)
      Serial.print("0");
    Serial.println(s, 3);
    Serial.print("Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print(" quality: ");
    Serial.println((int)GPS.fixquality);
    Serial.print("Time [s] since last fix: ");
    Serial.println(GPS.secondsSinceFix(), 3);
    Serial.print("    since last GPS time: ");
    Serial.println(GPS.secondsSinceTime(), 3);
    Serial.print("    since last GPS date: ");
    Serial.println(GPS.secondsSinceDate(), 3);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4);
      Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4);
      Serial.println(GPS.lon);
      Serial.print("Speed (mph): ");
      Serial.println(knotsToMph(GPS.speed));
      Serial.print("Angle: ");
      Serial.println(GPS.angle);
      Serial.print("Altitude: ");
      Serial.println(metersToFeet(GPS.altitude));
      Serial.print("Satellites: ");
      Serial.println((int)GPS.satellites);
    }

  }

  if ((millis() - lastPublish >= PUBLISH_INTERVAL_MS) && isInMotion()) {
    lastPublish = millis();
    publishLocation();
  }
}


