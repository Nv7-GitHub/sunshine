#include <Arduino.h>

#include "RCPWMReader.h"
#include "dshot/esc.h"

// Use Channel B pins: 1, 3, 5, 7 (odd numbers)
RCPWMReader chanA(1);
RCPWMReader chanB(3);
RCPWMReader chanC(5);
RCPWMReader chanD(7);

DShot::ESC dshot1(6, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 10);
DShot::ESC dshot2(8, pio0, DShot::Type::Bidir, DShot::Speed::DS600, 10);

void setup() {
  Serial.begin(115200);
  Serial.println("Hello, Sunshine!");

  chanA.begin();
  chanB.begin();
  chanC.begin();
  chanD.begin();
  dshot1.init();
  dshot2.init();
}

void printChan(RCPWMReader chan, const char* name, const char* start) {
  float duty = chan.readDuty();
  float pulse = chan.readPulseWidth();
  // %, microseconds
  Serial.printf("%sduty%s:%.2f,pulse%s:%.2f", start, name, duty * 100.0f, name,
                pulse);
}

void getTelemetry(DShot::ESC& esc, const char* name) {
  uint64_t raw_telemetry;
  if (esc.getRawTelemetry(raw_telemetry)) {
    DShot::Telemetry telemetry;
    if (esc.decodeTelemetry(raw_telemetry, telemetry)) {
      Serial.printf(
          ",%s_temp:%.1f,%s_voltage:%.2f,%s_current:%.2f,%s_erpm:%.2f", name,
          telemetry.temperature_C, name, telemetry.volts_cV, name,
          telemetry.amps_A, name, telemetry.eRPM_period_us);
    }
  }
}

void loop() {
  // Init DShot
  if (millis() < 3000) {
    dshot1.setCommand(0);  // 1046 is the example command
    dshot2.setCommand(0);
  } else if (millis() < 4000) {
    dshot1.setCommand(13);  // extended telemetry enable
    dshot2.setCommand(13);  // extended telemetry enable
  } else {
    dshot1.setThrottle(
        0.25);  // https://github.com/betaflight/betaflight/issues/2879
    dshot2.setThrottle(
        0.25);  // https://github.com/betaflight/betaflight/issues/2879
  }

  printChan(chanA, "A", "");
  printChan(chanB, "B", ",");
  printChan(chanC, "C", ",");
  printChan(chanD, "D", ",");

  // Check for telemetry
  getTelemetry(dshot1, "esc1");
  getTelemetry(dshot2, "esc2");
  Serial.println();
  delay(50);
}

void loop1() {
  chanA.update();
  chanB.update();
  chanC.update();
  chanD.update();
  delayMicroseconds(500);
}