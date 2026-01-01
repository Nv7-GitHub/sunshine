#include <Arduino.h>

#include "RCPWMReader.h"
#include "dshot/esc.h"

// Use Channel B pins: 1, 3, 5, 7 (odd numbers)
RCPWMReader chanA(1);
RCPWMReader chanB(3);
RCPWMReader chanC(5);
RCPWMReader chanD(7);

DShot::ESC dshot1(6, pio0, DShot::Type::Normal, DShot::Speed::DS600, 10);
DShot::ESC dshot2(8, pio0, DShot::Type::Normal, DShot::Speed::DS600, 10);

unsigned long lastDisarmTime = 0;
bool wasStopped = false;

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

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  // Init DShot - send zero throttle for 3 seconds for ESC initialization
  if (millis() < 3000) {
    dshot1.setCommand(0);
    dshot2.setCommand(0);
  } else {
    float throttle = mapf(chanA.readPulseWidth(), 1070.0f, 2125.0f, 0.0f, 1.0f);
    throttle = constrain(throttle, 0.0f, 1.0f);

    if (throttle < 0.05f) {
      // Deadband - send a very small throttle value that keeps ESC armed but
      // motor stopped
      dshot1.setThrottle(0.001f);  // Tiny value above 0
      dshot2.setThrottle(0.001f);
      Serial.print("STOPPED;");
    } else {
      // Map throttle from [0.05, 1.0] input to [0.05, 1.0] output
      dshot1.setThrottle(throttle);
      dshot2.setThrottle(throttle);
    }
  }

  printChan(chanA, "A", "");
  printChan(chanB, "B", ",");
  printChan(chanC, "C", ",");
  printChan(chanD, "D", ",");
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