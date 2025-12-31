#include <Arduino.h>

#include "RCPWMReader.h"

// Use Channel B pins: 1, 3, 5, 7 (odd numbers)
RCPWMReader chanA(1);
RCPWMReader chanB(3);
RCPWMReader chanC(5);
RCPWMReader chanD(7);

void setup() {
  Serial.begin(115200);
  Serial.println("Hello, Sunshine!");

  chanA.begin();
  chanB.begin();
  chanC.begin();
  chanD.begin();
}

void printChan(RCPWMReader chan, const char* name, const char* start) {
  float duty = chan.readDuty();
  float pulse = chan.readPulseWidth();
  // %, microseconds
  Serial.printf("%sduty%s:%.2f,pulse%s:%.2f", start, name, duty * 100.0f, name,
                pulse);
}

void loop() {
  printChan(chanA, "A", "");
  printChan(chanB, "B", ",");
  printChan(chanC, "C", ",");
  printChan(chanD, "D", ",");
  Serial.println();
  delay(100);
}

void loop1() {
  chanA.update();
  chanB.update();
  chanC.update();
  chanD.update();
  delayMicroseconds(500);
}