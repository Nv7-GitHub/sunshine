#include <Arduino.h>

#include "RCPWMReader.h"

RCPWMReader chanA(0);
RCPWMReader chanB(2);
RCPWMReader chanC(4);
RCPWMReader chanD(6);

void setup() {
  Serial.begin(115200);
  Serial.println("Hello, Sunshine!");
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
