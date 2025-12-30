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

void printChan(RCPWMReader chan) {
  float duty = chan.readDuty();
  float pulse = chan.readPulseWidth();
  // %, microseconds
  Serial.printf("Duty:%.2f,Pulse:%.2f\n", duty * 100.0f, pulse);
}

void loop() {
  printChan(chanA);
  printChan(chanB);
  printChan(chanC);
  printChan(chanD);
  Serial.println();
  delay(100);
}
