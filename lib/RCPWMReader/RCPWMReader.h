#ifndef RC_PWM_READER_H
#define RC_PWM_READER_H

#include <Arduino.h>

extern "C" {
#include "hardware/clocks.h"
#include "hardware/pwm.h"
}

class RCPWMReader {
 public:
  // Constructor: assign pin (must be PWM Channel B:
  // 1,3,5,7,9,11,13,15,17,19,21,23,25,27,29)
  RCPWMReader(uint8_t pin);

  // Initialize hardware
  void begin();

  // MUST call this at ~2kHz for accurate readings
  void update();

  // Returns duty cycle percentage (0–1.0)
  float readDuty();

  // Returns pulse width in microseconds
  float readPulseWidth();

 private:
  uint8_t _pin;
  uint _slice;
  volatile uint16_t _lastCount;
  volatile uint16_t _stableCount;
  volatile uint8_t _stableSamples;
  volatile uint16_t _cachedPulseWidth;
  volatile bool _initialized;
};

#endif
