#ifndef RC_PWM_READER_H
#define RC_PWM_READER_H

#include <Arduino.h>

extern "C" {
#include "hardware/clocks.h"
#include "hardware/pwm.h"
}

class RCPWMReader {
 public:
  // Constructor: assign pin
  RCPWMReader(uint8_t pin);

  // Initialize hardware
  void begin();

  // Returns duty cycle percentage (0–100%)
  float readDuty();

  // Returns pulse width in microseconds
  float readPulseWidth();

 private:
  uint8_t _pin;
  uint _slice;
  bool _isChannelB;
  pwm_config _cfg;
};

#endif
