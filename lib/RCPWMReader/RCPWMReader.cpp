#include "RCPWMReader.h"

RCPWMReader::RCPWMReader(uint8_t pin) {
  _pin = pin;
  _slice = pwm_gpio_to_slice_num(pin);
  _lastCount = 0;
  _stableCount = 0;
  _stableSamples = 0;
  _cachedPulseWidth = 1500;
  _initialized = false;
}

void RCPWMReader::begin() {
  // Assign GPIO to PWM function
  gpio_set_function(_pin, GPIO_FUNC_PWM);

  // Get default config
  pwm_config cfg = pwm_get_default_config();

  // Set clock divider: 125MHz / 125 = 1MHz (1 count = 1 microsecond)
  pwm_config_set_clkdiv(&cfg, 125.0f);

  // Use maximum wrap value
  pwm_config_set_wrap(&cfg, 0xFFFF);

  // Count only when Channel B input is HIGH
  pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_HIGH);

  // Initialize PWM slice
  pwm_init(_slice, &cfg, true);

  // Clear the counter
  pwm_set_counter(_slice, 0);
  _lastCount = 0;
  _stableCount = 0;
  _stableSamples = 0;
  _cachedPulseWidth = 1500;
  _initialized = true;
}

float RCPWMReader::readDuty() { return (float)_cachedPulseWidth / 20000.0f; }

float RCPWMReader::readPulseWidth() { return (float)_cachedPulseWidth; }
