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

void RCPWMReader::update() {
  if (!_initialized) return;

  uint16_t count = pwm_hw->slice[_slice].ctr;

  // If counter hasn't changed and is in valid range
  if (count == _lastCount && count >= 800 && count <= 2200) {
    _stableSamples++;
    if (_stableSamples >= 2) {
      // Stable for 1ms - capture it
      _cachedPulseWidth = count;
      // Reset for next period
      pwm_set_counter(_slice, 0);
      _lastCount = 0;
      _stableSamples = 0;
    }
  } else if (count != _lastCount) {
    // Counter changed - reset stability counter
    _stableSamples = 0;
    _lastCount = count;
  }

  // Safety: if counter gets too big, reset it
  if (count > 3000) {
    pwm_set_counter(_slice, 0);
    _lastCount = 0;
    _stableSamples = 0;
  }
}

float RCPWMReader::readDuty() { return (float)_cachedPulseWidth / 20000.0f; }

float RCPWMReader::readPulseWidth() { return (float)_cachedPulseWidth; }
