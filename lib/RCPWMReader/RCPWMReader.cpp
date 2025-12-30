#include "RCPWMReader.h"

RCPWMReader::RCPWMReader(uint8_t pin) {
  _pin = pin;
  _slice = pwm_gpio_to_slice_num(pin);
  _isChannelB = (pwm_gpio_to_channel(pin) == PWM_CHAN_B);
}

void RCPWMReader::begin() {
  // Assign GPIO to PWM function
  gpio_set_function(_pin, GPIO_FUNC_PWM);

  // Get default config
  _cfg = pwm_get_default_config();

  // Use free-running wrap (max 16-bit)
  pwm_config_set_wrap(&_cfg, 0xFFFF);

  // Count only when input is HIGH for duty cycle measurement
  pwm_config_set_clkdiv_mode(&_cfg, PWM_DIV_B_HIGH);

  // Initialize PWM slice
  pwm_init(_slice, &_cfg, true);
}

float RCPWMReader::readDuty() {
  uint16_t count = pwm_get_counter(_slice);

  // Duty = counts during HIGH / max wrap
  float duty = (float)count / (float)(_cfg.top + 1);
  return duty;
}

float RCPWMReader::readPulseWidth() {
  float duty = readDuty();

  // Convert duty % to pulse width in microseconds
  float period_us = ((float)(_cfg.top + 1) / clock_get_hz(clk_sys)) * 1e6;
  float pulse_us = duty / 100.0f * period_us;

  return pulse_us;
}
