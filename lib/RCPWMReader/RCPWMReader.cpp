#include "RCPWMReader.h"

RCPWMReader::RCPWMReader(uint8_t pin) {
  _pin = pin;
  _slice = pwm_gpio_to_slice_num(pin);
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
}

float RCPWMReader::readDuty() {
  uint16_t count = pwm_get_counter(_slice);
  // Duty = pulse_width / period (assuming 20ms period = 20000us)
  return (float)count / 20000.0f;
}

float RCPWMReader::readPulseWidth() {
  // Read the counter - it accumulates microseconds while Channel B is HIGH
  uint16_t count = pwm_get_counter(_slice);

  // Reset for next reading
  pwm_set_counter(_slice, 0);

  return (float)count;
}
