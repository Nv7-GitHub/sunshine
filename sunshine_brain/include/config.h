#pragma once
#include <stdint.h>

// ── SPI pins ──────────────────────────────────────────────────────────────────
// FSPI → ADXL375
#define PIN_ADXL_CS    10
#define PIN_ADXL_MOSI  11
#define PIN_ADXL_SCK   12
#define PIN_ADXL_MISO  13
// INT pins (not used in polled mode, reserved)
#define PIN_ADXL_INT1   8
#define PIN_ADXL_INT2   9

// HSPI → LIS3MDL
#define PIN_MAG_CS      18
#define PIN_MAG_MOSI    15
#define PIN_MAG_SCK     16
#define PIN_MAG_MISO    17
#define PIN_MAG_DRDY    14
#define PIN_MAG_INT     21

// ── DShot ─────────────────────────────────────────────────────────────────────
#define PIN_DSHOT_LEFT   4   // S1
#define PIN_DSHOT_RIGHT  5   // S2

// ── Battery ADC ───────────────────────────────────────────────────────────────
// PCB schematic routes the battery voltage divider (VSENSE net) to IO39 (ESP32
// module pad 32). However, GPIO 39 is digital-only on the ESP32-S3 — ADC1
// covers GPIO 1–10 and ADC2 covers GPIO 11–20 only. analogRead(39) returns 0.
// TODO: hardware fix required — reroute VSENSE to an ADC-capable pin.
// Using GPIO 7 (ADC1_CH6) as a stand-in until the PCB is revised.
#define PIN_BATT_ADC     7   // TODO: confirm against PCB schematic (IO39 on current PCB is digital-only)
// V = adc_raw * (3.3/4095) * (R_high+R_low)/R_low = adc_raw * (3.3/4095) * 3.0
static constexpr float BATT_ADC_SCALE = (3.3f / 4095.0f) * 3.0f;

// ── LED ───────────────────────────────────────────────────────────────────────
#define PIN_LED          38

// ── Timing ────────────────────────────────────────────────────────────────────
static constexpr uint32_t LOOP_INTERVAL_US   = 1000;  // 1 kHz
static constexpr uint32_t CTRL_WATCHDOG_MS   = 500;   // no ctrl → DISABLED

// ── Motor direction ───────────────────────────────────────────────────────────
// Set to true to invert a motor's DShot output (mirrors forward↔reverse around
// DSHOT_NEUTRAL). Use this when wiring or AM32 direction can't be changed.
// In MELTY mode with no translation input, both motors spinning "forward"
// should produce CCW body rotation (viewed from above). If the robot spins CW,
// flip both. If only one motor spins the wrong way, flip that one.
// See BRINGUP.md Level 2 for the verification procedure.
static constexpr bool MOTOR_LEFT_INVERT  = false;
static constexpr bool MOTOR_RIGHT_INVERT = false;

// ── ESP-NOW ───────────────────────────────────────────────────────────────────
static constexpr uint8_t ESPNOW_CHANNEL      = 1;
// Receiver MAC — fill in after running receiver and noting its WiFi STA MAC
static const uint8_t RECEIVER_MAC[6]         = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
