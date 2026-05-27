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
#define PIN_BATT_ADC    39
// V = adc_raw * (3.3/4095) * (R_high+R_low)/R_low = adc_raw * (3.3/4095) * 3.0
static constexpr float BATT_ADC_SCALE = (3.3f / 4095.0f) * 3.0f;

// ── LED ───────────────────────────────────────────────────────────────────────
#define PIN_LED          2

// ── Timing ────────────────────────────────────────────────────────────────────
static constexpr uint32_t LOOP_INTERVAL_US   = 1000;  // 1 kHz
static constexpr uint32_t TELEMETRY_FRAMES   = 20;    // inputs per 50 Hz frame
static constexpr uint32_t CTRL_WATCHDOG_MS   = 500;   // no ctrl → DISABLED

// ── ESP-NOW ───────────────────────────────────────────────────────────────────
static constexpr uint8_t ESPNOW_CHANNEL      = 1;
// Receiver MAC — fill in after running receiver and noting its WiFi STA MAC
static const uint8_t RECEIVER_MAC[6]         = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
