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
// AM32 2.x auto-detects BDSHOT from the inverted CRC — no configurator setting needed.
#define DSHOT_BIDIRECTIONAL  1

// ── Battery ADC ───────────────────────────────────────────────────────────────
// ADC1_CH7 = GPIO7, connected to battery voltage divider (R_high=2k, R_low=1k)
#define PIN_BATT_ADC     7
// V = adc_raw * (3.3/4095) * (R_high+R_low)/R_low = adc_raw * (3.3/4095) * 3.0
static constexpr float BATT_ADC_SCALE = (3.3f / 4095.0f) * 3.0f;

// ── LED ───────────────────────────────────────────────────────────────────────
#define PIN_LED          38
// Duty levels for analogWrite (8-bit, 0..255).
// Heading flash (TANK/MELTY): ALWAYS full brightness — only shown while spinning,
// where it must be visible in daylight, so it's never dimmed by bringup level.
#define LED_DUTY_ACTIVE  255
// DISABLED "breathe" and the sensor-init-error blink: a dim "board alive"
// indicator. Kept low at all times (the robot is stationary on a desk when
// these show), independent of bringup level. Tune to taste.
#define LED_DUTY_IDLE    40

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
static constexpr bool MOTOR_LEFT_INVERT  = true;
static constexpr bool MOTOR_RIGHT_INVERT = true;

// ── ESP-NOW ───────────────────────────────────────────────────────────────────
static constexpr uint8_t ESPNOW_CHANNEL      = 1;
// Receiver MAC — AUTO-LEARNED at runtime from the receiver's control packets
// (telemetry.cpp), so this does NOT need to be filled in. It is only the
// pre-pairing bootstrap: broadcast (FF:..) until the first control packet arrives,
// after which telemetry switches to UNICAST to the learned MAC (→ MAC ACK +
// hardware retransmit = no lost frames). Leave as broadcast unless you want to
// hard-pin a specific receiver.
static const uint8_t RECEIVER_MAC[6]         = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
