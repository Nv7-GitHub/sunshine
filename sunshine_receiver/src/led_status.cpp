// src/led_status.cpp
// Non-blocking onboard RGB status LED driver (ESP32-S3-DevKitC-1 WS2812).
// See led_status.h for the colour legend.

#include "led_status.h"
#include <Arduino.h>

// The ESP32-S3-DevKitC-1 onboard WS2812 is on GPIO48 on most revisions
// (a few early boards use GPIO38). The Arduino board definition exposes it as
// RGB_BUILTIN; override with -DSTATUS_LED_PIN=<gpio> in platformio.ini if your
// board differs.
#ifndef STATUS_LED_PIN
#  ifdef RGB_BUILTIN
#    define STATUS_LED_PIN RGB_BUILTIN
#  else
#    define STATUS_LED_PIN 48
#  endif
#endif

// Global brightness cap (0-255). The WS2812 is very bright at full scale; keep
// this modest so it's readable on a desk without being blinding.
static constexpr uint8_t LED_MAX = 48;

static volatile LedStatus s_status   = LED_BOOT;
static uint32_t           s_last_frame_ms = 0;
static uint32_t           s_last_render_ms = 0;

void led_status_init(void) {
    s_status = LED_BOOT;
    // First write clears the LED to a known (dim) state.
    neopixelWrite(STATUS_LED_PIN, 1, 1, 1);
}

void led_status_set(LedStatus s) {
    // Don't let a non-error status clobber a latched error.
    if (s_status == LED_ERROR && s != LED_ERROR) return;
    s_status = s;
}

void led_status_note_frame(void) {
    s_last_frame_ms = (uint32_t)millis();
}

// Triangle-wave breathing between lo and hi over `period_ms`.
static uint8_t breathe(uint32_t period_ms, uint8_t lo, uint8_t hi) {
    uint32_t t  = (uint32_t)millis() % period_ms;
    float    ph = (float)t / (float)period_ms;           // 0..1
    float    tri = ph < 0.5f ? ph * 2.0f : (1.0f - ph) * 2.0f; // 0..1..0
    return (uint8_t)(lo + (float)(hi - lo) * tri);
}

void led_status_tick(void) {
    uint32_t now = (uint32_t)millis();
    // Throttle actual WS2812 writes to ~60 Hz (the RMT transaction isn't free).
    if (now - s_last_render_ms < 16) return;
    s_last_render_ms = now;

    uint8_t r = 0, g = 0, b = 0;

    switch (s_status) {
        case LED_ERROR: {
            // Hard red blink, 150 ms on / 150 ms off.
            bool on = ((now / 150) & 1) == 0;
            r = on ? LED_MAX : 0;
            break;
        }
        case LED_BOOT: {
            // Dim white "alive" breathe.
            uint8_t v = breathe(2000, 2, LED_MAX / 2);
            r = g = b = v;
            break;
        }
        case LED_NO_BRAIN: {
            // Red breathe — receiver alive, waiting for the brain.
            r = breathe(1500, 3, LED_MAX);
            break;
        }
        case LED_BRAIN_IDLE: {
            // Amber breathe — brain link up, host silent (control disabled).
            uint8_t v = breathe(2000, 3, LED_MAX);
            r = v;
            g = (uint8_t)(v * 0.45f);  // amber = red + a touch of green
            break;
        }
        case LED_LIVE: {
            // Green breathe; flash brighter (whiter) briefly per telem frame.
            if (now - s_last_frame_ms < 70) {
                g = LED_MAX;
                b = LED_MAX / 3;       // cyan-ish pop on data activity
            } else {
                g = breathe(2500, 6, LED_MAX);
            }
            break;
        }
    }

    neopixelWrite(STATUS_LED_PIN, r, g, b);
}
