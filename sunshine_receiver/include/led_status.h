#pragma once
#include <stdint.h>

// ── Onboard RGB status LED (ESP32-S3-DevKitC-1 WS2812) ──────────────────────────
// Drives the single addressable RGB LED to show receiver liveness + link state.
// All rendering is non-blocking (millis-based breathing). Call led_status_tick()
// frequently from the main bridge loop.
//
// Colour legend:
//   white  breathe  → booting / idle, nothing connected yet
//   red    blink    → fatal init error (ESP-NOW failed)
//   red    breathe  → alive, but no brain telemetry
//   amber  breathe  → brain link up, host app silent (control disabled / safe)
//   green  breathe  → brain + host both live; brief brighter flash per telem frame

enum LedStatus {
    LED_BOOT,        // starting up / idle
    LED_ERROR,       // fatal error (latched red blink)
    LED_NO_BRAIN,    // waiting for brain frames
    LED_BRAIN_IDLE,  // brain up, host silent
    LED_LIVE,        // brain up + host talking
};

// Initialise the LED (safe to call before WiFi/ESP-NOW init).
void led_status_init(void);

// Set the current logical status. Cheap; only records state, rendering happens
// in led_status_tick(). LED_ERROR latches until the next explicit set.
void led_status_set(LedStatus s);

// Call when a telemetry frame is forwarded to the host (drives the activity blip).
void led_status_note_frame(void);

// Render the current status (non-blocking). Call frequently (every bridge tick).
void led_status_tick(void);
