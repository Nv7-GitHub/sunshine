#pragma once
#include <stdint.h>

// ── Brain MAC address ────────────────────────────────────────────────────────
// Fill in the actual MAC from the brain's WiFi STA interface.
// Run `WiFi.macAddress()` on brain to read it.
static const uint8_t BRAIN_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // TODO: fill in after bringup

// ── Timing constants ─────────────────────────────────────────────────────────
static constexpr uint32_t CTRL_TX_INTERVAL_US    = 2000;    // 500 Hz
static constexpr uint32_t HEARTBEAT_INTERVAL_MS  = 100;     // 10 Hz
static constexpr uint32_t RSSI_INTERVAL_MS       = 100;     // 10 Hz
static constexpr uint32_t HOST_WATCHDOG_US       = 3000000; // 3 s → force DISABLED
static constexpr uint32_t BRAIN_TIMEOUT_MS       = 200;     // 10 missed @50 Hz → disconnected

// ── ESP-NOW channel ──────────────────────────────────────────────────────────
static constexpr uint8_t  ESPNOW_CHANNEL = 1;
