// src/espnow_rx.cpp
// Core 0: ESP-NOW receive callback — stores latest brain telemetry frame
// into a double buffer; signals Core 1 when a new frame is ready.
//
// Note: compiled against the pioarduino IDF-5.x platform (ESP-NOW v2). The IDF-5
// recv callback takes esp_now_recv_info_t and exposes RSSI directly via
// info->rx_ctrl->rssi, so no promiscuous sniffer is needed.

#include "config.h"
#include "protocol.h"
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>
#include <Arduino.h>

// ── Double buffer ─────────────────────────────────────────────────────────────
// Two fixed slots; writer atomically flips write_idx; reader swaps read_idx.
static uint8_t       telem_buf[2][ESPNOW_TELEM_SIZE];
static volatile int  write_idx = 0;
static int8_t        brain_rssi = -127;

// ── Synchronisation ──────────────────────────────────────────────────────────
static SemaphoreHandle_t telem_sem;     // signalled each new frame
static SemaphoreHandle_t rssi_mutex;    // protects brain_rssi

// ── Brain connection state ───────────────────────────────────────────────────
static volatile uint32_t last_brain_frame_ms = 0;
static volatile bool     brain_connected     = false;

// ── Called from Core 0 ESP-NOW task (IDF-5 recv callback signature) ──────────
static void on_espnow_recv(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len) {
    if (len != (int)ESPNOW_TELEM_SIZE) return;  // wrong size — ignore

    // Validate first 3 bytes: frame_id(2) + type=0x01
    if (data[2] != 0x01) return;

    // Write to inactive slot, then flip
    int next = 1 - write_idx;
    memcpy(telem_buf[next], data, ESPNOW_TELEM_SIZE);
    write_idx = next;

    // Receiver-side RSSI is available directly in IDF 5.x
    if (info && info->rx_ctrl) espnow_rx_update_rssi(info->rx_ctrl->rssi);

    // Track brain connection
    last_brain_frame_ms = (uint32_t)millis();
    brain_connected     = true;

    // Signal bridge task
    xSemaphoreGive(telem_sem);
}

// ── Public API ────────────────────────────────────────────────────────────────

void espnow_rx_init(void) {
    telem_sem  = xSemaphoreCreateBinary();
    rssi_mutex = xSemaphoreCreateMutex();
    configASSERT(telem_sem  != NULL);
    configASSERT(rssi_mutex != NULL);
}

// Returns true when a new frame is available; copies it into out_buf (643 B).
// Blocks up to timeout_ms milliseconds.
bool espnow_rx_get_frame(uint8_t *out_buf, uint32_t timeout_ms) {
    if (xSemaphoreTake(telem_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        memcpy(out_buf, telem_buf[write_idx], ESPNOW_TELEM_SIZE);
        return true;
    }
    return false;
}

// Update RSSI from external sniffer task (IDF 4.x has no RSSI in recv cb).
void espnow_rx_update_rssi(int8_t rssi) {
    xSemaphoreTake(rssi_mutex, portMAX_DELAY);
    brain_rssi = rssi;
    xSemaphoreGive(rssi_mutex);
}

int8_t espnow_rx_get_rssi(void) {
    xSemaphoreTake(rssi_mutex, portMAX_DELAY);
    int8_t r = brain_rssi;
    xSemaphoreGive(rssi_mutex);
    return r;
}

bool espnow_rx_brain_connected(void) {
    if (brain_connected &&
        (uint32_t)millis() - last_brain_frame_ms > BRAIN_TIMEOUT_MS) {
        brain_connected = false;
    }
    return brain_connected;
}

// Called from main.cpp after WiFi init
void espnow_rx_register_callback(void) {
    esp_now_register_recv_cb(on_espnow_recv);
}
