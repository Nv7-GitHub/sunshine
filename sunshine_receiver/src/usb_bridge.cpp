// src/usb_bridge.cpp
// Core 1: USB TX/RX bridge + 500 Hz ESP-NOW control TX + host watchdog

#include "config.h"
#include "protocol.h"
#include <esp_now.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>
#include <Arduino.h>

// ── Latest control state (shared with 500 Hz timer) ──────────────────────────
static CtrlPayload     latest_ctrl    = {0, 0, 0, 0, 0}; // zeroed = DISABLED
static SemaphoreHandle_t ctrl_mutex;
static volatile int64_t last_host_rx_us = 0;  // µs timestamp of last host packet

// ── ESP-NOW peer info (brain) ─────────────────────────────────────────────────
static esp_now_peer_info_t brain_peer;

// ── Build the 8-byte brain-bound ESP-NOW packet ───────────────────────────────
// Format: seq_id(2) + type(1) + mode(1) + x(1) + y(1) + theta(1) + throttle(1)
static uint16_t tx_seq = 0;

static void build_ctrl_espnow(uint8_t *out8, const CtrlPayload *ctrl) {
    out8[0] = (uint8_t)(tx_seq & 0xFF);
    out8[1] = (uint8_t)(tx_seq >> 8);
    out8[2] = 0x02;                   // type
    out8[3] = ctrl->mode;
    out8[4] = (uint8_t)ctrl->ctrl_x;
    out8[5] = (uint8_t)ctrl->ctrl_y;
    out8[6] = (uint8_t)ctrl->ctrl_theta;
    out8[7] = ctrl->ctrl_throttle;
    tx_seq++;
}

// ── 500 Hz timer callback (runs on timer task, not Core 1) ───────────────────
static void timer_500hz_cb(void *arg) {
    CtrlPayload ctrl;
    bool taken = (xSemaphoreTake(ctrl_mutex, 0) == pdTRUE);
    ctrl = latest_ctrl; // stale read is acceptable if mutex was contended

    // Host watchdog: if host has been silent > 3 s, force DISABLED
    int64_t now_us = esp_timer_get_time();
    if (now_us - last_host_rx_us > (int64_t)HOST_WATCHDOG_US) {
        ctrl.mode          = 0; // DISABLED
        ctrl.ctrl_x        = 0;
        ctrl.ctrl_y        = 0;
        ctrl.ctrl_theta    = 0;
        ctrl.ctrl_throttle = 0;
        // Only update stored state if we hold the lock
        if (taken) latest_ctrl = ctrl;
    }
    if (taken) xSemaphoreGive(ctrl_mutex);

    uint8_t pkt[8];
    build_ctrl_espnow(pkt, &ctrl);
    esp_now_send(brain_peer.peer_addr, pkt, sizeof(pkt));
}

// ── USB frame TX helpers ──────────────────────────────────────────────────────
static uint8_t tx_scratch[ESPNOW_TELEM_SIZE + 8];

static void usb_send_telem(const uint8_t *telem_payload) {
    uint16_t n = frame_encode(tx_scratch, TYPE_TELEM_FRAME,
                              telem_payload, ESPNOW_TELEM_SIZE);
    Serial.write(tx_scratch, n);
}

static void usb_send_heartbeat(void) {
    uint32_t ms = (uint32_t)millis();
    uint8_t  payload[4];
    memcpy(payload, &ms, 4);
    uint16_t n = frame_encode(tx_scratch, TYPE_HEARTBEAT, payload, 4);
    Serial.write(tx_scratch, n);
}

static void usb_send_rssi(int8_t rssi) {
    uint8_t  payload[1] = {(uint8_t)rssi};
    uint16_t n = frame_encode(tx_scratch, TYPE_RX_RSSI, payload, 1);
    Serial.write(tx_scratch, n);
}

static void usb_send_status(uint8_t code, const char *msg) {
    uint8_t  payload[33];
    payload[0] = code;
    uint8_t  msglen = (uint8_t)strnlen(msg, 32);
    memcpy(payload + 1, msg, msglen);
    uint16_t n = frame_encode(tx_scratch, TYPE_STATUS, payload, 1 + msglen);
    Serial.write(tx_scratch, n);
}

// ── USB RX parser ─────────────────────────────────────────────────────────────
static FrameParser usb_parser;

static void process_host_frame(uint8_t type, const uint8_t *payload, uint16_t len) {
    last_host_rx_us = esp_timer_get_time();  // reset watchdog

    if (type == TYPE_CTRL_PACKET && len == CTRL_PAYLOAD_SIZE) {
        CtrlPayload ctrl;
        memcpy(&ctrl, payload, CTRL_PAYLOAD_SIZE);
        xSemaphoreTake(ctrl_mutex, portMAX_DELAY);
        latest_ctrl = ctrl;
        xSemaphoreGive(ctrl_mutex);
    }
    // HEARTBEAT from host also resets watchdog (already done above — any packet counts)
}

// ── Public API ────────────────────────────────────────────────────────────────

void usb_bridge_init(void) {
    ctrl_mutex = xSemaphoreCreateMutex();
    configASSERT(ctrl_mutex != NULL);
    last_host_rx_us = esp_timer_get_time(); // start watchdog from now

    // Configure brain peer
    memset(&brain_peer, 0, sizeof(brain_peer));
    memcpy(brain_peer.peer_addr, BRAIN_MAC, 6);
    brain_peer.channel = ESPNOW_CHANNEL;
    brain_peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&brain_peer));

    // Start 500 Hz timer
    esp_timer_create_args_t args = {};
    args.callback  = timer_500hz_cb;
    args.name      = "ctrl_tx";
    static esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, CTRL_TX_INTERVAL_US));
}

// Called from Core 1 bridge loop
void usb_bridge_tick(void) {
    static uint32_t last_heartbeat_ms = 0;
    static uint32_t last_rssi_ms      = 0;
    static bool     prev_connected    = false;

    // RX: process all available bytes from host
    while (Serial.available()) {
        uint8_t b = Serial.read();
        uint8_t  out_type;
        const uint8_t *out_payload;
        uint16_t       out_len;
        if (frame_parser_feed(&usb_parser, b, &out_type, &out_payload, &out_len))
            process_host_frame(out_type, out_payload, out_len);
    }

    // TX: forward new telemetry frames to host
    static uint8_t telem_buf[ESPNOW_TELEM_SIZE];
    if (espnow_rx_get_frame(telem_buf, 0)) {  // non-blocking
        usb_send_telem(telem_buf);
    }

    // TX: heartbeat at 10 Hz
    uint32_t now = (uint32_t)millis();
    if (now - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        last_heartbeat_ms = now;
        usb_send_heartbeat();
    }

    // TX: receiver-side RSSI at 10 Hz
    if (now - last_rssi_ms >= RSSI_INTERVAL_MS) {
        last_rssi_ms = now;
        usb_send_rssi(espnow_rx_get_rssi());
    }

    // TX: brain connection status events
    bool connected = espnow_rx_brain_connected();
    if (connected != prev_connected) {
        prev_connected = connected;
        if (connected)
            usb_send_status(STATUS_BRAIN_CONNECTED,    "Brain connected");
        else
            usb_send_status(STATUS_BRAIN_DISCONNECTED, "Brain disconnected");
    }
}
