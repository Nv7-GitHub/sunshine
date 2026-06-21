#include "telemetry.h"
#include "config.h"
#include "ring_buffer.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

// ── Control inputs (Core 0 writes via ESP-NOW callback, Core 1 reads) ────────
static CtrlInputs        latest_ctrl = {};
static SemaphoreHandle_t ctrl_mutex;

// ── Telemetry ring buffer (Core 1 pushes, Core 0 drains) ─────────────────────
struct TelemetryEntry {
    SunshineInput  input;
    SunshineState  state;
};
static RingBuffer<TelemetryEntry, 64> telem_ring;  // >= INPUTS_PER_FRAME + jitter margin (power of 2)

// ── ESP-NOW TX frame ──────────────────────────────────────────────────────────
// ESP-NOW v2 (IDF >= 5.4) raises the max payload from 250 to 1490 bytes, which
// lets us send the README-intended 50 Hz frame: one packet per 20 inputs.
// Frame layout: frame_id(2) + type(1) + SunshineState(60) + SunshineInput[20](20*29=580) = 643 bytes.
static constexpr int  INPUTS_PER_FRAME  = 20;
static constexpr int  FRAME_SIZE        = 2 + 1 + (int)sizeof(SunshineState) + INPUTS_PER_FRAME * (int)sizeof(SunshineInput);
static_assert(FRAME_SIZE <= ESP_NOW_MAX_DATA_LEN_V2,
              "telemetry frame exceeds ESP-NOW v2 max payload (needs IDF >= 5.4)");
static uint8_t        tx_frame[FRAME_SIZE];
static uint16_t       tx_frame_id = 0;
static int            drain_count = 0;
static SunshineState  first_state;

// ── ESP-NOW RX callback (runs on Core 0 in WiFi task context) ─────────────────
// Callback signature matches esp_now_recv_cb_t: IDF 5.x / Arduino-ESP32 3.x
static void on_espnow_recv(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len) {
    // Receiver → Brain control packet: 8 bytes
    // seq_id(2) + type(1) + mode(1) + ctrl_x(1) + ctrl_y(1) + ctrl_theta(1) + throttle(1)
    if (len != 8 || data[2] != 0x02) return;

    CtrlInputs c;
    c.mode          = data[3];
    c.ctrl_x        = (int8_t)data[4];
    c.ctrl_y        = (int8_t)data[5];
    c.ctrl_theta    = (int8_t)data[6];
    c.ctrl_throttle = data[7];
    c.rssi          = info->rx_ctrl->rssi;
    c.last_rx_ms    = (uint32_t)millis();

    if (xSemaphoreTake(ctrl_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        latest_ctrl = c;
        xSemaphoreGive(ctrl_mutex);
    }
    // else: drop this packet; next will arrive within ~2ms at 500Hz
}

CtrlInputs telemetry_get_ctrl(void) {
    CtrlInputs c;
    if (xSemaphoreTake(ctrl_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        c = latest_ctrl;
        xSemaphoreGive(ctrl_mutex);
    } else {
        // Timed out — return stale data; watchdog will catch prolonged absence
        c = latest_ctrl;  // read without lock (last-known value, safe to read)
    }
    return c;
}

bool telemetry_push(const SunshineInput *in, const SunshineState *state) {
    TelemetryEntry e;
    e.input = *in;
    e.state = *state;
    return telem_ring.push(e);
}

// ── Core 0 telemetry task ─────────────────────────────────────────────────────
static esp_now_peer_info_t receiver_peer;

void telemetry_task(void *) {
    for (;;) {
        TelemetryEntry entry;
        if (!telem_ring.pop(entry)) {
            vTaskDelay(1);   // ring empty — yield 1 tick
            continue;
        }

        if (drain_count == 0) {
            first_state = entry.state;
            // Start building TX frame header
            tx_frame[0] = (uint8_t)(tx_frame_id & 0xFF);
            tx_frame[1] = (uint8_t)(tx_frame_id >> 8);
            tx_frame[2] = 0x01;  // type = telemetry
            memcpy(tx_frame + 3, &first_state, sizeof(SunshineState));
        }

        // Copy serialised input into frame slot
        memcpy(tx_frame + 3 + sizeof(SunshineState) + drain_count * sizeof(SunshineInput),
               &entry.input, sizeof(SunshineInput));
        drain_count++;

        if (drain_count == INPUTS_PER_FRAME) {
            esp_err_t send_err = esp_now_send(receiver_peer.peer_addr, tx_frame, FRAME_SIZE);
            if (send_err != ESP_OK) {
                Serial.printf("TELEM TX drop: %s\n", esp_err_to_name(send_err));
            }
            tx_frame_id++;
            drain_count = 0;
        }
    }
}

void telemetry_init(void) {
    drain_count = 0;
    tx_frame_id = 0;
    ctrl_mutex = xSemaphoreCreateMutex();
    configASSERT(ctrl_mutex != NULL);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    // esp_wifi_set_max_tx_power takes int8_t in units of 0.25 dBm; 84 = 21 dBm
    esp_wifi_set_max_tx_power(84);

    esp_now_init();
    esp_now_register_recv_cb(on_espnow_recv);

    memset(&receiver_peer, 0, sizeof(receiver_peer));
    memcpy(receiver_peer.peer_addr, RECEIVER_MAC, 6);
    receiver_peer.channel = ESPNOW_CHANNEL;
    receiver_peer.encrypt = false;
    esp_now_add_peer(&receiver_peer);
}
