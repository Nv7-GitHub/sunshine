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
#include <atomic>
#include <string.h>

// ── Control inputs (Core 0 writes via ESP-NOW callback, Core 1 reads) ────────
static CtrlInputs        latest_ctrl = {};
static SemaphoreHandle_t ctrl_mutex;

// ── Telemetry ring buffer (Core 1 pushes, Core 0 drains) ─────────────────────
struct TelemetryEntry {
    SunshineInput  input;
    SunshineState  state;
};
// Sized for transient Core-0/WiFi preemption: at 1 kHz, 256 entries = 256 ms of
// slack. The WiFi task (Core 0, high prio) does 500 Hz control RX + 50 Hz telem TX
// and can preempt this drain task for tens of ms under RF contention; the old
// 64-entry ring (64 ms) overflowed once 50 Hz / 20-input framing made each TX a
// longer Core-0 hog (~13% of inputs dropped). 256 × sizeof(TelemetryEntry≈73 B) ≈
// 18 KB SRAM (ESP32-S3 has 512 KB). Power of 2 required.
static RingBuffer<TelemetryEntry, 256> telem_ring;
static std::atomic<uint32_t> dropped_inputs{0};

// ── Unicast pairing + delivery tracking + send-timing ────────────────────────
// Telemetry is sent UNICAST so the WiFi MAC ACKs it and retransmits in hardware
// on collision — broadcast (the old FF:FF:.. peer) has no ACK/retry, so any
// collision (incl. the half-duplex receiver being mid-control-TX) was a permanent
// lost frame → broke replay.
//
// Identity: prefer the HARD-CODED RECEIVER_MAC (config.h) — deterministic, and we
// then reject control from any other MAC (target_locked). If RECEIVER_MAC is left
// broadcast (FF:..), fall back to AUTO-LEARNING the receiver's MAC from the source
// address of the control packets it unicasts to us (convenient for bench bring-up,
// but a foreign broadcast could in principle be learned — hard-code it for comp).
static uint8_t        target_mac[6];
static volatile bool  target_known  = false;   // do we have a unicast destination yet?
static bool           target_locked = false;   // RECEIVER_MAC was hard-coded → filter control to it
static std::atomic<uint32_t> tx_fail{0};         // frames that failed even after MAC retries

static inline bool mac_is_broadcast(const uint8_t *m) {
    for (int i = 0; i < 6; i++) if (m[i] != 0xFF) return false;
    return true;
}
// Send-timing: a binary semaphore given on EVERY control RX (500 Hz). The telem
// task waits for the next control edge before sending, so its frame lands in the
// gap right after the receiver finishes a control TX (when it's listening), not
// while the half-duplex receiver is transmitting. Cuts first-try collisions →
// fewer retransmits. Fail-safe: a timeout sends anyway if the control link drops.
static SemaphoreHandle_t ctrl_edge_sem = nullptr;
// Given by the send callback when a frame's TX completes (ACK or retries-exhausted).
// The task waits on this before queuing the next frame so esp_now_send never
// overflows the ESP-NOW TX queue (ESP_ERR_ESPNOW_NO_MEM) when the ring backs up.
static SemaphoreHandle_t tx_done_sem  = nullptr;
// millis() of the last control packet from our receiver. If control is flowing the
// receiver is present (and listening); if it stops, the receiver is off → don't
// transmit telemetry (nowhere to send), just keep draining the ring.
static volatile uint32_t last_ctrl_ms = 0;

// ── ESP-NOW TX frame ──────────────────────────────────────────────────────────
// ESP-NOW v2 (IDF >= 5.4) raises the max payload from 250 to 1490 bytes, which
// lets us send the README-intended 50 Hz frame: one packet per 20 inputs.
// We carry TWO filter-state snapshots per frame so the host gets the *real*
// filter state at 100 Hz (one at the first input, one at the midpoint) — see the
// host replay/logging code. vars are NOT sent: they are a pure function of
// (state, inputs) and the host recomputes them.
// Frame layout: frame_id(2) + type(1) + SunshineState×2(88) + SunshineInput[20](580) = 671 bytes.
static constexpr int  INPUTS_PER_FRAME  = 20;
static constexpr int  FRAME_MID_INPUT   = INPUTS_PER_FRAME / 2;   // 2nd state snapshot here
static constexpr int  STATE_SIZE        = (int)sizeof(SunshineState);
static constexpr int  FRAME_SIZE        = 2 + 1 + 2 * STATE_SIZE + INPUTS_PER_FRAME * (int)sizeof(SunshineInput);
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

    // Identity: if RECEIVER_MAC is hard-coded (target_locked), accept control ONLY
    // from that MAC — in a multi-device arena this rejects every other ESP-NOW
    // device's traffic. If not locked, auto-pair off the first valid control packet.
    if (info && info->src_addr) {
        if (target_locked) {
            if (memcmp(info->src_addr, target_mac, 6) != 0) return;   // not our receiver
        } else if (!target_known) {
            memcpy(target_mac, info->src_addr, 6);
            target_known = true;
        }
    }

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

    // Receiver is present (and just finished a control TX → now listening): mark
    // the link alive and signal the telemetry task to send in this gap.
    last_ctrl_ms = (uint32_t)millis();
    if (ctrl_edge_sem) xSemaphoreGive(ctrl_edge_sem);
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
    bool ok = telem_ring.push(e);
    if (!ok) dropped_inputs.fetch_add(1, std::memory_order_relaxed);
    return ok;
}

uint32_t telemetry_dropped_count(void) {
    return dropped_inputs.load(std::memory_order_relaxed);
}

uint32_t telemetry_tx_fail_count(void) {
    return tx_fail.load(std::memory_order_relaxed);
}

// MAC TX-complete callback: SUCCESS = receiver ACKed (delivered); FAIL = no ACK
// after all hardware retries (a genuinely lost frame). Counting this tells us
// whether unicast retransmit alone achieves zero loss at the current link.
static void on_telem_sent(const wifi_tx_info_t * /*info*/, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS)
        tx_fail.fetch_add(1, std::memory_order_relaxed);
    if (tx_done_sem) xSemaphoreGive(tx_done_sem);   // unblock the next send
}

// ── Core 0 telemetry task ─────────────────────────────────────────────────────
static esp_now_peer_info_t receiver_peer;

void telemetry_task(void *) {
    bool peer_unicast = false;
    for (;;) {
        // ── Assemble exactly one 50 Hz frame (20 consecutive 1 kHz inputs) ──────
        // Popping one-at-a-time as inputs arrive; this paces the loop at ~50 Hz
        // without an extra timer. Every input is sent exactly once → replay-safe.
        for (drain_count = 0; drain_count < INPUTS_PER_FRAME; ) {
            TelemetryEntry entry;
            if (!telem_ring.pop(entry)) { vTaskDelay(1); continue; }  // ring empty — yield

            if (drain_count == 0) {
                first_state = entry.state;
                tx_frame[0] = (uint8_t)(tx_frame_id & 0xFF);
                tx_frame[1] = (uint8_t)(tx_frame_id >> 8);
                tx_frame[2] = 0x01;  // type = telemetry
                memcpy(tx_frame + 3, &first_state, STATE_SIZE);          // state_start
            }
            if (drain_count == FRAME_MID_INPUT)
                memcpy(tx_frame + 3 + STATE_SIZE, &entry.state, STATE_SIZE);  // state_mid (100 Hz)

            memcpy(tx_frame + 3 + 2 * STATE_SIZE + drain_count * sizeof(SunshineInput),
                   &entry.input, sizeof(SunshineInput));
            drain_count++;
        }

        // ── Switch broadcast → UNICAST once the receiver MAC is known ───────────
        // Unicast gets MAC-layer ACK + hardware retransmit (no app reorder; each
        // frame delivered once or retried). Broadcast (bring-up fallback) does not.
        // Add the unicast peer ALONGSIDE the broadcast one (harmless to keep) so
        // there's never a window with no usable peer.
        if (target_known && !peer_unicast) {
            esp_now_peer_info_t up = {};
            memcpy(up.peer_addr, target_mac, 6);
            up.channel = ESPNOW_CHANNEL;
            up.encrypt = false;
            if (esp_now_add_peer(&up) == ESP_OK || esp_now_is_peer_exist(target_mac))
                peer_unicast = true;
        }
        const uint8_t *dest = peer_unicast ? target_mac : receiver_peer.peer_addr;

        // ── Only transmit while the receiver is present ─────────────────────────
        // We know it is when its control stream is arriving (last_ctrl_ms fresh).
        // With the receiver OFF there is nowhere to send: we already drained this
        // frame's inputs above (so the ring can't overflow), so just DROP it rather
        // than hammer esp_now_send at an unACKing peer — that backs up the ESP-NOW
        // TX queue (→ ESP_ERR_ESPNOW_NO_MEM) and would stall the drain.
        bool link_alive = (last_ctrl_ms != 0) &&
                          ((uint32_t)((uint32_t)millis() - last_ctrl_ms) < 200u);
        if (!link_alive) { tx_frame_id++; continue; }

        // ── Flow control: wait for the PREVIOUS send to finish ──────────────────
        // One frame in flight at a time → esp_now_send can't overflow the ESP-NOW
        // TX queue. The send callback gives this; timeout is a fail-safe.
        if (tx_done_sem) xSemaphoreTake(tx_done_sem, pdMS_TO_TICKS(50));

        // ── Gap-align the send to just after a control RX ───────────────────────
        // Drain any stale edge, then wait for the NEXT control edge (≤ ~2 ms at
        // 500 Hz) so our frame arrives while the half-duplex receiver is listening.
        // Short timeout so a momentary control gap never throttles the drain.
        if (ctrl_edge_sem) {
            xSemaphoreTake(ctrl_edge_sem, 0);                   // clear stale give
            xSemaphoreTake(ctrl_edge_sem, pdMS_TO_TICKS(5));    // next edge or timeout
        }

        esp_err_t send_err = esp_now_send(dest, tx_frame, FRAME_SIZE);
        if (send_err != ESP_OK) {
            static uint32_t last_err_ms = 0;
            uint32_t now_ms = (uint32_t)millis();
            if (now_ms - last_err_ms > 1000) {                 // rate-limit the print
                Serial.printf("TELEM TX err: %s\n", esp_err_to_name(send_err));
                last_err_ms = now_ms;
            }
            if (tx_done_sem) xSemaphoreGive(tx_done_sem);       // no send-cb → unblock now
        }
        tx_frame_id++;
    }
}

void telemetry_init(void) {
    drain_count = 0;
    tx_frame_id = 0;
    ctrl_mutex = xSemaphoreCreateMutex();
    configASSERT(ctrl_mutex != NULL);
    ctrl_edge_sem = xSemaphoreCreateBinary();   // control-RX edge for gap-aligned TX
    configASSERT(ctrl_edge_sem != NULL);
    tx_done_sem = xSemaphoreCreateBinary();     // TX-complete flow control
    configASSERT(tx_done_sem != NULL);
    xSemaphoreGive(tx_done_sem);                // prime: first send proceeds immediately

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    // esp_wifi_set_max_tx_power takes int8_t in units of 0.25 dBm; 84 = 21 dBm
    esp_wifi_set_max_tx_power(84);

    esp_now_init();
    esp_now_register_recv_cb(on_espnow_recv);
    esp_now_register_send_cb(on_telem_sent);   // track unicast ACK / retry-exhausted

    memset(&receiver_peer, 0, sizeof(receiver_peer));
    memcpy(receiver_peer.peer_addr, RECEIVER_MAC, 6);
    receiver_peer.channel = ESPNOW_CHANNEL;
    receiver_peer.encrypt = false;
    esp_now_add_peer(&receiver_peer);

    // If RECEIVER_MAC is hard-coded, lock onto it: telemetry unicasts straight to
    // it and control from any other MAC is rejected (multi-device arena). If left
    // broadcast, we auto-pair from the first control packet (bench bring-up).
    if (!mac_is_broadcast(RECEIVER_MAC)) {
        memcpy(target_mac, RECEIVER_MAC, 6);
        target_known  = true;
        target_locked = true;
        Serial.println("TELEM: unicast to hard-coded RECEIVER_MAC (control filtered to it)");
    } else {
        Serial.println("TELEM: RECEIVER_MAC unset — auto-pairing + broadcast until paired "
                       "(hard-code RECEIVER_MAC for comp)");
    }
}
