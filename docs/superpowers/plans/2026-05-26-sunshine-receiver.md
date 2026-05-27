# sunshine_receiver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the ESP32-S3 receiver firmware that bridges ESP-NOW (from brain) to USB serial (to host app) and sends 500 Hz control packets back to the brain.

**Architecture:** Two FreeRTOS tasks. Core 0 handles the ESP-NOW RX callback (stores latest telemetry frame in a double buffer). Core 1 runs a continuous loop: reads USB from host, forwards telemetry frames to host at 50 Hz, and drives a 500 Hz `esp_timer` that sends control packets to the brain. A 3-second host-silence watchdog forces `mode=DISABLED` on the 500 Hz output.

**Tech Stack:** PlatformIO, ESP32-S3, Arduino framework (esp-idf 6.0+), ESP-NOW v2.0, FreeRTOS, `esp_timer`.

---

## File Structure

```
sunshine_receiver/
├── platformio.ini
├── include/
│   ├── config.h          # MAC addresses, timing constants
│   └── protocol.h        # USB frame format, type codes, encode/decode
├── src/
│   ├── main.cpp          # setup() + task creation + ESP-NOW init
│   ├── espnow_rx.cpp     # ESP-NOW RX callback + double buffer
│   └── usb_bridge.cpp    # 500 Hz timer, USB TX/RX, watchdog, heartbeat
└── lib/                  # (empty — all deps via platformio.ini)
```

---

## Task 1: PlatformIO Setup + config.h

**Files:**
- Create: `sunshine_receiver/platformio.ini`
- Create: `sunshine_receiver/include/config.h`

- [ ] **Step 1: Create platformio.ini**

```ini
[env:receiver]
platform = espressif32 @ >=6.0.0
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 921600
upload_speed = 921600
build_flags =
    -DCORE_DEBUG_LEVEL=0
    -DARDUINO_USB_CDC_ON_BOOT=1
```

`ARDUINO_USB_CDC_ON_BOOT=1` routes `Serial` to the native USB CDC port so we get maximum bandwidth without UART overhead.

- [ ] **Step 2: Create include/config.h**

```cpp
#pragma once
#include <stdint.h>

// ── Brain MAC address ────────────────────────────────────────────────────────
// Fill in the actual MAC from the brain's WiFi STA interface.
// Run `WiFi.macAddress()` on brain to read it.
static const uint8_t BRAIN_MAC[6] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};

// ── Timing constants ─────────────────────────────────────────────────────────
static constexpr uint32_t CTRL_TX_INTERVAL_US    = 2000;    // 500 Hz
static constexpr uint32_t HEARTBEAT_INTERVAL_MS  = 100;     // 10 Hz
static constexpr uint32_t RSSI_INTERVAL_MS       = 100;     // 10 Hz
static constexpr uint32_t HOST_WATCHDOG_US       = 3000000; // 3 s → force DISABLED
static constexpr uint32_t BRAIN_TIMEOUT_MS       = 200;     // 10 missed @50 Hz → disconnected

// ── ESP-NOW channel ──────────────────────────────────────────────────────────
static constexpr uint8_t  ESPNOW_CHANNEL = 1;
```

- [ ] **Step 3: Verify project compiles (no sources yet)**

```bash
cd sunshine_receiver && pio run --environment receiver 2>&1 | tail -10
```
Expected: errors about missing source files — not "No such file" for platformio.ini.

- [ ] **Step 4: Commit**

```bash
git add sunshine_receiver/platformio.ini sunshine_receiver/include/config.h
git commit -m "feat(receiver): add PlatformIO project scaffold and config"
```

---

## Task 2: USB Serial Framing Protocol

**Files:**
- Create: `sunshine_receiver/include/protocol.h`

- [ ] **Step 1: Create include/protocol.h**

All frame encode/decode is inline here so both `usb_bridge.cpp` and the host app can refer to the same constants.

```cpp
#pragma once
#include <stdint.h>
#include <string.h>

// ── Frame format ─────────────────────────────────────────────────────────────
// [0xAA][type:1B][len:2B LE][payload:NB][checksum:1B XOR of payload bytes]
//
// Total overhead per frame: 5 bytes (start + type + len16 + checksum)

static constexpr uint8_t FRAME_START = 0xAA;

// ── Type codes ───────────────────────────────────────────────────────────────
static constexpr uint8_t TYPE_TELEM_FRAME = 0x01; // Receiver→Host, 643 B payload
static constexpr uint8_t TYPE_CTRL_PACKET = 0x02; // Host→Receiver, 5 B payload
static constexpr uint8_t TYPE_STATUS      = 0x03; // Both,          1+≤32 B
static constexpr uint8_t TYPE_HEARTBEAT   = 0x04; // Both,          4 B
static constexpr uint8_t TYPE_RX_RSSI     = 0x05; // Receiver→Host, 1 B

// ── Status codes ─────────────────────────────────────────────────────────────
static constexpr uint8_t STATUS_OK               = 0x00;
static constexpr uint8_t STATUS_BRAIN_CONNECTED  = 0x01;
static constexpr uint8_t STATUS_BRAIN_DISCONNECTED = 0x02;
static constexpr uint8_t STATUS_LOGGING_STOPPED  = 0x03;
static constexpr uint8_t STATUS_INIT_ERROR       = 0x04;

// ── Payload sizes ─────────────────────────────────────────────────────────────
static constexpr uint16_t ESPNOW_TELEM_SIZE  = 643; // 2+1+60+580
static constexpr uint16_t CTRL_PAYLOAD_SIZE  = 5;   // mode+x+y+theta+throttle
static constexpr uint16_t HEARTBEAT_SIZE     = 4;
static constexpr uint16_t RSSI_SIZE          = 1;

// ── Packed control struct (matches ESP-NOW ctrl packet payload) ───────────────
struct __attribute__((packed)) CtrlPayload {
    uint8_t mode;
    int8_t  ctrl_x;
    int8_t  ctrl_y;
    int8_t  ctrl_theta;
    uint8_t ctrl_throttle;
};

// ── XOR checksum ─────────────────────────────────────────────────────────────
static inline uint8_t frame_checksum(const uint8_t *payload, uint16_t len) {
    uint8_t cs = 0;
    for (uint16_t i = 0; i < len; i++) cs ^= payload[i];
    return cs;
}

// ── Encode a frame into buf (must be len+5 bytes) → returns total bytes written
static inline uint16_t frame_encode(uint8_t *buf, uint8_t type,
                                    const uint8_t *payload, uint16_t len) {
    buf[0] = FRAME_START;
    buf[1] = type;
    buf[2] = (uint8_t)(len & 0xFF);
    buf[3] = (uint8_t)(len >> 8);
    memcpy(buf + 4, payload, len);
    buf[4 + len] = frame_checksum(payload, len);
    return (uint16_t)(5 + len);
}

// ── Incremental frame parser state ───────────────────────────────────────────
// Usage: call frame_parser_feed() byte by byte. When it returns true,
// out_type/out_payload/out_len are valid for one complete frame.
struct FrameParser {
    enum State { IDLE, TYPE, LEN0, LEN1, PAYLOAD, CHECKSUM } state = IDLE;
    uint8_t  type;
    uint16_t expected_len;
    uint16_t rx_count;
    uint8_t  buf[CTRL_PAYLOAD_SIZE + 8]; // max inbound payload (ctrl=5)
    uint8_t  computed_cs;
};

static inline bool frame_parser_feed(FrameParser *p, uint8_t byte,
                                     uint8_t *out_type,
                                     const uint8_t **out_payload,
                                     uint16_t *out_len) {
    switch (p->state) {
        case FrameParser::IDLE:
            if (byte == FRAME_START) p->state = FrameParser::TYPE;
            break;
        case FrameParser::TYPE:
            p->type  = byte;
            p->state = FrameParser::LEN0;
            break;
        case FrameParser::LEN0:
            p->expected_len = byte;
            p->state        = FrameParser::LEN1;
            break;
        case FrameParser::LEN1:
            p->expected_len |= ((uint16_t)byte << 8);
            p->rx_count      = 0;
            p->computed_cs   = 0;
            if (p->expected_len == 0) { p->state = FrameParser::CHECKSUM; }
            else if (p->expected_len <= sizeof(p->buf)) { p->state = FrameParser::PAYLOAD; }
            else { p->state = FrameParser::IDLE; } // oversized, drop
            break;
        case FrameParser::PAYLOAD:
            p->buf[p->rx_count++]  = byte;
            p->computed_cs        ^= byte;
            if (p->rx_count == p->expected_len) p->state = FrameParser::CHECKSUM;
            break;
        case FrameParser::CHECKSUM:
            p->state = FrameParser::IDLE;
            if (byte == p->computed_cs) {
                *out_type    = p->type;
                *out_payload = p->buf;
                *out_len     = p->expected_len;
                return true;
            }
            break;
    }
    return false;
}
```

- [ ] **Step 2: Compile check**

Create a stub `src/main.cpp` to verify the header compiles:
```cpp
// src/main.cpp (stub)
#include "config.h"
#include "protocol.h"
void setup() {}
void loop()  {}
```

```bash
cd sunshine_receiver && pio run --environment receiver 2>&1 | tail -5
```
Expected: compile success, zero errors.

- [ ] **Step 3: Commit**

```bash
git add sunshine_receiver/include/protocol.h sunshine_receiver/src/main.cpp
git commit -m "feat(receiver): define USB serial framing protocol"
```

---

## Task 3: ESP-NOW RX + Double Buffer

**Files:**
- Create: `sunshine_receiver/src/espnow_rx.cpp`

- [ ] **Step 1: Create src/espnow_rx.cpp**

```cpp
// src/espnow_rx.cpp
// Core 0: ESP-NOW receive callback — stores latest brain telemetry frame
// into a double buffer; signals Core 1 when a new frame is ready.

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

// ── Called from Core 0 ESP-NOW task ─────────────────────────────────────────
static void on_espnow_recv(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len) {
    if (len != (int)ESPNOW_TELEM_SIZE) return;  // wrong size — ignore

    // Validate first 3 bytes: frame_id(2) + type=0x01
    if (data[2] != 0x01) return;

    // Write to inactive slot, then flip
    int next = 1 - write_idx;
    memcpy(telem_buf[next], data, ESPNOW_TELEM_SIZE);
    write_idx = next;

    // Record receiver-side RSSI
    xSemaphoreTake(rssi_mutex, portMAX_DELAY);
    brain_rssi = info->rx_ctrl->rssi;
    xSemaphoreGive(rssi_mutex);

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
```

- [ ] **Step 2: Add header declarations to protocol.h**

Append to `sunshine_receiver/include/protocol.h`:
```cpp
// ── espnow_rx.cpp public API ──────────────────────────────────────────────────
void  espnow_rx_init(void);
bool  espnow_rx_get_frame(uint8_t *out_buf, uint32_t timeout_ms);
int8_t espnow_rx_get_rssi(void);
bool  espnow_rx_brain_connected(void);
void  espnow_rx_register_callback(void);
```

- [ ] **Step 3: Compile check**

```bash
cd sunshine_receiver && pio run --environment receiver 2>&1 | grep -E "error:|warning:" | head -20
```
Expected: no errors.

- [ ] **Step 4: Commit**

```bash
git add sunshine_receiver/src/espnow_rx.cpp sunshine_receiver/include/protocol.h
git commit -m "feat(receiver): ESP-NOW RX double buffer with RSSI tracking"
```

---

## Task 4: 500 Hz Timer + Control TX + Host Watchdog

**Files:**
- Create: `sunshine_receiver/src/usb_bridge.cpp`

- [ ] **Step 1: Create src/usb_bridge.cpp**

```cpp
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
    xSemaphoreTake(ctrl_mutex, 0);  // non-blocking: skip if contended (rare)
    ctrl = latest_ctrl;

    // Host watchdog: if host has been silent > 3 s, force DISABLED
    int64_t now_us = esp_timer_get_time();
    if (now_us - last_host_rx_us > (int64_t)HOST_WATCHDOG_US) {
        ctrl.mode         = 0; // DISABLED
        ctrl.ctrl_x       = 0;
        ctrl.ctrl_y       = 0;
        ctrl.ctrl_theta   = 0;
        ctrl.ctrl_throttle = 0;
        // Also update stored state so the watchdog stays triggered until host reconnects
        latest_ctrl = ctrl;
    }
    xSemaphoreGive(ctrl_mutex);

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
    last_host_rx_us = esp_timer_get_time(); // start watchdog from now

    // Configure brain peer
    memset(&brain_peer, 0, sizeof(brain_peer));
    memcpy(brain_peer.peer_addr, BRAIN_MAC, 6);
    brain_peer.channel = ESPNOW_CHANNEL;
    brain_peer.encrypt = false;
    esp_now_add_peer(&brain_peer);

    // Start 500 Hz timer
    esp_timer_create_args_t args = {};
    args.callback  = timer_500hz_cb;
    args.name      = "ctrl_tx";
    esp_timer_handle_t timer;
    esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, CTRL_TX_INTERVAL_US);
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
```

- [ ] **Step 2: Add usb_bridge API to protocol.h**

Append to `sunshine_receiver/include/protocol.h`:
```cpp
// ── usb_bridge.cpp public API ─────────────────────────────────────────────────
void usb_bridge_init(void);
void usb_bridge_tick(void);  // call from Core 1 main loop
```

- [ ] **Step 3: Compile check**

```bash
cd sunshine_receiver && pio run --environment receiver 2>&1 | grep -E "^.*error:" | head -20
```
Expected: no errors.

- [ ] **Step 4: Commit**

```bash
git add sunshine_receiver/src/usb_bridge.cpp sunshine_receiver/include/protocol.h
git commit -m "feat(receiver): 500 Hz control TX, USB bridge, host watchdog"
```

---

## Task 5: main.cpp — ESP-NOW Init + Task Creation

**Files:**
- Modify: `sunshine_receiver/src/main.cpp`

- [ ] **Step 1: Replace stub main.cpp with full implementation**

```cpp
// src/main.cpp
// Receiver firmware: bridges brain (ESP-NOW) ↔ host app (USB serial)
//
// Core 0: FreeRTOS task — ESP-NOW RX callback (registered via esp_now, runs on
//         Core 0 by default in Arduino framework)
// Core 1: bridge_task — USB TX/RX + 500 Hz timer (timer runs on its own task
//         but we drive the USB I/O from this loop)

#include "config.h"
#include "protocol.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ── Bridge task (Core 1) ──────────────────────────────────────────────────────
static void bridge_task(void *) {
    for (;;) {
        usb_bridge_tick();
        vTaskDelay(pdMS_TO_TICKS(1));  // yield ~1 ms between ticks (USB is buffered)
    }
}

void setup() {
    Serial.begin(921600);
    delay(100);

    // ── WiFi: STA mode, fixed channel 1, max power ────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(84); // 21 dBm, maximum

    // ── ESP-NOW init ──────────────────────────────────────────────────────────
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        // Blink fast indefinitely to signal error
        while (true) {
            delay(100);
        }
    }
    esp_now_set_pmk((uint8_t *)"pmk_not_used_000"); // required call; PMK unused

    // ── Initialise subsystems ─────────────────────────────────────────────────
    espnow_rx_init();
    espnow_rx_register_callback();
    usb_bridge_init();

    // ── Start bridge task pinned to Core 1 ───────────────────────────────────
    xTaskCreatePinnedToCore(bridge_task, "bridge", 8192, nullptr, 5, nullptr, 1);

    Serial.println("Receiver ready");
}

void loop() {
    // All work is in bridge_task and the esp_timer callback.
    // loop() runs on Core 1 too but we keep it empty.
    vTaskDelay(portMAX_DELAY);
}
```

- [ ] **Step 2: Full compile + link**

```bash
cd sunshine_receiver && pio run --environment receiver 2>&1 | tail -15
```
Expected:
```
Building .pio/build/receiver/firmware.bin
Linking .pio/build/receiver/firmware.elf
RAM:   [=         ]   X.X% (...)
Flash: [====      ]   X.X% (...)
```

- [ ] **Step 3: Flash and verify serial output**

Connect receiver ESP32-S3 via USB. Then:
```bash
pio run --target upload --environment receiver
pio device monitor --baud 921600
```
Expected: `Receiver ready` printed. No crash/reset loop.

- [ ] **Step 4: Commit**

```bash
git add sunshine_receiver/src/main.cpp
git commit -m "feat(receiver): integrate ESP-NOW + USB bridge into main.cpp"
```

---

## Task 6: Bringup Level 3 Integration Test

This task verifies the full brain ↔ receiver ↔ host pipeline works end to end. Done manually once the brain firmware is also at bringup level 3.

- [ ] **Step 1: Flash brain at bringup_3_telemetry**

Follow bringup level 3 instructions in `docs/BRINGUP.md` (written in sunshine_brain plan).

- [ ] **Step 2: Connect receiver to host, open host app**

Open host app → Live tab → select receiver serial port → click Connect.  
Expected:
- Status shows "Receiver connected"
- Within 200ms: "Brain connected" status
- `inputs.accel_z` plotting ≈ +20 counts at rest

- [ ] **Step 3: Validate watchdog**

Disconnect host app (close serial connection) for >3 seconds, then reconnect.  
Expected:
- Brain receives `mode=DISABLED` during the 3-second window
- Host shows "Receiver connected" + "Brain connected" after reconnect
- A new log file is started

- [ ] **Step 4: Validate RSSI**

Walk receiver 5 meters away from brain.  
Expected: `inputs.rssi` (brain-side) and `RX_RSSI` (receiver-side) both decrease. 

- [ ] **Step 5: Commit BRINGUP notes**

```bash
# Add any config.h MAC address corrections discovered during testing
git add sunshine_receiver/include/config.h
git commit -m "chore(receiver): fill in actual brain MAC address after bringup-3 test"
```

---

*End of receiver plan. Next: `2026-05-26-sunshine-brain.md`.*
