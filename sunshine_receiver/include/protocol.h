#pragma once
#include <stdint.h>
#include <string.h>

// ── Frame format ─────────────────────────────────────────────────────────────
// [0xAA][type:1B][len:2B LE][payload:NB][checksum:1B XOR of payload bytes]
//
// Total overhead per frame: 5 bytes (start + type + len16 + checksum)

static constexpr uint8_t FRAME_START = 0xAA;

// ── Type codes ───────────────────────────────────────────────────────────────
static constexpr uint8_t TYPE_TELEM_FRAME = 0x01; // Receiver→Host, 703 B payload
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
// Brain sends 20 inputs per ESP-NOW v2 packet (50 Hz; IDF >= 5.4 lifts the old
// 250-byte cap to 1490). Two SunshineState snapshots per frame give 100 Hz real
// state; vars are not sent (host recomputes them). Layout: 2 (frame_id) + 1
// (type) + 2*60 (SunshineState) + 20*29 (SunshineInput) = 703. Keep in sync with
// brain telemetry.cpp FRAME_SIZE.
static constexpr uint16_t ESPNOW_TELEM_SIZE  = 703;
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

// ── espnow_rx.cpp public API ──────────────────────────────────────────────────
void  espnow_rx_init(void);
bool  espnow_rx_get_frame(uint8_t *out_buf, uint32_t timeout_ms);
void  espnow_rx_update_rssi(int8_t rssi); // call from sniffer task (IDF 4.x only)
int8_t espnow_rx_get_rssi(void);
bool  espnow_rx_brain_connected(void);
void  espnow_rx_register_callback(void);

// ── usb_bridge.cpp public API ─────────────────────────────────────────────────
void usb_bridge_init(void);
void usb_bridge_tick(void);  // call from Core 1 main loop
