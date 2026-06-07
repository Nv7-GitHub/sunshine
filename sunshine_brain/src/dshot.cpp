#include "dshot.h"
#include "config.h"
#include <DShotRMT.h>
#include <Arduino.h>

// DShotRMT 0.9.5 API:
//   Constructor: DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
//   begin()       → dshot_result_t
//   sendThrottle(uint16_t throttle) → dshot_result_t  (range 48–2047; 0 = disarm/motor-stop command)
//   getTelemetry() → dshot_result_t  (.erpm field populated when bidirectional)
//
// Bidirectional is enabled so eRPM telemetry is available on the same wire.
// Motor magnet count uses library default (14) — adjust to match actual motors if known.

// AM32 auto-detects bidirectional DShot from the inverted signal.
// DSHOT_BIDIRECTIONAL is set in config.h — 1 enables eRPM telemetry.
// DSHOT600 halves the bit period vs DSHOT300, so each TX frame (and the
// blocking rmt_tx_wait_all_done in _sendPacket) is ~2× shorter — important for
// keeping the 1 kHz nav loop under budget while still capturing eRPM telemetry.
static DShotRMT dshot_left (PIN_DSHOT_LEFT,  DSHOT600, DSHOT_BIDIRECTIONAL);
static DShotRMT dshot_right(PIN_DSHOT_RIGHT, DSHOT600, DSHOT_BIDIRECTIONAL);

static float erpm_left_val  = 0.0f;
static float erpm_right_val = 0.0f;

static char dshot_err[64];
const char *dshot_last_error(void) { return dshot_err; }

static uint32_t dshot_normalized_erpm(uint16_t packed) {
    // Bidirectional DShot encodes eRPM as [eee][mmmmmmmmm].
    uint16_t exponent = (packed >> 9) & 0x7;
    uint16_t value    = packed & 0x1FF;
    return static_cast<uint32_t>(value << exponent);
}

bool dshot_init(void) {
    // Force AM32 to reset its arm/protocol state, mimicking a brain power-cycle.
    // On a warm brain reset (reflash) the ESC stays powered and the 5.1k pull-up
    // holds the inverted-DShot line HIGH (idle) the whole time, so the ESC never
    // sees a disconnect and stays latched/unresponsive (no telemetry) until it is
    // physically power-cycled. Powering the brain off drops the line LOW, which IS
    // what makes the ESC reset and re-arm. Reproduce that here: drive the lines LOW
    // briefly (a sustained low = "signal lost") before RMT takes over, so the ESC
    // resets and then re-arms from the zero-throttle burst below — no power-cycle.
    pinMode(PIN_DSHOT_LEFT,  OUTPUT); digitalWrite(PIN_DSHOT_LEFT,  LOW);
    pinMode(PIN_DSHOT_RIGHT, OUTPUT); digitalWrite(PIN_DSHOT_RIGHT, LOW);
    delay(500);

    dshot_result_t res_l = dshot_left.begin();
    dshot_result_t res_r = dshot_right.begin();
    snprintf(dshot_err, sizeof(dshot_err), "L=%s(%d) R=%s(%d)",
        res_l.success ? "OK" : "FAIL", res_l.result_code,
        res_r.success ? "OK" : "FAIL", res_r.result_code);
    if (!res_l.success || !res_r.success) return false;

    // Send ~150ms of zero-throttle frames so AM32 completes its arm handshake
    // before loop() runs (same as a clean cold boot, now that the LOW pulse above
    // has reset the ESC).
    for (int i = 0; i < 150; i++) {
        dshot_left.sendThrottle(0);
        dshot_right.sendThrottle(0);
        delay(1);
    }
    return true;
}

void dshot_send(uint16_t left, uint16_t right) {
    // Retrieve any pending telemetry before the next transmission window.
    dshot_result_t telem_l = dshot_left .getTelemetry();
    dshot_result_t telem_r = dshot_right.getTelemetry();
    if (telem_l.success) erpm_left_val  = (float)telem_l.erpm;
    if (telem_r.success) erpm_right_val = (float)telem_r.erpm;

    dshot_result_t res_l = dshot_left .sendThrottle(left);
    dshot_result_t res_r = dshot_right.sendThrottle(right);

    static uint32_t err_count = 0;
    if (!res_l.success || !res_r.success) {
        if (err_count < 10)
            Serial.printf("DSHOT ERR L=%d(%d) R=%d(%d)\n",
                res_l.success, res_l.result_code,
                res_r.success, res_r.result_code);
        err_count++;
    }
}

float dshot_erpm_left(void)  { return erpm_left_val;  }
float dshot_erpm_right(void) { return erpm_right_val; }

extern volatile uint32_t g_dshot_rx_cb_count;
extern volatile uint32_t g_dshot_rx_sym_last;
extern volatile uint32_t g_dshot_rx_crc_ok;
extern volatile uint32_t g_dshot_rx_crc_fail;

void dshot_print_telem_debug(void) {
    dshot_result_t tl = dshot_left.getTelemetry();
    dshot_result_t tr = dshot_right.getTelemetry();
    Serial.printf("TELEM L: success=%d code=%d erpm=%lu  R: success=%d code=%d erpm=%lu\n",
        tl.success, tl.result_code, tl.erpm,
        tr.success, tr.result_code, tr.erpm);
    Serial.printf("PACKED L: raw=0x%03x exp=%u mant=0x%03x norm_erpm=%lu  R: raw=0x%03x exp=%u mant=0x%03x norm_erpm=%lu\n",
        tl.telemetry_available ? tl.telemetry_data.rpm : 0,
        tl.telemetry_available ? ((tl.telemetry_data.rpm >> 9) & 0x7) : 0,
        tl.telemetry_available ? (tl.telemetry_data.rpm & 0x1FF) : 0,
        tl.telemetry_available ? dshot_normalized_erpm(tl.telemetry_data.rpm) : 0,
        tr.telemetry_available ? tr.telemetry_data.rpm : 0,
        tr.telemetry_available ? ((tr.telemetry_data.rpm >> 9) & 0x7) : 0,
        tr.telemetry_available ? (tr.telemetry_data.rpm & 0x1FF) : 0,
        tr.telemetry_available ? dshot_normalized_erpm(tr.telemetry_data.rpm) : 0);
    Serial.printf("RX ISR: fired=%lu last_sym=%lu crc_ok=%lu crc_fail=%lu\n",
        g_dshot_rx_cb_count, g_dshot_rx_sym_last, g_dshot_rx_crc_ok, g_dshot_rx_crc_fail);
}

void dshot_dump_rx_frames(void) {
    Serial.println("LEFT motor last RX frame:");
    dshot_left.dumpLastRxFrame();
    Serial.println("RIGHT motor last RX frame:");
    dshot_right.dumpLastRxFrame();
}
