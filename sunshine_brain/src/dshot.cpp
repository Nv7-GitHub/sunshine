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

// ── eRPM sanitising filter ──────────────────────────────────────────────────
// Bidirectional-DShot telemetry carries only a weak 4-bit GCR checksum, so a
// few percent of frames decode to wrong values: implausibly LOW eRPM (corrupt
// = oversized period) or absurdly HIGH (tiny period → getTelemetry returns
// ~6e7 eRPM, which then overflows the float16 telemetry field to +inf). Both
// show up as the "dropouts" and inf spikes seen in recorded logs.
//
// The flywheel's huge rotational inertia means TRUE eRPM changes slowly, so any
// large single-sample jump is a decode error, not real motion. Three-stage
// clean-up, modelled and validated against logs/2026-06-12_..._spiritridge.sun
// (driven-motor envelope-outlier rate fell from ~15% to ~0.2%, zero inf out):
//   1. range-gate : reject decodes outside [0, ERPM_MAX]      — kills +inf source
//   2. deviation-gate : reject a decode that deviates > {LO,HI}× the running
//        median (only once the motor is clearly spinning, median > GATE_MIN) —
//        rejects the low/high garbage cloud that a plain median can't outvote
//   3. median-5 over accepted decodes                         — smooths residue
// An ESCAPE counter prevents the gate from latching: after ERPM_ESCAPE
// consecutive out-of-band decodes (a genuine fast change, e.g. a hard brake)
// the ring is flushed to the new level. eRPM is telemetry-only (never read by
// the Kalman/control), so the few-sample median lag is irrelevant to behaviour.
static constexpr float ERPM_MAX      = 65000.0f;  // ~1100KV·8.5V·7pp; < f16 finite max (65504)
static constexpr int   ERPM_MED_N    = 5;
static constexpr float ERPM_GATE_MIN = 3000.0f;   // below this the gate is off (arming/idle)
static constexpr float ERPM_GATE_LO  = 0.5f;      // accept band: [LO, HI] × running median
static constexpr float ERPM_GATE_HI  = 1.7f;
static constexpr int   ERPM_ESCAPE   = 6;         // consecutive rejects → accept (real change)

struct ErpmFilter {
    float ring[ERPM_MED_N] = {0, 0, 0, 0, 0};
    int   count  = 0;
    int   reject = 0;
    float median() const {
        if (count == 0) return 0.0f;
        float tmp[ERPM_MED_N];
        for (int i = 0; i < count; i++) tmp[i] = ring[i];
        for (int i = 1; i < count; i++) {  // insertion sort (count ≤ 5)
            float k = tmp[i]; int j = i - 1;
            while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
            tmp[j + 1] = k;
        }
        return tmp[count / 2];
    }
    // Feed one freshly-decoded eRPM; returns the current sanitised value.
    float push(float v) {
        if (!(v >= 0.0f && v <= ERPM_MAX)) return median();  // out of range → hold
        float m = median();
        if (count > 0 && m > ERPM_GATE_MIN && (v < ERPM_GATE_LO * m || v > ERPM_GATE_HI * m)) {
            if (++reject < ERPM_ESCAPE) return median();      // transient outlier → reject/hold
            for (int i = 0; i < ERPM_MED_N; i++) ring[i] = v; // sustained → flush to new level
            count = ERPM_MED_N; reject = 0; return v;
        }
        reject = 0;
        for (int i = ERPM_MED_N - 1; i > 0; i--) ring[i] = ring[i - 1];
        ring[0] = v;
        if (count < ERPM_MED_N) count++;
        return median();
    }
};
static ErpmFilter erpm_filt_left, erpm_filt_right;

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
    // Sanitise each freshly-decoded eRPM (see ErpmFilter above): range-gate +
    // deviation-gate + median. push() handles all rejection internally and
    // always returns the current clean value, so bad frames can't poison the
    // logged/telemetered stream.
    if (telem_l.success) erpm_left_val  = erpm_filt_left .push((float)telem_l.erpm);
    if (telem_r.success) erpm_right_val = erpm_filt_right.push((float)telem_r.erpm);

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
