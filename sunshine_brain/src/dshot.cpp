#include "dshot.h"
#include "config.h"
#include <sunshine_core.h>   // DSHOT_NEUTRAL/DSHOT_MAX, MOTOR_KV_RPM_PER_V, MOTOR_POLE_PAIRS
#include <math.h>            // NAN
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

// NaN, not 0: before the first successful decode there is no measurement, and
// 0.0f would now be read downstream as "the wheel is stopped" (see §2.5).
static float erpm_left_val  = NAN;
static float erpm_right_val = NAN;

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
// An ESCAPE counter prevents the gate from latching: after enough consecutive
// out-of-band decodes (a genuine fast change) the ring is flushed to the new
// level. eRPM is telemetry-only (never read by the Kalman/control), so the
// few-sample median lag is irrelevant to behaviour.
//
// Three further defects were found in 2026-07-20_04-20-24_translation2.sun and
// are fixed here (design §4.2):
//   1. the escape was direction-agnostic, so a downward garbage run flushed the
//      ring to a bogus low level (the t=402.9 s collapse) — now asymmetric;
//   2. the ceiling was a fixed 65000, which only ever catches the +inf case —
//      now derived from the command actually sent;
//   3. undriven decodes were fed to the filter at all — now they are discarded
//      and the value is NaN ("not measurable"), not 0 ("stopped").
static constexpr float ERPM_MAX      = 65000.0f;  // ~1100KV·8.5V·7pp; < f16 finite max (65504)
static constexpr int   ERPM_MED_N    = 5;
static constexpr float ERPM_GATE_MIN = 3000.0f;   // below this the gate is off (arming/idle)
static constexpr float ERPM_GATE_LO  = 0.5f;      // accept band: [LO, HI] × running median
static constexpr float ERPM_GATE_HI  = 1.7f;

// Escape thresholds are ASYMMETRIC because the physics is. Spin-up really is
// fast — a wheel unloaded in the air reaches its commanded no-load speed within
// a few ms — so an upward run of 6 is still the right trigger. A DECREASE of
// the same size is physically impossible: the flywheel cannot lose half its
// speed in 6 ms, and the coast-down window at t=402.9 s in the evidence log
// (body ω flat at −94 rad/s while eRPM "fell" 18416 → 3408) is exactly the
// failure the symmetric escape let through. Requiring ~50 ms of sustained
// downward evidence rejects it. NB the counter counts consecutive out-of-band
// *decodes*, not ticks, so dropped frames stretch the real window beyond 50 ms
// — that errs toward rejection, which is the safe direction.
static constexpr int   ERPM_ESCAPE_UP   = 6;      // ~6 ms at the 1 kHz send rate
static constexpr int   ERPM_ESCAPE_DOWN = 50;     // ~50 ms — flywheel inertia bound

// Commutation threshold. Round, per-build number — deliberately NOT fitted to
// log data. Below ~5 % duty AM32 turns the FETs off and coasts, so there is no
// commutation to time and the ESC emits a decaying, meaningless period (§2.5:
// 77.6 % of neutral-command samples read zero). 5 % ≈ 50 DShot steps ≈ 0.4 V at
// 8 V, whose no-load eRPM is 0.05·8·1100·7 ≈ 3080 — i.e. it lands on
// ERPM_GATE_MIN by construction, so the two thresholds agree rather than
// fighting: anything the duty gate lets through is also where the median gate
// switches on.
static constexpr float ERPM_MIN_DUTY = 0.05f;

// A wheel back-driven by the flywheel (landing, or the braking half of the
// MELTY drift wave) regenerates and can turn faster than the speed its own
// command implies, so the ceiling needs headroom above the no-load model.
// 1.5× covers that while still rejecting the 25504-while-coasting spike and the
// >40000 driven spikes (0.07 % of decodes) that a fixed 65000 never catches.
static constexpr float ERPM_REGEN_MARGIN = 1.5f;

// Plausibility window on the battery reading. Outside it (including the 0.0f
// "unknown" default) the command-derived ceiling is skipped entirely and only
// ERPM_MAX applies — fail-open, the same policy §3.8 uses for the speed cap: a
// bad voltage reading must degrade the filter, not blind the telemetry.
static constexpr float ERPM_VBATT_LO = 5.0f;
static constexpr float ERPM_VBATT_HI = 10.0f;

// After this many consecutive undriven ticks the median ring is dropped; see
// ErpmFilter::idle().
static constexpr int   ERPM_STALE_TICKS = 50;

struct ErpmFilter {
    float ring[ERPM_MED_N] = {0, 0, 0, 0, 0};
    int   count  = 0;
    int   reject = 0;
    bool  reject_up = false;  // direction of the rejection run currently in progress
    int   undriven  = 0;      // consecutive undriven (non-commutating) ticks
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
    // Feed one freshly-decoded eRPM from a DRIVEN motor; returns the current
    // sanitised value. `ceiling` is the caller's per-tick plausibility bound
    // (erpm_ceiling(), already clamped to ERPM_MAX).
    float push(float v, float ceiling) {
        // `undriven` counts CONSECUTIVE undriven ticks, and push() is only reached
        // on a driven tick, so reaching here breaks any run in progress — reset it
        // BEFORE the gates, so a run is broken by the ESC commutating again and not
        // by whether this particular decode happened to survive. Without this reset
        // the counter is a cumulative lifetime count that latches at
        // ERPM_STALE_TICKS during the pre-arm idle stretch of every run (prev_duty_*
        // start at 0, so idle() runs from boot), after which idle() drops the ring
        // on the FIRST undriven tick and the freeze documented in idle() never
        // happens at all — the exact re-acquire-every-wave-cycle behaviour it exists
        // to prevent.
        undriven = 0;
        // Range gate, now against the command-derived ceiling. There is
        // deliberately NO escape hatch here: unlike the deviation gate, whose
        // reference is the filter's own history and can therefore be stale, the
        // ceiling is recomputed from physics (duty × V_batt × KV) every tick. A
        // value that stays above it means the decode is wrong, not that the
        // model has fallen behind, so flushing to it could never be right.
        // NB "hold" means hold the PREVIOUS value, and with an empty ring there is
        // no previous value — median() would fabricate a 0, which under §4.1 now
        // reads downstream as "the wheel is genuinely stopped". That is exactly the
        // fake zero NaN was introduced to eliminate, and the empty-ring case is not
        // exotic: it is the state at boot and after every idle() stale-drop, i.e.
        // when a still-coasting wheel is re-driven at a low duty and its (true)
        // decode legitimately sits above the freshly-computed low ceiling.
        if (!(v >= 0.0f && v <= ceiling)) return count > 0 ? median() : NAN;
        float m = median();
        if (count > 0 && m > ERPM_GATE_MIN && (v < ERPM_GATE_LO * m || v > ERPM_GATE_HI * m)) {
            // Direction-aware escape: an upward run flushes fast, a downward run
            // must be sustained (see ERPM_ESCAPE_UP/DOWN above). A change of
            // direction RESTARTS the run rather than extending it, so an
            // oscillating garbage cloud — which is what a stream of bad GCR
            // decodes looks like — can never accumulate its way to an escape;
            // only a consistent, one-directional trend can.
            bool up = (v > m);
            if (reject == 0 || up != reject_up) { reject = 1; reject_up = up; }
            else                                { reject++; }
            if (reject < (up ? ERPM_ESCAPE_UP : ERPM_ESCAPE_DOWN))
                return median();                              // transient outlier → reject/hold
            for (int i = 0; i < ERPM_MED_N; i++) ring[i] = v; // sustained → flush to new level
            count = ERPM_MED_N; reject = 0; return v;
        }
        reject = 0;
        for (int i = ERPM_MED_N - 1; i > 0; i--) ring[i] = ring[i - 1];
        ring[0] = v;
        if (count < ERPM_MED_N) count++;
        return median();
    }
    // The undriven path: the ESC is coasting with its FETs off, so whatever it
    // decoded this tick is a meaningless decaying period. Feed the ring nothing
    // and report "no measurement".
    //
    // The ring is FROZEN rather than cleared immediately because the MELTY drift
    // wave routinely dips one wheel below the duty threshold for a tick or two;
    // re-clearing on every dip would throw away a perfectly good speed level and
    // force the deviation gate to re-acquire from scratch each wave cycle. After
    // ERPM_STALE_TICKS (~50 ms) the opposite risk dominates — the wheel may
    // genuinely have decayed — so the stale median is dropped, otherwise it
    // would gate (and reject) the true speed when drive resumes.
    float idle() {
        reject = 0;
        if (undriven < ERPM_STALE_TICKS) undriven++;
        else                             count = 0;
        return NAN;
    }
};
static ErpmFilter erpm_filt_left, erpm_filt_right;

// Command state from the PREVIOUS dshot_send(). The ordering matters: within one
// call telemetry is read BEFORE the new throttle is sent, so a decode arriving
// now was produced by the throttle sent on the previous call. Gating it against
// the command being sent this call would be off by one tick — which is exactly
// the tick that matters at the drive/coast boundary — so the gate uses these
// saved values instead.
static float prev_duty_left = 0.0f, prev_duty_right = 0.0f, prev_v_batt = 0.0f;

// Magnitude of the applied duty implied by a wire value, covering both AM32 3D
// zones. BOTH zones run in the SAME direction — speed increases with the wire
// value away from that zone's minimum:
//   forward [1048..2047]: 1048 = stopped, 2047 = full;
//   reverse [48..1047]  : 48   = slowest, 1047 = full.
// nav_control's motor_cmd() preserves that ordering when it inverts a motor: it
// SHIFTS by 1001 (min fwd 1049 → min rev 48, max fwd 2047 → max rev 1046),
// deliberately not reflecting around neutral, "keeping slow↔slow and fast↔fast".
// An earlier version of this function assumed the reverse zone ran backwards and
// computed (1047 − cmd)/999, i.e. 1 − duty. Since both motors are inverted
// (config.h) every MELTY command lands in the reverse zone, so that mistake made
// full spin read as ~0 duty (→ `driven` false → eRPM NaN for the whole match)
// and neutral read as ~1.0 (→ erpm_ceiling() saturated to ERPM_MAX, disabling
// the very ceiling §4.2 added). Both halves of the eRPM hardening were inverted.
// Derived here rather than at the call site so the mapping lives in one place.
static inline float dshot_duty(uint16_t cmd) {
    if (cmd == 0) return 0.0f;  // disarm
    if (cmd > DSHOT_NEUTRAL) return ((float)cmd - DSHOT_NEUTRAL) / (DSHOT_MAX - DSHOT_NEUTRAL);
    if (cmd < DSHOT_NEUTRAL) return ((float)cmd - DSHOT_MIN)     / (DSHOT_MAX - DSHOT_NEUTRAL);
    return 0.0f;                // exactly neutral
}

// Plausibility ceiling for a decode produced by `duty` at `v_batt`. This is the
// same open-loop-voltage-source model the wheel-speed cap inverts (§2.2/§3.1):
// the ESC applies duty×V_batt to the winding and the motor settles at the
// no-load speed that voltage implies. KV is MECHANICAL rpm/V, so the pole-pair
// factor converts it to the electrical rpm the ESC actually reports.
static inline float erpm_ceiling(float duty, float v_batt) {
    if (!(v_batt >= ERPM_VBATT_LO && v_batt <= ERPM_VBATT_HI))
        return ERPM_MAX;  // unknown/implausible battery (incl. the 0.0f default) → fail open
    float c = duty * v_batt * MOTOR_KV_RPM_PER_V * MOTOR_POLE_PAIRS * ERPM_REGEN_MARGIN;
    return c < ERPM_MAX ? c : ERPM_MAX;
}

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

void dshot_send(uint16_t left, uint16_t right, float v_batt) {
    // Was the ESC actually commutating when this decode was produced? The gate
    // is on the PREVIOUS tick's duty, because the telemetry read below happens
    // before this tick's throttle is sent (see prev_duty_* above).
    bool driven_l = prev_duty_left  >= ERPM_MIN_DUTY;
    bool driven_r = prev_duty_right >= ERPM_MIN_DUTY;

    // Retrieve any pending telemetry before the next transmission window.
    dshot_result_t telem_l = dshot_left .getTelemetry();
    dshot_result_t telem_r = dshot_right.getTelemetry();
    // Sanitise each freshly-decoded eRPM (see ErpmFilter above): commutation
    // gate → range-gate against the command-derived ceiling → deviation-gate →
    // median. push() handles all rejection internally and always returns the
    // current clean value, so bad frames can't poison the logged/telemetered
    // stream.
    //
    // When the motor was NOT driven the decode is discarded regardless of
    // telem.success — a successful GCR decode of a coasting ESC is still a
    // meaningless number (§2.5) — and NaN is emitted unconditionally. This is
    // the single place NaN is produced. When the motor WAS driven but the frame
    // failed to decode, the previous value is held, exactly as before.
    if (!driven_l)            erpm_left_val  = erpm_filt_left .idle();
    else if (telem_l.success) erpm_left_val  = erpm_filt_left .push((float)telem_l.erpm,
                                                   erpm_ceiling(prev_duty_left,  prev_v_batt));
    if (!driven_r)            erpm_right_val = erpm_filt_right.idle();
    else if (telem_r.success) erpm_right_val = erpm_filt_right.push((float)telem_r.erpm,
                                                   erpm_ceiling(prev_duty_right, prev_v_batt));

    dshot_result_t res_l = dshot_left .sendThrottle(left);
    dshot_result_t res_r = dshot_right.sendThrottle(right);

    // Remember what we just sent so next tick's gate can be paired with the
    // command that actually produced the decode. Placed after the sends (rather
    // than before the telemetry read) so the one-tick pairing is obvious from
    // the code order. dshot_init()'s 150 zero-throttle arming frames bypass this
    // function entirely, so these stay 0 and the first dshot_send() correctly
    // reports NaN rather than trusting a decode from an unknown command.
    prev_duty_left  = dshot_duty(left);
    prev_duty_right = dshot_duty(right);
    prev_v_batt     = v_batt;

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
