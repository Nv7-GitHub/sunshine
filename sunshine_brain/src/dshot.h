#pragma once
#include <stdint.h>
#include <stdbool.h>

bool        dshot_init(void);                          // init both ESCs; returns false on error
const char *dshot_last_error(void);                    // human-readable init failure details
// DShot values 0–2047; 0=disarm. `v_batt` is the pack voltage in volts and is
// used ONLY to compute the eRPM plausibility ceiling (a wheel cannot exceed
// duty·V_batt·KV·pole_pairs by more than a regen margin) — it never influences
// the command that is actually sent. The default of 0.0f means "battery
// unknown" and disables the command-derived ceiling entirely (fail-open, same
// policy as the cap's §3.8 battery handling), leaving only the fixed ERPM_MAX
// f16 guard; main.cpp's bringup-level-2 motor test has no battery reading and
// relies on that default, so it must be kept.
//
// Only `left`/`right` are passed, not a duty: the duty is derived inside
// dshot.cpp from the wire values so the inverted-motor reverse-zone mapping
// that nav_control's motor_cmd() applies lives in exactly one place. Note the
// gate necessarily uses the PREVIOUS tick's duty — dshot_send() reads pending
// telemetry before sending the new throttle, so a decode arriving now was
// produced by the throttle sent on the previous call.
void  dshot_send(uint16_t left, uint16_t right, float v_batt = 0.0f);
// Return NaN when the ESC is not commutating (undriven / coasting): NaN means
// "no measurement", 0 now means a genuinely stopped wheel (spec §2.5 / §4.1).
// nav_control float16-encodes these, and sunshine_f32_to_f16() preserves the
// mantissa so the NaN survives into the log rather than collapsing to +inf.
float dshot_erpm_left(void);
float dshot_erpm_right(void);
void  dshot_print_telem_debug(void);                   // print raw getTelemetry result codes
void  dshot_dump_rx_frames(void);                      // dump last RX frame symbols (durations)

// Quantise [0.0, 2047.0] → uint8 for SunshineInput
static inline uint8_t dshot_quantize(float v) {
    int q = (int)(v * (255.0f / 2047.0f) + 0.5f);
    return (uint8_t)(q < 0 ? 0 : q > 255 ? 255 : q);
}
