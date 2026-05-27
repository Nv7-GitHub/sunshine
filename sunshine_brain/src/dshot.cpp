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

static DShotRMT dshot_left (PIN_DSHOT_LEFT,  DSHOT600, true);
static DShotRMT dshot_right(PIN_DSHOT_RIGHT, DSHOT600, true);

static float erpm_left_val  = 0.0f;
static float erpm_right_val = 0.0f;

bool dshot_init(void) {
    dshot_result_t res_l = dshot_left.begin();
    dshot_result_t res_r = dshot_right.begin();
    if (!res_l.success || !res_r.success) return false;

    // Arming sequence: send disarm (0) for 300 ms so ESC recognises the controller.
    for (int i = 0; i < 300; i++) {
        dshot_left .sendThrottle(0);
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

    dshot_left .sendThrottle(left);
    dshot_right.sendThrottle(right);
}

float dshot_erpm_left(void)  { return erpm_left_val;  }
float dshot_erpm_right(void) { return erpm_right_val; }
