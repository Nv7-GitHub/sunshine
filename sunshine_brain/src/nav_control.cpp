#include "nav_control.h"
#include "config.h"
#include "bringup.h"
#include "sensors/adxl375.h"
#include "sensors/lis3mdl.h"
#include "dshot.h"
#include "telemetry.h"
#include <sunshine_core.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static SunshineState kf_state;
static SunshineVars  vars;
static uint32_t      overrun_count = 0;

void nav_control_init(void) {
    sunshine_state_init(&kf_state);
}

void nav_control_task(void *) {
    uint32_t t_next = micros();

    for (;;) {
        vars.loop_overrun = false;  // clear each tick
        uint32_t t_start = micros();

        // ── 1. Build SunshineInput ──────────────────────────────────────
        SunshineInput in = {};
        in.time_us = t_start;

        // Sensors
        Adxl375Sample a = adxl375_read();
        MagSample     m = lis3mdl_read();
        float         v = batt_read_v();

        in.accel_x = a.x;
        in.accel_y = a.y;
        in.accel_z = a.z;
        in.mag_x   = m.x;
        in.mag_y   = m.y;
        in.mag_z   = m.z;

        // Battery voltage → batt_offset (see sunshine_core.h for scale)
        // batt_offset encodes (voltage - 7.6V) / 0.0205V per LSB
        float batt_offset_f = (v - BATT_OFFSET_REF_V) / BATT_SCALE_V;
        in.batt_offset = (int8_t)(batt_offset_f < -128.0f ? -128 :
                                  batt_offset_f >  127.0f ?  127 :
                                  (int8_t)batt_offset_f);

        // eRPM as float16 using sunshine_core helper
        in.erpm_left  = sunshine_f32_to_f16(dshot_erpm_left());
        in.erpm_right = sunshine_f32_to_f16(dshot_erpm_right());

        // ── 2. Control inputs (from telemetry task via mutex) ───────────
        CtrlInputs ctrl = telemetry_get_ctrl();

        // Safety watchdog: if no control packet for CTRL_WATCHDOG_MS, force DISABLED
        uint32_t now_ms = (uint32_t)millis();
        if (now_ms - ctrl.last_rx_ms > CTRL_WATCHDOG_MS) {
            in.mode = SUNSHINE_MODE_DISABLED;
        } else {
            in.mode = ctrl.mode;
        }
        in.ctrl_x        = ctrl.ctrl_x;
        in.ctrl_y        = ctrl.ctrl_y;
        in.ctrl_theta    = ctrl.ctrl_theta;
        in.ctrl_throttle = ctrl.ctrl_throttle;
        in.rssi          = ctrl.rssi;

        // ── 3. Previous DShot command (quantised from last tick) ─────────
        in.dshot_left_q  = dshot_quantize(vars.dshot_cmd_left);
        in.dshot_right_q = dshot_quantize(vars.dshot_cmd_right);

        // ── 4. sunshine_step ──────────────────────────────────────────────
        sunshine_step(&in, &kf_state, &vars);

        // Safety: zero DShot at bringup levels 3-4
#if FORCE_DSHOT_ZERO
        vars.dshot_cmd_left  = 0.0f;
        vars.dshot_cmd_right = 0.0f;
#endif

        // ── 5. Apply outputs ──────────────────────────────────────────────
#if FEATURE_DSHOT
        auto motor_cmd = [](float cmd, bool invert) -> uint16_t {
            if (cmd == 0.0f) return 0;  // preserve disarm
            float c = invert ? (2.0f * DSHOT_NEUTRAL - cmd) : cmd;
            return (uint16_t)(c + 0.5f);
        };
        dshot_send(motor_cmd(vars.dshot_cmd_left,  MOTOR_LEFT_INVERT),
                   motor_cmd(vars.dshot_cmd_right, MOTOR_RIGHT_INVERT));
#endif
        analogWrite(PIN_LED, vars.led_on ? LED_DUTY : 0);

        // ── 6. Push to telemetry ring buffer ──────────────────────────────
#if FEATURE_TELEMETRY
        bool pushed = telemetry_push(&in, &kf_state);
        if (!pushed) vars.loop_overrun = true;  // set if current tick overran OR telem ring full
#endif

        // ── 7. Timing: busy-wait until next 1 kHz tick ───────────────────
        uint32_t elapsed = micros() - t_start;
        if (elapsed >= LOOP_INTERVAL_US) {
            overrun_count++;
            t_next = micros();  // re-sync BEFORE printf to exclude print latency
            vars.loop_overrun = true;  // set if current tick overran OR telem ring full
            if (overrun_count <= 3 || (overrun_count % 100) == 0) {
                Serial.printf("OVERRUN: %u µs (total=%u)\n", elapsed, overrun_count);
            }
        } else {
            overrun_count = 0;
            t_next += LOOP_INTERVAL_US;
            while ((int32_t)(micros() - t_next) < 0) {}  // busy-wait
        }
    }
}
