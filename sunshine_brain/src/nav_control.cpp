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

        // Sensors  (per-section µs timing — see PROF print at end of loop)
        uint32_t t_a0 = micros();
        Adxl375Sample a = adxl375_read();
        uint32_t t_a1 = micros();
        MagSample     m = lis3mdl_read();
        uint32_t t_a2 = micros();
        float         v = batt_read_v();
        uint32_t t_a3 = micros();

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
        uint32_t t_c0 = micros();
        CtrlInputs ctrl = telemetry_get_ctrl();
        uint32_t t_c1 = micros();

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
        // Snapshot the filter state BEFORE stepping. Telemetry must carry the
        // "state at the start" of each frame (README → Replay): the host seeds
        // replay from frame.state and then re-runs the inputs. Pairing an input
        // with its POST-step state would seed replay one sample ahead of the
        // inputs, so replayed θ/ω would not match the real trajectory.
        SunshineState pre_step_state = kf_state;
        uint32_t t_s0 = micros();
        sunshine_step(&in, &kf_state, &vars);
        uint32_t t_s1 = micros();

        // Safety: zero DShot at bringup level 3 (telemetry only, no motion).
#if FORCE_DSHOT_ZERO
        vars.dshot_cmd_left  = 0.0f;
        vars.dshot_cmd_right = 0.0f;
#endif
        // Level 4 (nav tuning): only TANK drives — used to spin the robot for
        // filter tuning. DISABLED and MELTY stay zeroed.
#if TANK_ONLY_OUTPUT
        if (in.mode != SUNSHINE_MODE_TANK) {
            vars.dshot_cmd_left  = 0.0f;
            vars.dshot_cmd_right = 0.0f;
        }
#endif

        // ── 5. Apply outputs ──────────────────────────────────────────────
        uint32_t dshot_us = 0;
#if FEATURE_DSHOT
        auto motor_cmd = [](float cmd, bool invert) -> uint16_t {
            if (cmd == 0.0f) return 0;  // preserve disarm
            float c = invert ? (2.0f * DSHOT_NEUTRAL - cmd) : cmd;
            return (uint16_t)(c + 0.5f);
        };
        uint32_t t_d0 = micros();
        dshot_send(motor_cmd(vars.dshot_cmd_left,  MOTOR_LEFT_INVERT),
                   motor_cmd(vars.dshot_cmd_right, MOTOR_RIGHT_INVERT));
        dshot_us = micros() - t_d0;
#endif
        // Disabled: slow breathe so the board is visibly alive but clearly idle.
        // Active: binary heading flash from sunshine_step.
        if (in.mode == SUNSHINE_MODE_DISABLED) {
            uint32_t phase = (uint32_t)millis() % 2000;          // 2-s period
            float t = phase < 1000 ? phase * 0.001f              // 0→1 ramp
                                   : (2000 - phase) * 0.001f;    // 1→0 ramp
            analogWrite(PIN_LED, (uint8_t)(t * t * LED_DUTY));   // squared for a softer fade
        } else {
            analogWrite(PIN_LED, vars.led_on ? LED_DUTY : 0);
        }

        // ── 6. Push to telemetry ring buffer ──────────────────────────────
#if FEATURE_TELEMETRY
        bool pushed = telemetry_push(&in, &pre_step_state);
        if (!pushed) vars.loop_overrun = true;  // set if current tick overran OR telem ring full
#endif

        // ── 7. Timing: busy-wait until next 1 kHz tick ───────────────────
        uint32_t elapsed = micros() - t_start;

        // ── PROF (diagnostic): per-section + total + worst-case-over-window.
        // Printed inside the idle budget (before the busy-wait) so it doesn't
        // itself cause an overrun. `rest` = everything not individually timed
        // (analogWrite, telemetry_push, overhead, ISR/preemption jitter).
        // Remove once the 1 kHz budget is comfortably met.
        {
            uint32_t adxl_us = t_a1 - t_a0, mag_us = t_a2 - t_a1, batt_us = t_a3 - t_a2;
            uint32_t ctrl_us = t_c1 - t_c0, step_us = t_s1 - t_s0;
            uint32_t measured = adxl_us + mag_us + batt_us + ctrl_us + step_us + dshot_us;
            uint32_t rest_us  = elapsed > measured ? elapsed - measured : 0;
            static uint32_t mx_total = 0, prof_n = 0;
            if (elapsed > mx_total) mx_total = elapsed;
            if (++prof_n % 500 == 0) {
                Serial.printf("PROF us: adxl=%u mag=%u batt=%u ctrl=%u step=%u dshot=%u rest=%u | total=%u max=%u\n",
                              adxl_us, mag_us, batt_us, ctrl_us, step_us, dshot_us, rest_us, elapsed, mx_total);
                mx_total = 0;
            }
        }

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
