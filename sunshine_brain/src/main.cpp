// src/main.cpp
// Sunshine Brain — main entry point
// Initialises all subsystems in order, handles init failures with LED patterns.
// Core 0: telemetry_task (FreeRTOS, priority 5)
// Core 1: nav_control_task (FreeRTOS, priority 10, pinned)

#include "bringup.h"
#include "config.h"
#include "sensors/adxl375.h"
#include "sensors/lis3mdl.h"
#include "dshot.h"
#include "telemetry.h"
#include "nav_control.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ── Error LED patterns ────────────────────────────────────────────────────────
// N fast blinks (50ms on/off) → 1s off → repeat
// Also prints error continuously to USB serial.
static void error_halt(int blink_count, const char *msg) {
    Serial.printf("FATAL INIT ERROR: %s (blink=%d)\n", msg, blink_count);
    while (true) {
        for (int i = 0; i < blink_count; i++) {
            analogWrite(PIN_LED, LED_DUTY_IDLE); delay(50);
            analogWrite(PIN_LED, 0);             delay(50);
        }
        delay(1000);
        Serial.printf("ERROR: %s\n", msg);  // repeat for late USB attach
    }
}

void setup() {
    Serial.begin(921600);
    pinMode(PIN_LED, OUTPUT);
    analogReadResolution(12);

    // ── Level 1+ : init sensors ───────────────────────────────────────────────
    if (!adxl375_init()) error_halt(1, "ADXL375 init failed");
    if (!lis3mdl_init())  error_halt(2, "LIS3MDL init failed");

#if BRINGUP_LEVEL == 1
    // Bringup 1: CSV output only
    Serial.println("BRINGUP 1: accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,batt_v");
    return;  // don't start tasks
#endif

    // ── Level 2+ : init DShot ─────────────────────────────────────────────────
#if FEATURE_DSHOT
    if (!dshot_init()) error_halt(3, dshot_last_error());
#endif

#if BRINGUP_LEVEL == 2
    // Bringup 2: interactive DShot test only
    Serial.println("BRINGUP 2: DShot test. l <val>, r <val>, s, t, d  (wait ~0.5s for ESC to arm)");
    return;
#endif

    // ── Level 3+ : init telemetry and start tasks ─────────────────────────────
#if FEATURE_TELEMETRY
    telemetry_init();
    if (xTaskCreatePinnedToCore(telemetry_task, "telemetry", 8192, nullptr, 5, nullptr, 0) != pdPASS) {
        error_halt(4, "telemetry task create failed");
    }
#endif

    nav_control_init();
    if (xTaskCreatePinnedToCore(nav_control_task, "nav_ctrl", 8192, nullptr, 10, nullptr, 1) != pdPASS) {
        error_halt(5, "nav_ctrl task create failed");
    }

    Serial.printf("Brain ready (bringup=%d)\n", BRINGUP_LEVEL);
}

void loop() {
    // Bringup 1 and 2 use loop(); production uses FreeRTOS tasks.
#if BRINGUP_LEVEL == 1
    Adxl375Sample a = adxl375_read();
    MagSample     m = lis3mdl_read();
    float         v = batt_read_v();
    Serial.printf("%d,%d,%d,%d,%d,%d,%.3f\n",
                  a.x, a.y, a.z, m.x, m.y, m.z, v);
    delay(10);

#elif BRINGUP_LEVEL == 2
    static char     cmd[32];
    static int      ci = 0;
    static uint16_t cmd_left  = 0;
    static uint16_t cmd_right = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmd[ci] = '\0'; ci = 0;
            int v;
            if (cmd[0] == 'l') {
                v = atoi(cmd + 2);
                v = v < 0 ? 0 : v > 2047 ? 2047 : v;
                cmd_left = (uint16_t)v;
                Serial.printf("L→%d\n", v);
            } else if (cmd[0] == 'r') {
                v = atoi(cmd + 2);
                v = v < 0 ? 0 : v > 2047 ? 2047 : v;
                cmd_right = (uint16_t)v;
                Serial.printf("R→%d\n", v);
            } else if (cmd[0] == 's') {
                cmd_left = cmd_right = 0;
                Serial.println("Stop");
            } else if (cmd[0] == 't') {
                Serial.printf("eRPM L=%.0f R=%.0f\n",
                              dshot_erpm_left(), dshot_erpm_right());
            } else if (cmd[0] == 'd') {
                dshot_print_telem_debug();
            } else if (cmd[0] == 'v') {
                dshot_dump_rx_frames();
            }
        } else if (ci < 31) { cmd[ci++] = c; }
    }
    dshot_send(cmd_left, cmd_right);
    delay(1);

#else
    // Levels 3-4 and production: FreeRTOS tasks do all work
    vTaskDelay(portMAX_DELAY);
#endif
}
