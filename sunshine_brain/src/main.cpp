#include "bringup.h"
#include "config.h"
#include "sensors/adxl375.h"
#include "sensors/lis3mdl.h"
#if FEATURE_DSHOT
#include "dshot.h"
#endif
#include <Arduino.h>

static bool adxl_ok = false;
static bool mag_ok  = false;

static void blink_error(int n) {
    while (true) {
        for (int i = 0; i < n; i++) {
            digitalWrite(PIN_LED, HIGH); delay(50);
            digitalWrite(PIN_LED, LOW);  delay(50);
        }
        delay(1000);
        Serial.printf("INIT ERROR: sensor fault (blink count %d)\n", n);
    }
}

void setup() {
    Serial.begin(921600);
    pinMode(PIN_LED, OUTPUT);
    analogReadResolution(12);

    adxl_ok = adxl375_init();
    mag_ok  = lis3mdl_init();

    int err_code = (!adxl_ok ? 1 : 0) + (!mag_ok ? 2 : 0);
    if (err_code) blink_error(err_code);

#if FEATURE_DSHOT
    if (!dshot_init()) blink_error(3);
    Serial.println("DShot ready. Commands: l <0-2047>, r <0-2047>, s (stop), t (print eRPM)");
#endif

#if BRINGUP_LEVEL == 1
    Serial.println("accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,batt_v");
#endif
}

void loop() {
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
    static uint16_t cmd_left  = 0;   // start at 0 (disarm)
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
                Serial.printf("Left → %d\n", cmd_left);
            } else if (cmd[0] == 'r') {
                v = atoi(cmd + 2);
                v = v < 0 ? 0 : v > 2047 ? 2047 : v;
                cmd_right = (uint16_t)v;
                Serial.printf("Right → %d\n", cmd_right);
            } else if (cmd[0] == 's') {
                cmd_left = cmd_right = 0;
                Serial.println("Stop");
            } else if (cmd[0] == 't') {
                Serial.printf("eRPM L=%.0f R=%.0f\n",
                              dshot_erpm_left(), dshot_erpm_right());
            }
        } else if (ci < 31) {
            cmd[ci++] = c;
        }
    }
    dshot_send(cmd_left, cmd_right);   // keepalive with stored values
    delay(1);

#else
    delay(100);
#endif
}
