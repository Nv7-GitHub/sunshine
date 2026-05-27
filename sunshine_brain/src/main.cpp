#include "bringup.h"
#include "config.h"
#include "sensors/adxl375.h"
#include "sensors/lis3mdl.h"
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

    Serial.println("accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,batt_v");
}

void loop() {
#if BRINGUP_LEVEL == 1
    Adxl375Sample a = adxl375_read();
    MagSample     m = lis3mdl_read();
    float         v = batt_read_v();
    Serial.printf("%d,%d,%d,%d,%d,%d,%.3f\n",
                  a.x, a.y, a.z, m.x, m.y, m.z, v);
    delay(10);
#else
    delay(100);
#endif
}
