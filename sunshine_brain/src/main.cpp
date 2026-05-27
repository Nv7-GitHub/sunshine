#include "bringup.h"
#include "config.h"
#include "sensors/adxl375.h"
#include <Arduino.h>

static bool adxl_ok = false;

void setup() {
    Serial.begin(921600);
    pinMode(PIN_LED, OUTPUT);
    adxl_ok = adxl375_init();
    if (!adxl_ok) {
        Serial.println("ERROR: ADXL375 init failed");
        // Blink pattern 1 (1 fast blink → 1s off → repeat)
        while (true) {
            for (int i = 0; i < 1; i++) {
                digitalWrite(PIN_LED, HIGH); delay(50);
                digitalWrite(PIN_LED, LOW);  delay(50);
            }
            delay(1000);
        }
    }
    // Arduino Serial Plotter header
    Serial.println("accel_x,accel_y,accel_z");
}

void loop() {
#if BRINGUP_LEVEL == 1
    if (adxl_ok) {
        Adxl375Sample s = adxl375_read();
        Serial.printf("%d,%d,%d\n", s.x, s.y, s.z);
    }
    delay(10);  // ~100 Hz for Serial Plotter
#endif
}
