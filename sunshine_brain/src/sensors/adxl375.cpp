#include "adxl375.h"
#include "config.h"
#include <SPI.h>
#include <Adafruit_ADXL375.h>

// Constructor: (cs, SPIClass*, sensorID)
static SPIClass         fspi(FSPI);
static Adafruit_ADXL375 adxl(PIN_ADXL_CS, &fspi);

bool adxl375_init(void) {
    fspi.begin(PIN_ADXL_SCK, PIN_ADXL_MISO, PIN_ADXL_MOSI, PIN_ADXL_CS);
    if (!adxl.begin()) return false;
    // ODR 1600 Hz (setRange is a no-op on ADXL375 — always ±200g, 49 mg/LSB)
    adxl.setDataRate(ADXL3XX_DATARATE_1600_HZ);
    return true;
}

Adxl375Sample adxl375_read(void) {
    Adxl375Sample s;
    // getXYZ returns raw counts directly (49 mg/LSB)
    s.valid = adxl.getXYZ(s.x, s.y, s.z);
    return s;
}
