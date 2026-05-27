#include "lis3mdl.h"
#include "config.h"
#include <SPI.h>
#include <Adafruit_LIS3MDL.h>
#include <Arduino.h>

static SPIClass        hspi(HSPI);
static Adafruit_LIS3MDL mag;

bool lis3mdl_init(void) {
    hspi.begin(PIN_MAG_SCK, PIN_MAG_MISO, PIN_MAG_MOSI, PIN_MAG_CS);
    if (!mag.begin_SPI(PIN_MAG_CS, &hspi)) return false;
    mag.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    mag.setRange(LIS3MDL_RANGE_16_GAUSS);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    return true;
}

MagSample lis3mdl_read(void) {
    MagSample s;
    mag.read();
    s.x     = mag.x;
    s.y     = mag.y;
    s.z     = mag.z;
    s.valid = true;
    return s;
}

float batt_read_v(void) {
    int raw = analogRead(PIN_BATT_ADC);
    return (float)raw * BATT_ADC_SCALE;
}
