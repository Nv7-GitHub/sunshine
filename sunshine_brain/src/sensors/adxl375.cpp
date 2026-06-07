#include "adxl375.h"
#include "config.h"
#include <SPI.h>
#include <Arduino.h>
#include <Adafruit_ADXL375.h>

// Constructor: (cs, SPIClass*, sensorID)
static SPIClass         fspi(FSPI);
static Adafruit_ADXL375 adxl(PIN_ADXL_CS, &fspi);

// ADXL register map (ADXL343 family). Data is 6 bytes from DATAX0, little-endian.
static constexpr uint8_t ADXL_REG_DATAX0 = 0x32;
static constexpr uint8_t ADXL_SPI_READ   = 0x80;  // bit7: read
static constexpr uint8_t ADXL_SPI_MULTI  = 0x40;  // bit6: multi-byte
// The Adafruit driver hardcodes a 1 MHz SPI clock (~100 µs per read in the
// 1 kHz loop). The ADXL375 supports 5 MHz, so the hot-path read below bypasses
// the driver and clocks at 5 MHz in SPI_MODE3. Init/config still go through the
// Adafruit driver (one-time, clock speed irrelevant there).
static const SPISettings ADXL_SPI(5000000, MSBFIRST, SPI_MODE3);

bool adxl375_init(void) {
    fspi.begin(PIN_ADXL_SCK, PIN_ADXL_MISO, PIN_ADXL_MOSI, PIN_ADXL_CS);
    if (!adxl.begin()) return false;
    // ODR 1600 Hz (setRange is a no-op on ADXL375 — always ±200g, 49 mg/LSB)
    adxl.setDataRate(ADXL3XX_DATARATE_1600_HZ);
    return true;
}

Adxl375Sample adxl375_read(void) {
    Adxl375Sample s;
    // Direct 5 MHz burst read of DATAX0..DATAZ1 (6 bytes, raw 49 mg/LSB counts).
    uint8_t rx[6];
    fspi.beginTransaction(ADXL_SPI);
    digitalWrite(PIN_ADXL_CS, LOW);
    fspi.transfer(ADXL_REG_DATAX0 | ADXL_SPI_READ | ADXL_SPI_MULTI);
    for (int i = 0; i < 6; i++) rx[i] = fspi.transfer(0x00);
    digitalWrite(PIN_ADXL_CS, HIGH);
    fspi.endTransaction();
    s.x = (int16_t)((uint16_t)rx[1] << 8 | rx[0]);
    s.y = (int16_t)((uint16_t)rx[3] << 8 | rx[2]);
    s.z = (int16_t)((uint16_t)rx[5] << 8 | rx[4]);
    s.valid = true;
    return s;
}
