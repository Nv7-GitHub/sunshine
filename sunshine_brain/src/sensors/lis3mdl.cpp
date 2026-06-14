#include "lis3mdl.h"
#include "config.h"
#include <SPI.h>
#include <Adafruit_LIS3MDL.h>
#include <Arduino.h>

static SPIClass        hspi(HSPI);
static Adafruit_LIS3MDL mag;

bool lis3mdl_init(void) {
    hspi.begin(PIN_MAG_SCK, PIN_MAG_MISO, PIN_MAG_MOSI, PIN_MAG_CS);
    // 5 MHz SPI (LIS3MDL max is 10 MHz). Default was 1 MHz → ~90 µs per read in
    // the 1 kHz loop; 5 MHz cuts the transfer time ~5×.
    if (!mag.begin_SPI(PIN_MAG_CS, &hspi, 5000000)) return false;
    // 1000 Hz / low-power mode. The nav loop runs at 1 kHz and reads the mag every
    // tick; at the old 155 Hz the SAME conversion was re-read ~6× while est_theta
    // advanced 20–40° at high spin, smearing the synchronous derotation (|derot|
    // collapsed 25→6 µT above ~500 RPM). 1000 Hz delivers a FRESH sample every
    // tick — which is exactly what the LP4 derot filter (designed for fs=1000 Hz,
    // see sunshine_core.h) assumes. The extra per-sample noise of LP mode is
    // dominated out by that 1 Hz LP filter's heavy averaging.
    //
    // LIS3MDL_DATARATE_1000_HZ internally sets LP mode + FAST_ODR (per Adafruit
    // library: setDataRate calls setPerformanceMode(LP) before writing the ODR
    // bits). Do NOT call setPerformanceMode separately — it would clobber the
    // FAST_ODR bit and leave the register in an undefined state.
    //
    // NOTE (bench check): the Adafruit driver leaves BDU (block-data-update,
    // CTRL_REG5 bit6) at 0. At 1 kHz a burst read that straddles a conversion
    // can occasionally tear one axis. If the spin-up mag trace shows isolated
    // spikes breaking the sine, enable BDU (needs a raw CTRL_REG5|=0x40 write —
    // do it with hardware in the loop so it can be verified live).
    mag.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    mag.setRange(LIS3MDL_RANGE_16_GAUSS);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
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
