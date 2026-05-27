/* src/utils.c */
#include "sunshine_core.h"
#include <string.h>
#include <math.h>

float sunshine_f16_to_f32(uint16_t h) {
    uint32_t sign =  (h >> 15) & 0x1;
    uint32_t exp  =  (h >> 10) & 0x1F;
    uint32_t mant =   h        & 0x3FF;
    uint32_t f;
    if (exp == 0) {
        if (mant == 0) { f = sign << 31; }
        else {
            exp = 1;
            while (!(mant & 0x400)) { mant <<= 1; exp--; }
            mant &= 0x3FF;
            f = (sign << 31) | ((exp + 127 - 15) << 23) | (mant << 13);
        }
    } else if (exp == 31) {
        f = (sign << 31) | 0x7F800000u | (mant << 13);
    } else {
        f = (sign << 31) | ((exp + 127 - 15) << 23) | (mant << 13);
    }
    float r; memcpy(&r, &f, 4); return r;
}

uint16_t sunshine_f32_to_f16(float val) {
    uint32_t x; memcpy(&x, &val, 4);
    uint32_t sign = (x >> 31) & 0x1;
    int32_t  exp  = (int32_t)((x >> 23) & 0xFF) - 127 + 15;
    uint32_t mant = (x & 0x7FFFFFu) >> 13;
    if (exp <= 0)  return (uint16_t)(sign << 15);
    if (exp >= 31) return (uint16_t)((sign << 15) | 0x7C00u);
    return (uint16_t)((sign << 15) | ((uint32_t)exp << 10) | mant);
}

float sunshine_accel_to_ms2(int16_t raw) { return (float)raw * ADXL_SCALE_MS2; }
float sunshine_mag_to_ut   (int16_t raw) { return (float)raw * MAG_SCALE_UT;   }
float sunshine_batt_to_v   (int8_t  off) { return BATT_OFFSET_REF_V + (float)off * BATT_SCALE_V; }
uint32_t sunshine_schema_version(void)   { return SUNSHINE_SCHEMA_VERSION; }
