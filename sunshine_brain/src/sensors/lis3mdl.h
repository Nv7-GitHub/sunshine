#pragma once
#include <stdint.h>
#include <stdbool.h>

struct MagSample {
    int16_t x, y, z;   // raw counts, 0.058 µT/LSB at ±16 Gauss
    bool    valid;
};

bool      lis3mdl_init(void);
MagSample lis3mdl_read(void);
float     batt_read_v(void);    // read battery voltage (V)
