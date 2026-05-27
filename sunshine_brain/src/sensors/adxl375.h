#pragma once
#include <stdint.h>
#include <stdbool.h>

struct Adxl375Sample {
    int16_t x, y, z;   // raw counts, 49 mg/LSB
    bool    valid;
};

bool          adxl375_init(void);   // returns false on failure
Adxl375Sample adxl375_read(void);
