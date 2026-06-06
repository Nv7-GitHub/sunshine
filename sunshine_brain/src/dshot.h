#pragma once
#include <stdint.h>
#include <stdbool.h>

bool        dshot_init(void);                          // init both ESCs; returns false on error
const char *dshot_last_error(void);                    // human-readable init failure details
void  dshot_send(uint16_t left, uint16_t right); // DShot values 0–2047; 0=disarm
float dshot_erpm_left(void);
float dshot_erpm_right(void);
void  dshot_print_telem_debug(void);                   // print raw getTelemetry result codes
void  dshot_dump_rx_frames(void);                      // dump last RX frame symbols (durations)

// Quantise [0.0, 2047.0] → uint8 for SunshineInput
static inline uint8_t dshot_quantize(float v) {
    int q = (int)(v * (255.0f / 2047.0f) + 0.5f);
    return (uint8_t)(q < 0 ? 0 : q > 255 ? 255 : q);
}
