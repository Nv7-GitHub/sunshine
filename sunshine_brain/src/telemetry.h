#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <sunshine_core.h>

// Control inputs received from receiver (updated by Core 0 ESP-NOW callback)
struct CtrlInputs {
    uint8_t  mode;
    int8_t   ctrl_x, ctrl_y, ctrl_theta;
    uint8_t  ctrl_throttle;
    int8_t   rssi;
    uint32_t last_rx_ms;   // millis() at last reception
};

void       telemetry_init(void);

// Core 0 task entry point — pass to xTaskCreatePinnedToCore
void       telemetry_task(void *arg);

// Called from Core 1 to get latest control inputs (thread-safe, uses mutex)
CtrlInputs telemetry_get_ctrl(void);

// Called from Core 1 to push a (SunshineInput, SunshineState) for TX
// Returns false if ring buffer is full (item dropped)
bool       telemetry_push(const SunshineInput *in, const SunshineState *state);
