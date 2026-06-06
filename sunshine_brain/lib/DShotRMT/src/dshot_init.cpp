/**
 * @file dshot_init.cpp
 * @brief RMT configuration and initialization functions for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-10-04
 * @license MIT
 */

#include "dshot_init.h"
#include <Arduino.h>

// Function to initialize the RMT TX channel
dshot_result_t init_rmt_tx_channel(gpio_num_t gpio, rmt_channel_handle_t *out_channel, bool is_bidirectional)
{
    rmt_tx_channel_config_t tx_channel_config = {
        .gpio_num = gpio,
        .clk_src = DSHOT_CLOCK_SRC_DEFAULT,
        .resolution_hz = DSHOT_RMT_RESOLUTION,
        .mem_block_symbols = RMT_TX_BUFFER_SYMBOLS,
        .trans_queue_depth = RMT_QUEUE_DEPTH,
        .flags = {
            .invert_out = is_bidirectional ? 1 : 0,
            // Open-drain is required for BDSHOT: after TX the pin must release so
            // the ESC can pull the line to 0V for its response. Push-pull keeps
            // driving HIGH after TX, so the ESC can only reach ~1.6V (voltage
            // divider), which never crosses the RMT RX threshold.
            .io_od_mode = is_bidirectional ? 1 : 0,
            .init_level = 0}};

    rmt_transmit_config_t rmt_tx_config = {}; // Initialize all members to zero
    rmt_tx_config.loop_count = 0;             // No automatic loops - real-time calculation

    if (rmt_new_tx_channel(&tx_channel_config, out_channel) != DSHOT_OK)
    {
        return {false, DSHOT_TX_INIT_FAILED};
    }

    if (rmt_enable(*out_channel) != DSHOT_OK)
    {
        return {false, DSHOT_TX_INIT_FAILED};
    }

    if (is_bidirectional)
    {
        // Internal pull-up (~45 kΩ) holds the line HIGH when neither MCU nor ESC
        // is driving. With open-drain TX this is the only thing pulling the line
        // up between pulses — without it the line floats and both sides see noise.
        gpio_pullup_en(gpio);
    }

    return {true, DSHOT_TX_INIT_SUCCESS};
}

// Function to initialize the RMT RX channel
dshot_result_t init_rmt_rx_channel(gpio_num_t gpio, rmt_channel_handle_t *out_channel, rmt_rx_event_callbacks_t *rx_event_callbacks, void *user_data)
{
    // Use internal RMT memory (no DMA). ESP32-S3 has only one DMA-capable RMT RX
    // channel; the original with_dma=1 config therefore fails on the second motor.
    // mem_block_symbols must be >= SOC_RMT_MEM_WORDS_PER_CHANNEL (48 on ESP32-S3);
    // 48 is enough for the 21-symbol eRPM GCR response. Full 10-byte ESC telemetry
    // (110 symbols) is not available with this config.
    rmt_rx_channel_config_t rx_channel_config = {
        .gpio_num = gpio,
        .clk_src = DSHOT_CLOCK_SRC_DEFAULT,
        .resolution_hz = DSHOT_RMT_RESOLUTION,
        .mem_block_symbols = 48,
    };

    esp_err_t rx_err = rmt_new_rx_channel(&rx_channel_config, out_channel);
    if (rx_err != DSHOT_OK)
    {
        Serial.printf("rmt_new_rx_channel gpio=%d err=0x%x\n", (int)gpio, rx_err);
        return {false, DSHOT_RX_INIT_FAILED};
    }

    if (rmt_rx_register_event_callbacks(*out_channel, rx_event_callbacks, user_data) != DSHOT_OK)
    {
        return {false, DSHOT_CALLBACK_REGISTERING_FAILED};
    }

    if (rmt_enable(*out_channel) != DSHOT_OK)
    {
        return {false, DSHOT_RX_INIT_FAILED};
    }

    return {true, DSHOT_RX_INIT_SUCCESS};
}

// Function to initialize the DShot RMT encoder
dshot_result_t init_dshot_encoder(rmt_encoder_handle_t *out_encoder, const rmt_ticks_t &rmt_ticks, uint16_t pulse_level, uint16_t idle_level)
{
    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {
            .duration0 = rmt_ticks.t0h_ticks,
            .level0 = pulse_level,
            .duration1 = rmt_ticks.t0l_ticks,
            .level1 = idle_level,
        },
        .bit1 = {
            .duration0 = rmt_ticks.t1h_ticks,
            .level0 = pulse_level,
            .duration1 = rmt_ticks.t1l_ticks,
            .level1 = idle_level,
        },

        .flags = {
            .msb_first = 1 // DShot is MSB first
        }};

    if (rmt_new_bytes_encoder(&encoder_config, out_encoder) != DSHOT_OK)
    {
        return {false, DSHOT_ENCODER_INIT_FAILED};
    }

    return {true, DSHOT_ENCODER_INIT_SUCCESS};
}