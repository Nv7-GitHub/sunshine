// src/main.cpp
// Receiver firmware: bridges brain (ESP-NOW) ↔ host app (USB serial)
//
// Core 0: FreeRTOS task — ESP-NOW RX callback (registered via esp_now, runs on
//         Core 0 by default in Arduino framework)
// Core 1: bridge_task — USB TX/RX + 500 Hz timer (timer runs on its own task
//         but we drive the USB I/O from this loop)

#include "config.h"
#include "protocol.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ── Bridge task (Core 1) ──────────────────────────────────────────────────────
static void bridge_task(void *) {
    for (;;) {
        usb_bridge_tick();
        vTaskDelay(pdMS_TO_TICKS(1));  // yield ~1 ms between ticks (USB is buffered)
    }
}

void setup() {
    Serial.begin(921600);
    delay(100);

    // ── WiFi: STA mode, fixed channel 1, max power ────────────────────────────
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(84); // 21 dBm, maximum

    // ── ESP-NOW init ──────────────────────────────────────────────────────────
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        // Blink fast indefinitely to signal error
        while (true) {
            delay(100);
        }
    }
    esp_now_set_pmk((uint8_t *)"pmk_not_used_000"); // required call; PMK unused

    // ── Initialise subsystems ─────────────────────────────────────────────────
    espnow_rx_init();
    espnow_rx_register_callback();
    usb_bridge_init();

    // ── Start bridge task pinned to Core 1 ───────────────────────────────────
    xTaskCreatePinnedToCore(bridge_task, "bridge", 8192, nullptr, 5, nullptr, 1);

    Serial.println("Receiver ready");
}

void loop() {
    // All work is in bridge_task and the esp_timer callback.
    // loop() runs on Core 1 too but we keep it empty.
    vTaskDelay(portMAX_DELAY);
}
