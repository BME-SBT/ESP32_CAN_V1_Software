// Simple CAN app using the centralized CAN manager for TX/RX and per-frame locking

const static char* TAG = "<BoardName>";  //<BoardName> is the name of the board used in logging

#include "solar.h"
#include "can_manager.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* --------------------- Definitions and static variables for TWAI ------------------ */
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);

// Message IDs for the two transmittable messages
#define example_frame1_ID 0x100
#define example_frame2_ID 0x101

// Shared transmittable/receivable frames (fully defined with zeroed data)
static twai_message_t example_frame1 = {
    .extd = 0,
    .rtr = 0,
    .identifier = example_frame1_ID,
    .data_length_code = 8,
    .data = {0}};
static twai_message_t example_frame2 = {
    .extd = 0,
    .rtr = 0,
    .identifier = example_frame2_ID,
    .data_length_code = 8,
    .data = {0}};


/* For more example frame definitions (MotorTemps, BMS, GPS, etc.)
   see README.md. Keep frame definitions minimal here and register
   only what you need for this app. */


/* ------------------------------- App entry ---------------------------------- */

void app_main(void) {
    // Optionally set capacity (no -D flags needed). Skip if you want auto-grow.

    // Register frames with the manager BEFORE init (required; dynamic registration while running is not supported)
    ESP_ERROR_CHECK(can_manager_register_frame(&example_frame1, true, true));
    ESP_ERROR_CHECK(can_manager_register_frame(&example_frame2, true, true));
    // Start the CAN manager (fixed 100 ms period for all TX-enabled frames)
    ESP_ERROR_CHECK(can_manager_init(&g_config, &t_config, &f_config, 100, TAG));
    ESP_LOGI(TAG, "CAN manager started; periodic TX enabled for example frames");

    // This following part is an example that randomizes example_frame1 data periodically under mutex
    // change all example_frame1 to example_frame2 for a second device and see if they both report
    // random numbers while connected over CAN bus
    srand(69);  // Seed the random number generator
    while (1) {
        // Fill example_frame1 data with random bytes under manager lock, as a test
        can_manager_lock_frame(&example_frame1);        // always use lock/unlock everytime you modify/access a frame
        for (int i = 0; i < example_frame1.data_length_code; ++i) {
            example_frame1.data[i] = (uint8_t)(rand() & 0xFF);
        }
        can_manager_unlock_frame(&example_frame1);

        // Example: fetch and print current CAN error state
        bool err = can_manager_error_get();     // retrieve error state
        ESP_LOGI(TAG, "CAN error state: %s", err ? "true" : "false");
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}