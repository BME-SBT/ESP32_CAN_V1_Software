// Simple CAN bus send and receive data code with Mutex

const static char* TAG = "<BoardName>";  //<BoardName> is the name of the board used in logging

#include <solar.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
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

// Mutexes to protect each frame (named m_<frameName>)
static SemaphoreHandle_t m_example_frame1 = NULL;
static SemaphoreHandle_t m_example_frame2 = NULL;

// CAN error flag and counter protected by mutex
static volatile bool can_error = false;
static volatile uint8_t can_error_counter = 0;
static SemaphoreHandle_t m_can_error = NULL;

// Simple helpers for manipulating the CAN error flag/counter (protected by m_can_error)
static void can_error_set(void) {
    if (m_can_error) xSemaphoreTake(m_can_error, portMAX_DELAY);
    if (can_error_counter < 10) can_error_counter++;
    can_error = true;
    if (m_can_error) xSemaphoreGive(m_can_error);
}

static void can_error_clear_one(void) {
    if (m_can_error) xSemaphoreTake(m_can_error, portMAX_DELAY);
    if (can_error_counter > 0) can_error_counter--;
    if (can_error_counter <= 0) can_error = false;
    if (m_can_error) xSemaphoreGive(m_can_error);
}

static bool can_error_get(void) {
    bool v;
    if (m_can_error) xSemaphoreTake(m_can_error, portMAX_DELAY);
    v = can_error;
    if (m_can_error) xSemaphoreGive(m_can_error);
    return v;
}

/* ------------------------------- TX and RX task ---------------------------------- */
// Quick note on mutex: If you ever need to lock both mutexes in the same function, always
// take them in a fixed order (e.g., lock m_example_frame1 then m_example_frame2) to avoid deadlocks.

static void tx_task(void* arg) {
    // Local copies used for transmission (to avoid holding mutex while transmitting)
    twai_message_t local1;
    twai_message_t local2;

    while (1) {
        // Every 100ms, send latest values of example_frame1 and example_frame2
        // Copy frame1 under its mutex
        xSemaphoreTake(m_example_frame1, portMAX_DELAY);
        local1 = example_frame1;  // struct copy (copies data[] too)
        xSemaphoreGive(m_example_frame1);

        // Copy frame2 under its mutex
        xSemaphoreTake(m_example_frame2, portMAX_DELAY);
        local2 = example_frame2;
        xSemaphoreGive(m_example_frame2);

        // Ensure data_length_code is set correctly before transmission (in case it was modified by received data)
        local1.data_length_code = 8;
        local2.data_length_code = 8;

        esp_err_t r1 = twai_transmit(&local1, pdMS_TO_TICKS(10));
        if (r1 != ESP_OK) {
            ESP_LOGW(TAG, "Failed to transmit example_frame1: %s", esp_err_to_name(r1));
            can_error_set();
        } else {
            can_error_clear_one();
        }
        esp_err_t r2 = twai_transmit(&local2, pdMS_TO_TICKS(10));
        if (r2 != ESP_OK) {
            ESP_LOGW(TAG, "Failed to transmit example_frame2: %s", esp_err_to_name(r2));
            can_error_set();
        } else {
            can_error_clear_one();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}

static void rx_task(void* arg) {
    while (1) {
        twai_message_t rx_msg;
        esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Received message with ID: 0x%03X", rx_msg.identifier);
            ESP_LOGI(TAG, "Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                     rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3],
                     rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);
            // take mutex of the matching frame and update it with the received frame
            switch (rx_msg.identifier) {
                case example_frame1_ID:
                    xSemaphoreTake(m_example_frame1, portMAX_DELAY);
                    example_frame1 = rx_msg;  // struct copy
                    xSemaphoreGive(m_example_frame1);
                    break;
                case example_frame2_ID:
                    xSemaphoreTake(m_example_frame2, portMAX_DELAY);
                    example_frame2 = rx_msg;  // struct copy
                    xSemaphoreGive(m_example_frame2);
                    break;
                default:
                    // Unhandled IDs can be ignored or logged
                    // ESP_LOGW(TAG, "Received message with unhandled ID: 0x%03X", rx_msg.identifier);
                    break;
            }
            can_error_clear_one();
        } else {
            ESP_LOGW(TAG, "Failed to receive message: %s", esp_err_to_name(ret));
            can_error_set();
        }
    }

    vTaskDelete(NULL);
}

void app_main(void) {
    // Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));

    esp_err_t res = twai_start();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(res));
        return;
    }
    ESP_LOGI(TAG, "Driver started");
    ESP_LOGI(TAG, "Starting transmissions");

    // Create mutexes for each frame
    m_example_frame1 = xSemaphoreCreateMutex();
    if (m_example_frame1 == NULL) {
        ESP_LOGE(TAG, "Failed to create m_example_frame1 mutex");
        return;
    }
    m_example_frame2 = xSemaphoreCreateMutex();
    if (m_example_frame2 == NULL) {
        ESP_LOGE(TAG, "Failed to create m_example_frame2 mutex");
        return;
    }
    // Create mutex for CAN error flag
    m_can_error = xSemaphoreCreateMutex();
    if (m_can_error == NULL) {
        ESP_LOGE(TAG, "Failed to create m_can_error mutex");
        return;
    }
    ESP_LOGI(TAG, "CAN Driver installed");

    xTaskCreate(
        tx_task,    // Task function
        "TWAI_tx",  // Task name
        4096,       // Stack size
        NULL,       // Task input parameter
        1,          // Task priority
        NULL);      // Task handle

    xTaskCreate(
        rx_task,
        "TWAI_rx",
        4096,   // Stack size
        NULL,   // Task input parameter
        1,      // Task priority
        NULL);  // Task handle

    // Let rx_task handle receives
    vTaskDelay(pdMS_TO_TICKS(100));

    // This following part is an example that randomizes example_frame1 data periodically under mutex
    // change all example_frame1 to example_frame2 for a second device and see if they both report
    // random numbers while connected over CAN bus
    srand(12345678);  // Seed the random number generator
    while (1) {
        // Fill example_frame1 data with random bytes under mutex, as a test
        xSemaphoreTake(m_example_frame1, portMAX_DELAY);
        for (int i = 0; i < example_frame1.data_length_code; ++i) {
            example_frame1.data[i] = (uint8_t)(rand() & 0xFF);
        }
        xSemaphoreGive(m_example_frame1);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
