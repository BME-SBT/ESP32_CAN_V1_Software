// Simple CAN bus send and receive data code with Mutex


// start of pin deffinitions
#define PIN_GPIO_0 GPIO_NUM_1
#define PIN_GPIO_1 GPIO_NUM_0
#define PIN_GPIO_2 GPIO_NUM_7
#define PIN_GPIO_3 GPIO_NUM_6
#define PIN_GPIO_4 GPIO_NUM_5
#define PIN_GPIO_5 GPIO_NUM_4
#define PIN_GPIO_6 GPIO_NUM_16
#define PIN_GPIO_7 GPIO_NUM_9
#define PIN_GPIO_8 GPIO_NUM_18
#define PIN_GPIO_9 GPIO_NUM_19
#define PIN_GPIO_10 GPIO_NUM_20
#define PIN_GPIO_11 GPIO_NUM_21
#define PIN_GPIO_12 GPIO_NUM_22
#define PIN_GPIO_13 GPIO_NUM_23
#define PIN_GPIO_14 GPIO_NUM_15
#define PIN_GPIO_15 GPIO_NUM_17
#define PIN_CAN_RX GPIO_NUM_3
#define PIN_CAN_TX GPIO_NUM_2
#define PIN_CLK GPIO_NUM_8
#define PIN_LATCH GPIO_NUM_10
#define PIN_DATA GPIO_NUM_11
// end of pin deffinitions

const static char* TAG = "<BoardName>";  //<BoardName> is the name of the board used in logging

#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/twai_periph.h"  // For GPIO matrix signal index

// used instead of gpio_set_direction, this on actually works
void gpio_set_dir(int gpio, int mode) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    if (mode == 2) {
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    if (mode == 1) {
        io_conf.mode = GPIO_MODE_INPUT;
    }

    gpio_config(&io_conf);
}

/* --------------------- Definitions and static variables for TWAI ------------------ */
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);

// Message IDs for the two transmittable messages
#define TX_MSG1_ID 0x100
#define TX_MSG2_ID 0x101

// Shared transmittable variables (could be updated by ADC or other tasks)
static uint8_t tx_var1 = 0;
static uint8_t tx_var2 = 0;

// Mutex to protect access to shared transmit variables
static SemaphoreHandle_t mutex = NULL;

static volatile bool tx_started = false;

static void tx_task(void* arg) {
    // Prepare message containers
    twai_message_t msg1 = {
        .extd = 0,
        .rtr = 0,
        .identifier = TX_MSG1_ID,
        .data_length_code = 8,  // Always use 8 for data length on all devices
        .data = {0}
    };
    twai_message_t msg2 = {
        .extd = 0,
        .rtr = 0,
        .identifier = TX_MSG2_ID,
        .data_length_code = 8,
        .data = {0}
    };

    // Wait until controller starts transmissions
    while (!tx_started) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    while (1) {
        // Every 100ms, send latest values of tx_var1 and tx_var2
        xSemaphoreTake(mutex, portMAX_DELAY);
        msg1.data[0] = tx_var1;
        msg2.data[0] = tx_var2;
        xSemaphoreGive(mutex);

        esp_err_t r1 = twai_transmit(&msg1, pdMS_TO_TICKS(10));
        if (r1 != ESP_OK) {
            ESP_LOGW(TAG, "Failed to transmit msg1: %s", esp_err_to_name(r1));
        }
        esp_err_t r2 = twai_transmit(&msg2, pdMS_TO_TICKS(10));
        if (r2 != ESP_OK) {
            ESP_LOGW(TAG, "Failed to transmit msg2: %s", esp_err_to_name(r2));
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
            // take mutex of tx_variable and modify it with data received
            switch (rx_msg.identifier) {
                case TX_MSG1_ID:
                    xSemaphoreTake(mutex, portMAX_DELAY);
                    tx_var1 = rx_msg.data[0];
                    xSemaphoreGive(mutex);
                    break;
                case TX_MSG2_ID:
                    xSemaphoreTake(mutex, portMAX_DELAY);
                    tx_var2 = rx_msg.data[0];
                    xSemaphoreGive(mutex);
                    break;
                default:
                    // Unhandled IDs can be ignored or logged
                    break;
            }
        } else {
            ESP_LOGW(TAG, "Failed to receive message: %s", esp_err_to_name(ret));
        }
    }

    vTaskDelete(NULL);
}

void app_main(void) {
    // Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");
    ESP_LOGI(TAG, "Starting transmissions");

    // Create mutex for protecting shared transmit variables
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Initialize transmit variables (can be updated by other tasks, e.g. ADC)
    xSemaphoreTake(mutex, portMAX_DELAY);
    tx_var1 = 69; // seed with random values for this example
    tx_var2 = 70;
    xSemaphoreGive(mutex);

    // Signal tx_task to start
    tx_started = true;
    ESP_LOGI(TAG, "CAN Driver installed");


    xTaskCreatePinnedToCore(
        tx_task,      // Task function
        "TWAI_tx",     // Task name
        4096,          // Stack size
        NULL,          // Task input parameter
        1,             // Task priority
        NULL,          // Task handle
        tskNO_AFFINITY);
    xTaskCreatePinnedToCore(
        rx_task, 
        "TWAI_rx", 
        4096,          // Stack size
        NULL,          // Task input parameter
        1,             // Task priority
        NULL,          // Task handle
        tskNO_AFFINITY);

    

    // Let rx_task handle receives
    vTaskDelay(pdMS_TO_TICKS(100));
}
