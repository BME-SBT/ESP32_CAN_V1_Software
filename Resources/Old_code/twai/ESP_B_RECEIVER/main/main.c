#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#define TX_GPIO_NUM 11
#define RX_GPIO_NUM 10

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

#define TAG "ESP_B_RESPONDER"
#define ID_SEND 0x100
#define ID_REPLY 0x101

void app_main(void) {
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    while (1) {
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK && rx_msg.identifier == ID_SEND) {
            ESP_LOGI(TAG, "Received data: %d", rx_msg.data[0]);

            // Respond
            twai_message_t reply = {
                .identifier = ID_REPLY,
                .data_length_code = 1,
                .data = {rx_msg.data[0] + 1},
                .extd = 0,
                .rtr = 0
            };
            ESP_ERROR_CHECK(twai_transmit(&reply, pdMS_TO_TICKS(100)));
            ESP_LOGI(TAG, "Replied with: %d", reply.data[0]);
        }
    }
}
