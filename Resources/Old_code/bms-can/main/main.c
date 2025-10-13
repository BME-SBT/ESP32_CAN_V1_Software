#define PIN_GPIO_1        GPIO_NUM_8
#define PIN_GPIO_2        GPIO_NUM_9
#define PIN_GPIO_3        GPIO_NUM_15
#define PIN_GPIO_4        GPIO_NUM_16
#define PIN_GPIO_5        GPIO_NUM_17
#define PIN_GPIO_6        GPIO_NUM_18
#define PIN_GPIO_7        GPIO_NUM_19
#define PIN_GPIO_8        GPIO_NUM_20
#define PIN_GPIO_9        GPIO_NUM_21
#define PIN_GPIO_10       GPIO_NUM_22
#define PIN_GPIO_11       GPIO_NUM_23
#define PIN_CAN_RX        GPIO_NUM_10
#define PIN_CAN_TX        GPIO_NUM_11
#define PIN_CLK           GPIO_NUM_6
#define PIN_LATCH_1       GPIO_NUM_7
#define PIN_LATCH_2       GPIO_NUM_1
#define PIN_DATA          GPIO_NUM_0
#define PIN_I_MON         GPIO_NUM_2
const static char *TAG = "BMS";
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "math.h"

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

void app_main(void)
{
    twai_message_t BMS = {
        .identifier = 0x200,
        .data_length_code = 8,
        .data = {0},
        .extd = 0,
        .rtr = 0
    };
    twai_message_t BMS2 = {
        .identifier = 0x201,
        .data_length_code = 6,
        .data = {0},
        .extd = 0,
        .rtr = 0
    };

    
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    while(1){
        int cntr = 0;
        twai_message_t rx_msg;
        rx_msg.identifier = 0x000;

        while(rx_msg.identifier != BMS.identifier && cntr < 11){
            twai_receive(&rx_msg, pdMS_TO_TICKS(200));

            
            cntr ++;

        }
        if(cntr == 11){
            ESP_LOGI(TAG, "No reply received");
        }
        if(rx_msg.identifier == BMS.identifier){
            BMS.data[0] = rx_msg.data[0];
            BMS.data[1] = rx_msg.data[1];
            int16_t current = (((int16_t)BMS.data[0] << 8) | BMS.data[1]);
            BMS.data[2] = rx_msg.data[2];
            BMS.data[3] = rx_msg.data[3];
            uint16_t voltage = (((uint16_t)BMS.data[2] << 8) | BMS.data[3]);
            BMS.data[4] = rx_msg.data[4];
            BMS.data[5] = rx_msg.data[5];
            uint16_t low_voltage = (((uint16_t)BMS.data[4] << 8) | BMS.data[5]);
            BMS.data[6] = rx_msg.data[6];
            BMS.data[7] = rx_msg.data[7];
            uint16_t high_voltage = (((uint16_t)BMS.data[6] << 8) | BMS.data[7]);

            ESP_LOGI(TAG, "Received reply:");
            ESP_LOGI(TAG, "current: %.1f [A]", current / 10.0);
            ESP_LOGI(TAG, "voltage: %.1f [V]", voltage / 10.0);
            ESP_LOGI(TAG, "Low cell: %.3f [V]", low_voltage / 10000.0);
            ESP_LOGI(TAG, "High cell: %.3f [V]", high_voltage / 10000.0);
        }

        rx_msg.identifier = 0x000;
        cntr = 0;

        while(rx_msg.identifier != BMS2.identifier && cntr < 11){
            twai_receive(&rx_msg, pdMS_TO_TICKS(1000));

            
            cntr ++;

        }
        ESP_LOGI(TAG, "cntr %d", cntr);
        if(cntr == 11){
            ESP_LOGI(TAG, "No reply received");
        }
        if(rx_msg.identifier == BMS2.identifier){
            BMS2.data[0] = rx_msg.data[0];
            BMS2.data[1] = rx_msg.data[1];
            BMS2.data[2] = rx_msg.data[2];
            BMS2.data[3] = rx_msg.data[3];
            BMS2.data[4] = rx_msg.data[4];
            BMS2.data[5] = rx_msg.data[5];

            ESP_LOGI(TAG, "Received reply:");
            ESP_LOGI(TAG, "soc: %.1f [Percent]", BMS2.data[2] / 2.0);
            ESP_LOGI(TAG, "Highest temperature: %d [C]", BMS2.data[4]);
            ESP_LOGI(TAG, "Lowest temperature: %d [C]", BMS2.data[5]);
            if(BMS2.data[3] == 1){
                ESP_LOGI(TAG, "Voltage Failsafe Status");
            }
            if(BMS2.data[3] == 2){
                ESP_LOGI(TAG, "Current Failsafe Status");
            }
            if(BMS2.data[3] == 3){
                ESP_LOGI(TAG, "Relay Failsafe Status");
            }
            if(BMS2.data[3] == 8){
                ESP_LOGI(TAG, "Cell Ballancing Active");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));


    }

}
