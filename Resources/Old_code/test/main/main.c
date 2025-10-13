#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"



const static char *TAG = "CLYDE_1";

void app_main(void)
{
    gpio_set_direction(GPIO_NUM_20, GPIO_MODE_OUTPUT);
    gpio_pullup_en(GPIO_NUM_20);
    vTaskDelay(10);
    while(1){
        gpio_pulldown_dis(GPIO_NUM_20);
        gpio_pullup_en(GPIO_NUM_20);
        ESP_LOGI(TAG, "GPIO LEVEL %d", 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_pullup_dis(GPIO_NUM_20);
        gpio_pulldown_en(GPIO_NUM_20);
        ESP_LOGI(TAG, "GPIO LEVEL %d", 1);
        vTaskDelay(pdMS_TO_TICKS(2000));



    }

}
