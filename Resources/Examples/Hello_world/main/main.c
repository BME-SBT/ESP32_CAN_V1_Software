const static char *TAG = "ESP";
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

void hello_world(){
    ESP_LOGI(TAG, "Hello world!");
    vTaskDelay(pdMS_TO_TICKS(500));
}

void app_main(void)
{
    xTaskCreate(
        hello_world,          // Task function
        "HelloWorld",    // Name of the task
        2048,                 // Stack size in words
        NULL,                 // Parameter to pass
        1,                    // Priority
        NULL                  // Task handle
    );

}