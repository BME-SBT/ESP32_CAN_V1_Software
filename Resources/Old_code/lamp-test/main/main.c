#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include <stdint.h>


// shift‑reg
#define PIN_SR_CLK    GPIO_NUM_4
#define PIN_SR_DATA   GPIO_NUM_7
#define PIN_SR_LATCH1 GPIO_NUM_5
#define PIN_SR_LATCH2 GPIO_NUM_6


static void shiftOut16(uint16_t combined_value, bool latch1, bool latch2) {
  
    for (int i = 15; i >= 0; --i) {
        gpio_set_level(PIN_SR_DATA, (combined_value >> i) & 1);
        gpio_set_level(PIN_SR_CLK, 1);
        gpio_set_level(PIN_SR_CLK, 0);
    }
    if (latch1) {
        gpio_set_level(PIN_SR_LATCH1, 1);
        gpio_set_level(PIN_SR_LATCH1, 0);
    }
    if (latch2) {
        gpio_set_level(PIN_SR_LATCH2, 1);
        gpio_set_level(PIN_SR_LATCH2, 0);
    }
}
void lamp(uint8_t red, uint8_t yellow, uint8_t green) {
    uint16_t data = 0;

    // Red lamp
    if (red == 1) data |= (1 << 6);         // Red ON
    if (red == 2) data |= (1 << 5);         // Red BLINKING

    // Yellow lamp
    if (yellow == 1) data |= (1 << 4);      // Yellow ON
    if (yellow == 2) data |= (1 << 3);      // Yellow BLINKING

    // Green lamp
    if (green == 1) data |= (1 << 2);       // Green ON
    if (green == 2) data |= (1 << 1);       // Green BLINKING

    // Shift out the 16-bit data
    shiftOut16(data, /*latch1=*/true, /*latch2=*/true);;
}

const static char *TAG = "PINKY_2";
void setup_gpio() {

    gpio_reset_pin(PIN_SR_CLK);
    gpio_set_direction(PIN_SR_CLK, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_SR_DATA);
    gpio_set_direction(PIN_SR_DATA, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_SR_LATCH1);
    gpio_set_direction(PIN_SR_LATCH1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(PIN_SR_LATCH2);
    gpio_set_direction(PIN_SR_LATCH2, GPIO_MODE_OUTPUT);
}


void app_main(void) {
    setup_gpio();
    while(1){
        ESP_LOGI(TAG, "green1");
        lamp(1,0,0);
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "green2");
        lamp(2,0,0);
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "yellow1");
        lamp(0,1,0);
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "yellow2");
        lamp(0,2,0);
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "red1");
        lamp(0,0,1);
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "red2");
        lamp(0,0,2);
        vTaskDelay(pdMS_TO_TICKS(5000));

    }

}
