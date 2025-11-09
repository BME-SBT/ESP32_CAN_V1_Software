#include <stdio.h>
#include "freertos/FreeRTOS.h" //freertos drivers
#include "freertos/task.h"     //task drivers
#include "esp_log.h"           //needed for logging
#include "driver/gpio.h"       //GPIO config/read/write
#include "esp_rom_sys.h"       //for delay
#include <stdbool.h>           //for bool
#include "solar.h"

const static char *TAG = "LED_SHIFTREG";

static uint8_t output_state = 0; // stores current output states

//sends data from the microcontroller to the shift register
void shiftreg_write(uint8_t data)
{
    for (int i = 7; i >= 0; i--) {
        bool bitVal = (data >> i) & 0x01;
        gpio_set_level(PIN_DATA, bitVal);

        gpio_set_level(PIN_CLK, 1);
        esp_rom_delay_us(1);
        gpio_set_level(PIN_CLK, 0);
    }
    // once all 8 bits are shifted in, toggle the LATCH pin.
    // this moves data from internal register → output pins (Q0–Q7).
    gpio_set_level(PIN_LATCH, 1);
    esp_rom_delay_us(1);
    gpio_set_level(PIN_LATCH, 0);
}


// this sets the GPIO pins as output so we can control the shift register
void shiftreg_init(void) {
    //reset
    gpio_reset_pin(PIN_DATA);
    gpio_reset_pin(PIN_CLK);
    gpio_reset_pin(PIN_LATCH);

    //set pin direction to output
    gpio_set_dir(PIN_DATA, 2);   // 2 = OUTPUT
    gpio_set_dir(PIN_CLK, 2);
    gpio_set_dir(PIN_LATCH, 2);


    // initialize them to low
    gpio_set_level(PIN_DATA, 0);
    gpio_set_level(PIN_CLK, 0);
    gpio_set_level(PIN_LATCH, 0);

    ESP_LOGI(TAG, "Shift register initialized");
}

//for LEDs functioning
void setOut(int bitToSet, bool value)
{
    if (bitToSet < 0 || bitToSet > 7) return;

    if (value)
        output_state |= (1 << bitToSet);
    else
        output_state &= ~(1 << bitToSet);

    shiftreg_write(output_state);
}

//error flags
bool can_error = false;
bool motor_error = false;
bool general_error = false;

//LED
#define USER_LED_BIT 0

//for morse code
void dot() {
    setOut(USER_LED_BIT, true);
    vTaskDelay(pdMS_TO_TICKS(150));
    setOut(USER_LED_BIT, false);
    vTaskDelay(pdMS_TO_TICKS(150));
}

void dash() {
    setOut(USER_LED_BIT, true);
    vTaskDelay(pdMS_TO_TICKS(450));
    setOut(USER_LED_BIT, false);
    vTaskDelay(pdMS_TO_TICKS(150));
}

// morse-inspired patterns
void morse_CAN() { dash(); dot(); dash(); dot(); vTaskDelay(pdMS_TO_TICKS(500)); }      // CAN
void morse_Motor() { dash(); dash(); dash(); vTaskDelay(pdMS_TO_TICKS(500)); }       // Motor
void morse_General() { dot(); dot(); dot(); vTaskDelay(pdMS_TO_TICKS(500)); } // General

//led control
void led_task(void *pvParameters)
{
    while (1)
    {
    switch (get_current_error()) {
        case can_error:
            morse_CAN();
            break;

        case motor_error:
            morse_Motor();
            break;

        case general_error:
            morse_General();
            break;

        default:
        // Normal heartbeat: blink by toggling LED state
            ledState = !ledState;                      // invert previous value
            setOut(USER_LED_BIT, ledState);            // update shift register or LED pin
            vTaskDelay(pdMS_TO_TICKS(500));            // heartbeat rate (0.5s ON/OFF)
            break;
        }
    }
}

void app_main(void)
{
    shiftreg_init();

    //start LED status task
    xTaskCreate(led_task, "LED_Task", 2048, NULL, 2, NULL);

    // Example: simulate errors (for testing)
    /*
    vTaskDelay(pdMS_TO_TICKS(5000));
    can_error = true;
    */
}
