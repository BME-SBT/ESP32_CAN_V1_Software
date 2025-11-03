#include <stdio.h>
#include "freertos/FreeRTOS.h" //freertos drivers
#include "freertos/task.h"     //task drivers
#include "esp_log.h"           //needed for logging
#include "driver/gpio.h"       //GPIO config/read/write
#include "esp_rom_sys.h"       //for delay
#include <stdbool.h>           //for bool

const static char *TAG = "LED_SHIFTREG";

//pin definition
#define PIN_CLK GPIO_NUM_8 
#define PIN_LATCH GPIO_NUM_10 
#define PIN_DATA GPIO_NUM_11

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


//from github main, used instead of gpio_set_direction, this one actually works
void gpio_set_dir(int gpio, int mode)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),         // choose which pin(s) to configure
        .pull_up_en = GPIO_PULLUP_DISABLE,      // no internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // no internal pull-down
        .intr_type = GPIO_INTR_DISABLE,         // no interrupts
        .mode = GPIO_MODE_OUTPUT                // default mode (can be changed below)
    };

    if (mode == 1) {
        io_conf.mode = GPIO_MODE_INPUT;
    } else if (mode == 2) {
        io_conf.mode = GPIO_MODE_OUTPUT;
    }

    gpio_config(&io_conf);  // apply configuration
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
void morse_CAN() { dash(); dot(); dash(); dot(); vTaskDelay(pdMS_TO_TICKS(600)); }      // CAN
void morse_Motor() { dash(); dash(); dash(); vTaskDelay(pdMS_TO_TICKS(600)); }       // Motor
void morse_General() { dot(); dot(); dot(); vTaskDelay(pdMS_TO_TICKS(600)); } // General

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