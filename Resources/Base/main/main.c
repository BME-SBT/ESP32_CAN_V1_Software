#include <stdio.h>
#include "freertos/FreeRTOS.h" //freertos drivers
#include "freertos/task.h"     //task drivers
#include "esp_log.h"           //needed for logging
#include "driver/twai.h"       //for can

//start of pin deffinitions
#define PIN_GPIO_0        GPIO_NUM_1
#define PIN_GPIO_1        GPIO_NUM_0
#define PIN_GPIO_2        GPIO_NUM_7
#define PIN_GPIO_3        GPIO_NUM_6
#define PIN_GPIO_4        GPIO_NUM_5
#define PIN_GPIO_5        GPIO_NUM_4
#define PIN_GPIO_6        GPIO_NUM_16
#define PIN_GPIO_7        GPIO_NUM_9
#define PIN_GPIO_8        GPIO_NUM_18
#define PIN_GPIO_9        GPIO_NUM_19
#define PIN_GPIO_10       GPIO_NUM_20
#define PIN_GPIO_11       GPIO_NUM_21
#define PIN_GPIO_12       GPIO_NUM_22
#define PIN_GPIO_13       GPIO_NUM_23
#define PIN_GPIO_14       GPIO_NUM_15
#define PIN_GPIO_15       GPIO_NUM_17
#define PIN_CAN_RX        GPIO_NUM_3
#define PIN_CAN_TX        GPIO_NUM_2
#define PIN_CLK           GPIO_NUM_8
#define PIN_LATCH         GPIO_NUM_10
#define PIN_DATA          GPIO_NUM_11
//end of pin deffinitions

const static char *TAG = "<BoardName>"; //<BoardName> is the name of the board used in logging

//used instead of gpio_set_direction, this on actually works
void gpio_set_dir(int gpio, int mode){

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    if(mode == 2){
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    if(mode == 1){
        io_conf.mode = GPIO_MODE_INPUT;
    }
    
    gpio_config(&io_conf);
}



void app_main(void)
{
//----------begining of setup----------//

    // put your setup code here, to run once:

//------------end of setup------------//
    while(1){
        // put your main code here, to run repeatedly:
    }

}