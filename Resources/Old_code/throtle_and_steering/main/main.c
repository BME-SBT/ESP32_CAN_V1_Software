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
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"

const static char *TAG = "CLYDE_1";
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


#define THROTTLE ADC_CHANNEL_0
#define STEERING ADC_CHANNEL_1
#define STEERING_ENA PIN_GPIO_11
#define AMS PIN_GPIO_8
#define STEERING_LED PIN_GPIO_1
#define THROTTLE_LED PIN_GPIO_2
#define AMS_LED PIN_GPIO_3

#define MIN_THROTTLE_POSITION          360     // Minimum input
#define MAX_THROTTLE_POSITION          2000    // Maximum input
#define MIN_STEERING_POSITION          570     // Minimum input
#define MAX_STEERING_POSITION          2600    // Maximum input
#define MIN_DATA                       0       // Minimum input
#define MAX_DATA                       255     // Maximum output

int steering_center = 127;
int throttle_center = 127;
int AMS_latch = 0;
int AMS_fault = 0;


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

int map(int x, int in_min, int in_max, int out_min, int out_max){
    return((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int clamp(int x, int min, int max){
    if(x<min){
        return(min);
    }
    if(x>max){
        return(max);
    }
    else{
        return(x);
    }
}

int data_to_pos(int data, int min, int max){
    return(map(clamp(data, min, max), min, max, MIN_DATA, MAX_DATA));
}


void app_main(void)
{
    static int throttle = 0;
    static int steering = 0;
    twai_message_t controll = {
        .identifier = 0x100,
        .data_length_code = 4,
        .ss = 1, 
        .data = {0},
        .extd = 0,
        .rtr = 0
    };
    twai_message_t ai_messages = {
        .identifier = 0x101,
        .data_length_code = 3,
        .ss = 1, 
        .data = {0},
        .extd = 0,
        .rtr = 0
    };
    //begining of I/O setup
    gpio_set_dir(AMS, GPIO_MODE_INPUT);
    gpio_set_dir(STEERING_ENA, GPIO_MODE_INPUT);
    gpio_set_dir(STEERING_LED, GPIO_MODE_OUTPUT);
    gpio_set_dir(THROTTLE_LED, GPIO_MODE_OUTPUT);
    gpio_set_dir(AMS_LED, GPIO_MODE_OUTPUT);

   


    //end of I/O setup
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    
    //-------------ADC Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, THROTTLE, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, STEERING, &config));
    //TWAI start
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    int AMS_STATE = 0;

    ai_messages.data[0] = 1;
    ai_messages.data[1] = 2;
    ai_messages.data[2] = 3;
    while(1){
        //reading and converting throttle position
        adc_oneshot_read(adc_handle, THROTTLE, &throttle);
        ESP_LOGI(TAG, "Throttle raw: %d", throttle);
        controll.data[0] = data_to_pos(throttle, MIN_THROTTLE_POSITION, MAX_THROTTLE_POSITION);

        //reading and converting steering position
        adc_oneshot_read(adc_handle, STEERING, &steering);
        ESP_LOGI(TAG, "Steering raw: %d", steering);
        controll.data[1] = 255 - (data_to_pos(steering, MIN_STEERING_POSITION, MAX_STEERING_POSITION));

        //reading AMS state
        AMS_STATE = (gpio_get_level(AMS) == 1) ? 0 : 1;
        controll.data[3] = (gpio_get_level(STEERING_ENA) == 1) ? 0 : 1;

        if((throttle_center-5)< controll.data[0] && controll.data[0] < (throttle_center+5)){
            gpio_set_level(THROTTLE_LED, 1);
        }
        if((throttle_center-5)> controll.data[0] || controll.data[0]> (throttle_center+5)){
            gpio_set_level(THROTTLE_LED, 0);
        }
        if((steering_center-15)< controll.data[1] && controll.data[1] < (steering_center+15)){
            gpio_set_level(STEERING_LED, 1);
        }
        if((steering_center-15)> controll.data[1] || controll.data[1]> (steering_center+15)){
            gpio_set_level(STEERING_LED, 0);
        }

        if(AMS_STATE == 0){
            AMS_fault = 0;
        }

        if(AMS_STATE != AMS_latch){
            if(AMS_STATE == 0){
                AMS_latch = 0;
            }
            if((throttle_center-5)< controll.data[0] && controll.data[0] < (throttle_center+5) && (steering_center-15)< controll.data[1] && controll.data[1] < (steering_center+15) && AMS_fault == 0){
                AMS_latch = AMS_STATE;
            }
            else{
                AMS_latch = 0;
                AMS_fault = 1;
            }
        }


        controll.data[2] = AMS_latch;
        gpio_set_level(AMS_LED, controll.data[2]);
        ESP_LOGI(TAG, "AMS out: %d", controll.data[2]);

        //sending CAN data
        ESP_LOGI(TAG, "Sending throttle data: %d", controll.data[0]);
        ESP_LOGI(TAG, "Sending steering data: %d", controll.data[1]);
        ESP_LOGI(TAG, "Sending AMS data: %d", controll.data[2]);
        ESP_LOGI(TAG, "Sending STEERING_ENA data: %d", controll.data[3]);
        ESP_ERROR_CHECK(twai_transmit(&controll, pdMS_TO_TICKS(10)));
        ESP_ERROR_CHECK(twai_transmit(&ai_messages, pdMS_TO_TICKS(10)));
        ESP_LOGI(TAG, "TWAI OK");
        vTaskDelay(pdMS_TO_TICKS(10)); //delay to set CAN bus timing and let FREERTOS do its thing
    }
    
   


}
