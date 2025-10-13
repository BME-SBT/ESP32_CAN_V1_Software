//balaton test version
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
#include "sdkconfig.h"
#include <string.h>
#include "driver/uart.h"
#include <stdint.h>


// shift‑reg
#define PIN_SR_CLK    GPIO_NUM_4
#define PIN_SR_DATA   GPIO_NUM_7
#define PIN_SR_LATCH1 GPIO_NUM_5
#define PIN_SR_LATCH2 GPIO_NUM_6


const static char *TAG = "CLYDE_1";
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define TX_PIN             8    // Change to your actual TX pin
#define RX_PIN             2
#define BUF_SIZE           1024

#define THROTTLE ADC_CHANNEL_0
#define STEERING ADC_CHANNEL_1
#define STEERING_ENA        PIN_GPIO_11
#define AMS                 PIN_GPIO_10
#define AI_START            PIN_GPIO_5
#define AI_MODE_SLALOM      PIN_GPIO_2
#define AI_MODE_TAXI        PIN_GPIO_3
#define AI_MODE_DOCKING     PIN_GPIO_4

#define MIN_THROTTLE_POSITION          360     // Minimum input
#define MAX_THROTTLE_POSITION          2090    // Maximum input
#define THROTTLE_DEAD_ZONE             200     // Throttle dead zone
#define MIN_STEERING_POSITION          680     // Minimum input
#define MAX_STEERING_POSITION          2910    // Maximum input
#define STEERING_DEAD_ZONE             150     // steering dead zone
#define MIN_DATA                       0       // Minimum input
#define MAX_DATA                       255     // Maximum output

#define MODE_SLALOM     2
#define MODE_TAXI       1
#define MODE_DOCKING    3
#define MODE_RUNNING    4
#define MODE_ERROR      9

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

//0 off, 1 blinking, 2 on green, yellow, red
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

int buttons(){
    int i = 0;
    i = i + gpio_get_level(AI_START);
    i = i + gpio_get_level(AI_MODE_SLALOM);
    i = i + gpio_get_level(AI_MODE_TAXI);
    i = i + gpio_get_level(AI_MODE_DOCKING);
    if(i == 4){
        return 0;
    }
    
    if (i == 3){
        if(gpio_get_level(AI_START) == 0){return 9;}
        if(gpio_get_level(AI_MODE_TAXI) == 0){return 1;}
        if(gpio_get_level(AI_MODE_SLALOM) == 0){return 2;}
        if(gpio_get_level(AI_MODE_DOCKING) == 0){return 3;}
    }

    if ( i < 3 && gpio_get_level(AI_START) == 0){
        return 4;
    }
    if ( i < 3){
        return 9;
    }
    return 9;
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
    setup_gpio();
    gpio_set_dir(AMS, GPIO_MODE_INPUT);
    gpio_set_dir(STEERING_ENA, GPIO_MODE_INPUT);
    gpio_set_dir(AI_START, GPIO_MODE_INPUT);
    gpio_set_dir(AI_MODE_TAXI, GPIO_MODE_INPUT);
    gpio_set_dir(AI_MODE_SLALOM, GPIO_MODE_INPUT);
    gpio_set_dir(AI_MODE_DOCKING, GPIO_MODE_INPUT);
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

     const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART initialized. Sending '0' at 10Hz...");

    ai_messages.data[0] = 69;
    ai_messages.data[1] = 69;
    ai_messages.data[2] = 69;
    

    while(1){
        //reading and converting throttle position
        adc_oneshot_read(adc_handle, THROTTLE, &throttle);
        ESP_LOGI(TAG, "Throttle raw: %d", throttle);
        controll.data[0] = data_to_pos(throttle, MIN_THROTTLE_POSITION, MAX_THROTTLE_POSITION);
        if(((MIN_THROTTLE_POSITION + MAX_THROTTLE_POSITION) / 2) - (THROTTLE_DEAD_ZONE / 2) < throttle && throttle < ((MIN_THROTTLE_POSITION + MAX_THROTTLE_POSITION) / 2) + (THROTTLE_DEAD_ZONE / 2) ){
            controll.data[0] = 127;
        }

        //sending CAN data
        //constans orange
        lamp(1, 2, 0);
        ESP_ERROR_CHECK(twai_transmit(&controll, pdMS_TO_TICKS(10)));
        ESP_ERROR_CHECK(twai_transmit(&ai_messages, pdMS_TO_TICKS(10)));
        ESP_LOGI(TAG, "TWAI OK");
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
        
    }
}
