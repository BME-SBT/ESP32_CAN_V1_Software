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
const static char *TAG = "PINKY_2";
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

void shift (){
    
}


void app_main(void)
{
    twai_message_t motor_temps = {
        .identifier = 0x200,
        .data_length_code = 8,
        .data = {0},
        .extd = 0,
        .rtr = 0
    };
    twai_message_t motor_data = {
        .identifier = 0x201,
        .data_length_code = 7,
        .data = {0},
        .extd = 0,
        .rtr = 0
    };
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    int16_t MotorTemp = 250;
    int16_t ControllerTemp = 690;
    int16_t InTemp = 0;
    int16_t OutTemp = 0;
    int16_t MotorRPM = 0;
    int16_t ControllerCurrent = 0;
    int16_t  MotorPower = 0;
    uint8_t PumpState = 0;



    while(1){

        motor_temps.data[0] = (int8_t)((MotorTemp >> 8) & 0xFF);
        motor_temps.data[1] = (int8_t)(MotorTemp & 0xFF);  
        motor_temps.data[2] = (int8_t)((ControllerTemp >> 8) & 0xFF);
        motor_temps.data[3] = (int8_t)(ControllerTemp & 0xFF);  
        motor_temps.data[4] = (int8_t)((InTemp >> 8) & 0xFF);
        motor_temps.data[5] = (int8_t)(InTemp & 0xFF);  
        motor_temps.data[6] = (int8_t)((OutTemp >> 8) & 0xFF);
        motor_temps.data[7] = (int8_t)(OutTemp & 0xFF);  

        motor_data.data[0]  = (int8_t)((MotorRPM >> 8) & 0xFF);
        motor_data.data[1]  = (int8_t)(MotorRPM & 0xFF); 
        motor_data.data[2]  = (int8_t)((ControllerCurrent >> 8) & 0xFF);
        motor_data.data[3]  = (int8_t)(ControllerCurrent & 0xFF);  
        motor_data.data[4]  = (int8_t)((MotorPower >> 8) & 0xFF);
        motor_data.data[5]  = (int8_t)(MotorPower & 0xFF);  
        motor_data.data[6]  = PumpState;





        twai_transmit(&motor_temps, pdMS_TO_TICKS(10));
        twai_transmit(&motor_data, pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(10));

        //MotorTemp++;
        //ControllerTemp++;
        InTemp++;
        OutTemp++;
        MotorRPM++;
        ControllerCurrent++;
        MotorPower++;

    }

}
