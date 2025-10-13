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
#define UART_PORT_NUM     1 

uint8_t motor_data_raw[47];
float motor_voltage = 0.0f;
bool success = false;

int controller_temp, motor_temp, rpm;
float current, power;

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

void init_uart()
{
    const uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 1024, 0, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    // Set mode to half-duplex (TX and RX on same line)
    ESP_ERROR_CHECK(uart_set_mode(UART_PORT_NUM, UART_MODE_UART));

    // Set same GPIO for TX and RX
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, 0, 1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    
}






twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

#define ID_CONTROLL      0x100
#define ID_MOTOR_TEMPS   0x200
#define ID_MOTOR_DATA    0x201

#define MIN_PULSEWIDTH_US 1280  // Minimum pulse width in microsecond
#define MAX_PULSEWIDTH_US 1830  // Maximum pulse width in microsecond
#define MAX_ANGLE          179  // Maximum angle of stepper
#define MIN_ANGLE         -179  // Minimum angle of stepper
#define MIN_REV              0  // Minimum input
#define MAX_REV            255  // Maximum input
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

//stepper
#define DIRECTION_PIN PIN_GPIO_10
#define STEP_PIN PIN_GPIO_11

#define PULSE_WIDTH_MICROS 100
#define STEP_DELAY_MICROS 500
#define STEPS_PER_REVOLUTION 1600

static float currentAngle = 0.0;  // Start at 0°

// Normalize angle to -180° to +180°
float normalizeAngle(float angle) {
    angle = fmod(angle + 180.0, 360.0);
    if (angle < 0) angle += 360.0;
    return angle - 180.0;
}

void stepMotor(int steps, bool direction) {
    gpio_set_level(DIRECTION_PIN, direction ? 1 : 0);  // Set direction pin

    for (int i = 0; i < steps; i++) {
        gpio_set_level(STEP_PIN, 1);  // Set step pin HIGH
        esp_rom_delay_us(PULSE_WIDTH_MICROS); 
        gpio_set_level(STEP_PIN, 0);  // Set step pin LOW
        esp_rom_delay_us(STEP_DELAY_MICROS);   // Delay between steps
    }
}

// Rotate to target angle
void rotateToAngle(float targetAngle) {
    float angleDifference = targetAngle - currentAngle;
    int stepsToMove = abs(angleDifference / 360.0 * STEPS_PER_REVOLUTION);
    bool direction = angleDifference >= 0; // true = CW, false = CCW

    ESP_LOGI("Stepper", "Rotating from %.2f° to %.2f° | Steps: %d | Direction: %s", currentAngle, targetAngle, stepsToMove, direction ? "CW" : "CCW");

    stepMotor(stepsToMove, direction);
    currentAngle = targetAngle;
}

int rotateToAngle(float targetAngle) {
    float angleDifference = targetAngle - currentAngle;
    if(angleDifference > 3 && angleDifference > 0){
        angleDifference = 3;
        currentAngle = currentAngle + 3;
    }
    if(angleDifference < -3 && angleDifference < 0){
        angleDifference = -3;
        currentAngle = currentAngle - 3;
    }
    if((angleDifference >= 3 && angleDifference < 0) || (angleDifference <= -3 && angleDifference > 0)){
        currentAngle = currentAngle - angleDifference;
    }

void read_motor_data()
{
    for (int i = 0; i < sizeof(motor_data_raw); i++) {
        motor_data_raw[i] = 0;
    }

    uint8_t request[2] = { 0x80, 0x8d };
    uart_write_bytes(UART_PORT_NUM, (const char *)request, sizeof(request));

    vTaskDelay(10 / portTICK_PERIOD_MS);



    int len = uart_read_bytes(UART_PORT_NUM, motor_data_raw, sizeof(motor_data_raw), 100 / portTICK_PERIOD_MS);
    //ESP_LOGI(TAG, "%d", motor_data_raw[0]);
    

    if (len == sizeof(motor_data_raw) && motor_data_raw[2] == 0x7c && motor_data_raw[3] == 0x8d)
    {
        if (motor_data_raw[45] == 0x7d)
        {
            success = true;
            controller_temp = motor_data_raw[18] - 20;
            motor_temp = motor_data_raw[19] - 20;
            rpm = ((motor_data_raw[24] << 8) | motor_data_raw[23]) * 10;
            current = ((motor_data_raw[31] << 8) | motor_data_raw[30]) / 10.0;
            motor_voltage = ((motor_data_raw[33] << 8) | motor_data_raw[32]) / 10.0;
            power = current * motor_voltage;

            ESP_LOGI(TAG, "Motor Data: RPM=%d, Current=%.2f, Voltage=%.2f, Power=%.2f, Motor Temp=%d, Controller Temp=%d",
                     rpm, current, motor_voltage, power, motor_temp, controller_temp);
        }
        else
        {
            success = false;
            ESP_LOGW(TAG, "Invalid motor data checksum");
        }
    }
    else
    {
        success = false;
        ESP_LOGW(TAG, "Failed to read motor data");
    }
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

int data_to_pos(int data){
    return(map(clamp(data, MIN_REV, MAX_REV), MIN_REV, MAX_REV, MIN_PULSEWIDTH_US, MAX_PULSEWIDTH_US));
}

int data_to_angle(int data){
    return(map(clamp(data, MIN_REV, MAX_REV), MIN_REV, MAX_REV, MIN_ANGLE, MAX_ANGLE));
}





void app_main(void)
{
    twai_message_t controll = {
        .identifier = ID_CONTROLL,
        .data_length_code = 5,
        .data = {0},
        .extd = 0,
        .rtr = 0
    };

    controll.data[0] = ((MIN_REV+MAX_REV) / 2);
    controll.data[1] = ((MIN_REV+MAX_REV) / 2);
    controll.data[2] = 0;
    controll.data[3] = 0;
    controll.data[4] = 0;

    twai_message_t motor_temps = {
        .identifier = ID_MOTOR_TEMPS,
        .data_length_code = 8,
        .data = {0},
        .extd = 0,
        .rtr = 0
    };

    twai_message_t motor_data = {
        .identifier = ID_MOTOR_DATA,
        .data_length_code = 5,
        .data = {0},
        .extd = 0,
        .rtr = 0
    };

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    // start of pwm config -------------------------------------------------------------------------------------------------------------------------------------------------
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        
    };
    mcpwm_new_timer(&timer_config, &timer);
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    mcpwm_new_operator(&operator_config, &oper);
    // Connect timer and operator
    mcpwm_operator_connect_timer(oper, timer);
    // Create comparator and generator from the operator
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(oper, &comparator_config, &comparator);
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = PIN_GPIO_8,
    };
    mcpwm_new_generator(oper, &generator_config, &generator);
    mcpwm_comparator_set_compare_value(comparator, 0); // set the initial compare value to the middle
    // go high on counter empty
    mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    // go low on compare threshold
    mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW));
    // enable timer
    mcpwm_timer_enable(timer);
    // start timer
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    //end of pwm config ----------------------------------------------------------------------------------------------------------------------------------------------------

    int position = 0;

    //stepper
    gpio_set_dir(DIRECTION_PIN, GPIO_MODE_OUTPUT);
    gpio_set_dir(STEP_PIN, GPIO_MODE_OUTPUT);
    init_uart();

    while(1){
        read_motor_data();
        twai_transmit(&motor_temps, pdMS_TO_TICKS(10));
        twai_transmit(&motor_data, pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
        twai_message_t rx_msg;
        esp_err_t receive_error = ESP_OK;
        int cntr = 0;
        receive_error = twai_receive(&rx_msg, pdMS_TO_TICKS(50));
        ESP_LOGI(TAG, "while loop %d", cntr);

        while(rx_msg.identifier != ID_CONTROLL && cntr < 11){
            twai_receive(&rx_msg, pdMS_TO_TICKS(50));
            
            cntr ++;
            ESP_LOGI(TAG, "while loop %d", cntr);

        }
        
        if(rx_msg.identifier == ID_CONTROLL){
            controll.data[0] = rx_msg.data[0];
            controll.data[1] = rx_msg.data[1];
            controll.data[2] = rx_msg.data[2];
            controll.data[3] = rx_msg.data[3];
            controll.data[4] = 1;

            ESP_LOGI(TAG, "Received reply: %d", rx_msg.data[1]); 
        }
        if(cntr > 10){
            ESP_LOGW(TAG, "No reply received.");
            cntr = 0;
        }
        if(receive_error != ESP_OK){
            ESP_LOGW(TAG, "No reply received.");
            controll.data[4] = 0;

        }
        if(controll.data[4] == 1){
            position = data_to_pos(controll.data[0]);
            mcpwm_comparator_set_compare_value(comparator, position);

            float targetAngle = normalizeAngle(data_to_angle(controll.data[1]));
            // If target angle is the same as the current angle, do nothing
            if (targetAngle == currentAngle) {
                ESP_LOGI("Stepper", "Target angle is the same as current. No movement.");
            } else {
                float angleDifference = targetAngle - currentAngle;

                // If moving over the 180° or -180° limit, make sure it rotates through 0°
                if ((currentAngle == 180 && targetAngle == -180) || (currentAngle == -180 && targetAngle == 180)) {
                    rotateToAngle(0);  // Go to 0° first
                    rotateToAngle(targetAngle);  // Then move to the target
                    currentAngle = targetAngle;
                } 
                else {
                    rotateToAngle(targetAngle);
                    currentAngle = targetAngle;
                }
        }
            

            ESP_LOGI(TAG, "position %d", position);
        }
        if(controll.data[4] != 1){
            int temp = ((MIN_REV+MAX_REV) / 2);
            position = data_to_pos(temp);
        }
        ESP_LOGI(TAG, "3");
        


    }

}
