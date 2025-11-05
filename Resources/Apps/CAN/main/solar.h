#pragma once
#ifndef MAIN_SOLAR_H_
#define MAIN_SOLAR_H_
#include "driver/gpio.h"
#include "driver/twai.h"

// start of pin deffinitions
#define PIN_GPIO_0 GPIO_NUM_1
#define PIN_GPIO_1 GPIO_NUM_0
#define PIN_GPIO_2 GPIO_NUM_7
#define PIN_GPIO_3 GPIO_NUM_6
#define PIN_GPIO_4 GPIO_NUM_5
#define PIN_GPIO_5 GPIO_NUM_4
#define PIN_GPIO_6 GPIO_NUM_16
#define PIN_GPIO_7 GPIO_NUM_9
#define PIN_GPIO_8 GPIO_NUM_18
#define PIN_GPIO_9 GPIO_NUM_19
#define PIN_GPIO_10 GPIO_NUM_20
#define PIN_GPIO_11 GPIO_NUM_21
#define PIN_GPIO_12 GPIO_NUM_22
#define PIN_GPIO_13 GPIO_NUM_23
#define PIN_GPIO_14 GPIO_NUM_15
#define PIN_GPIO_15 GPIO_NUM_17
#define PIN_CAN_RX GPIO_NUM_3
#define PIN_CAN_TX GPIO_NUM_2
#define PIN_CLK GPIO_NUM_8
#define PIN_LATCH GPIO_NUM_10
#define PIN_DATA GPIO_NUM_11
// end of pin deffinitions

#define MotorTemps_ID	0x200
#define MotorData_ID	0x201
#define BMSData_ID	    0x300
#define BMSExtra_ID	    0x301
#define GPS_ID	        0x400
#define Position_ID	    0x401
#define Control_ID	    0x100


// used instead of gpio_set_direction, this on actually works
void gpio_set_dir(int gpio, int mode) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    if (mode == 2) {
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    if (mode == 1) {
        io_conf.mode = GPIO_MODE_INPUT;
    }

    gpio_config(&io_conf);
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int clamp(int x, int min, int max) {
    if (x < min) {
        return (min);
    }
    if (x > max) {
        return (max);
    } else {
        return (x);
    }
}


/* Predefined TWAI frames (initialized with ID, DLC=8 and zeroed data)
   Data byte indexing in comments follows: bytes 7..0
*/
static twai_message_t motor_temps_frame = {
    .identifier = MotorTemps_ID,
    .extd = 0,
    .rtr = 0,
    .data_length_code = 8,
    .data = {0}
}; // MotorTemps: byte7+6=OutTemp, byte5+4=InTemp, byte3+2=ControllerTemp, byte1=MotorTemp

static twai_message_t motor_data_frame = {
    .identifier = MotorData_ID,
    .extd = 0,
    .rtr = 0,
    .data_length_code = 8,
    .data = {0}
}; // MotorData: byte6=PumpState, byte5+4=PumpState, byte3+2=ControllerCurrent, byte1=MotorRPM

static twai_message_t bms_data_frame = {
    .identifier = BMSData_ID,
    .extd = 0,
    .rtr = 0,
    .data_length_code = 8,
    .data = {0}
}; // BMSData: byte7+6=High Cell Voltage, byte5+4=Low Cell Voltage, byte3+2=Pack Voltage, byte1=Signed Pack Current

static twai_message_t bms_extra_frame = {
    .identifier = BMSExtra_ID,
    .extd = 0,
    .rtr = 0,
    .data_length_code = 8,
    .data = {0}
}; // BMSExtra: byte5=Lowest pack temp, byte4=Highest pack temp, byte3+2=Pack SOC, byte1=Pack Amphours

static twai_message_t gps_frame = {
    .identifier = GPS_ID,
    .extd = 0,
    .rtr = 0,
    .data_length_code = 8,
    .data = {0}
}; // GPS: byte7=GPS position

static twai_message_t position_frame = {
    .identifier = Position_ID,
    .extd = 0,
    .rtr = 0,
    .data_length_code = 8,
    .data = {0}
}; // Position: byte5+4=Heading, byte3=Pitch, byte2=Roll, byte1=Speed

static twai_message_t controll_frame = {
    .identifier = Control_ID,
    .extd = 0,
    .rtr = 0,
    .data_length_code = 8,
    .data = {0}
}; // Controll: byte2=AMS state?, byte1=Steering position?, byte0=Throttle position?


#endif /* MAIN_SOLAR_H_ */
