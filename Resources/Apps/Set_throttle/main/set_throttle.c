#include <stdio.h>
#include "freertos/FreeRTOS.h" //freertos drivers
#include "freertos/task.h"     //task drivers
#include "esp_log.h"           //needed for logging
#include "driver/twai.h"       //for can
#include "solar.h"

// pwm includes
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_timer.h"

//start of definitions

#define PULSE_GPIO 8

#define MIN_PULSEWIDTH_US 1000  // Minimum pulse width in microsecond
#define MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define MIN_REV           1483  // Minimum input
#define MAX_REV           1982  // Maximum input

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        10000    // 10000 ticks, 10ms
#define GROUP_ID 0 // Same group for all

//end of definitions

const static char *TAG = "<BoardName>"; //<BoardName> is the name of the board used in logging

// clamps potmeter data to MAX_REV MIN_REV, and maps it to the same range linearly
static inline uint16_t clamp_map(uint8_t x) {
  if (x > MAX_PULSEWIDTH_US) return MAX_REV;
  if (x < MIN_PULSEWIDTH_US) return MIN_REV;
  return (x - MIN_PULSEWIDTH_US) * ((MAX_REV - MIN_REV) / (MAX_PULSEWIDTH_US - MIN_PULSEWIDTH_US)) + MIN_REV;
}
/*
 * Question:
 * if I get an 8 bit number what am I really testing for here with
 * max min pulsewidth? or was the existing code just bad?
 */

// initializing the timer, operator, generator, comperator for pwm pulse generation
mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t cmpr = NULL;
mcpwm_gen_handle_t generator = NULL;

void init_pwm() {

  //Timer config
  mcpwm_timer_config_t timer_config = {
    .group_id = GROUP_ID,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
    .period_ticks = SERVO_TIMEBASE_PERIOD,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  mcpwm_new_timer(&timer_config, &timer);
  // TODO error handling for no more timer

  // Operator config
  mcpwm_operator_config_t oper_config = {
    .group_id = GROUP_ID, // same group ids as timer
  };
  mcpwm_new_operator(&oper_config, &oper);
  // TODO error handling for no more operators

  // Connect operator and timer
  mcpwm_operator_connect_timer(oper, timer);

  // Comperator config
  mcpwm_comparator_config_t cmpr_config = {
    .flags.update_cmp_on_tez = true,
  };
  mcpwm_new_comparator(oper, &cmpr_config, &cmpr);
  // TODO error handling

  // Generator config
  mcpwm_generator_config_t generator_config = {
    .gen_gpio_num = PULSE_GPIO,
  };
  mcpwm_new_generator(oper, &generator_config, &generator);
  // TODO error handling

  // initial compare value to the middle of the interval
  mcpwm_comparator_set_compare_value(cmpr, (MIN_REV + MAX_REV) / 2);

  // go high on counter empty
  mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
  // go low on compare threshold
  mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr, MCPWM_GEN_ACTION_LOW));

  // enable timer
  mcpwm_timer_enable(timer);
  // start timer
  mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

  // set to half the interval initially
  mcpwm_comparator_set_compare_value(cmpr, (MIN_REV + MAX_REV) / 2);
}

void vTaskSetThrottle(void* pvParameters) {
  init_pwm();

  // TODO get potmeter (throttle) position from CAN
  uint8_t potmeterData = 123;

  while (1) {

    // Set the compare value of the pwm comperator to the clamped mapped potmeter data
    mcpwm_comparator_set_compare_value(cmpr, clamp_map(potmeterData));

    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
  }
}

void app_main(void)
{

  // Create the task
  xTaskCreate(
    vTaskSetThrottle,
    "PWM generation",
    2048,
    NULL,
    3,
    NULL
  );

}
