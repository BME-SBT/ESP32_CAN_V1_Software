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
















#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"

#define UART_PORT_NUM     1      // UART1
#define UART_IO_PIN       8      // Shared TX/RX pin (GPIO4)

void dump_uart_rx_buffer()
{
    const int buf_size = 128;
    uint8_t buf[buf_size];
    int len = uart_read_bytes(UART_PORT_NUM, buf, buf_size, 100 / portTICK_PERIOD_MS);

    if (len > 0) {
        printf("Received %d bytes:\n", len);
        for (int i = 0; i < len; i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");

    } else {
        printf("No data received.\n");
    }
}

static const char *TAG = "MotorController";

uint8_t motor_data_raw[47];
float motor_voltage = 0.0f;
bool success = false;

int controller_temp, motor_temp, rpm;
float current, power;

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

void read_motor_data()
{
    for (int i = 0; i < sizeof(motor_data_raw); i++) {
        motor_data_raw[i] = 0;
    }

    uint8_t request[2] = { 0x80, 0x8d };
    uart_write_bytes(UART_PORT_NUM, (const char *)request, sizeof(request));

    vTaskDelay(100 / portTICK_PERIOD_MS);



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

void app_main()
{

    ESP_LOGI(TAG, "Initializing...");
    init_uart();
    while (1)
    {
        read_motor_data();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
