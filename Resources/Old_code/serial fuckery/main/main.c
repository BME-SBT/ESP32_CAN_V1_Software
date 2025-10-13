#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define TX_PIN             1    // Change to your actual TX pin
#define RX_PIN             0
#define BUF_SIZE           1024

static const char *TAG = "SerialSender";

void app_main(void)
{
    // UART configuration
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Install UART driver
    uart_driver_install(UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART initialized. Sending '0' at 10Hz...");
    

    while (1) {
        uint8_t ai_data[3] = {0, 127, 127};
        char data[3] = {'1', '\r', '\n'};

        uart_write_bytes(UART_PORT_NUM, (const char *)&data, sizeof(data));
        ESP_LOGI(TAG, "Sent 1");
        uart_read_bytes(UART_PORT_NUM, ai_data, sizeof(ai_data), 100 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "received: %d %d %d",ai_data[0], ai_data[1], ai_data[2]);
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz = every 100ms
        
    }
}

throttle = 127;
steering = 107;

for(int i = 1; i<41; i++){
    thrtottle++;
    steering++;
}

for(int i = 1; i<41; i++){
    thrtottle--;
    steering--;
}
