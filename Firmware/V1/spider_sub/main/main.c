#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "driver/i2c_slave.h"

#include "motor.h"
#include "encoder.h"

#define I2C_SLAVE_SCL_IO GPIO_NUM_12
#define I2C_SLAVE_SDA_IO GPIO_NUM_13

#define I2C_ADDR 0x58
#define RD_LENGTH 4
#define WR_LENGTH 16

uint8_t *data_rd;
uint8_t *data_wr;

int rcv_rdy = 0;

i2c_slave_dev_handle_t slave_handle;
QueueHandle_t s_receive_queue;
i2c_slave_rx_done_event_data_t rx_data;
i2c_slave_config_t i2c_config = {
    .addr_bit_len = I2C_ADDR_BIT_LEN_7,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .send_buf_depth = 256,
    .scl_io_num = I2C_SLAVE_SCL_IO,
    .sda_io_num = I2C_SLAVE_SDA_IO,
    .slave_addr = I2C_ADDR,
};

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void prep_queue();

void app_main(void) {
    // esp_task_wdt_deinit();

    printf("Starting setup\n");

    motor_init();
    encoder_init();

    data_rd = (uint8_t *) malloc(RD_LENGTH);
    data_wr = (uint8_t *) malloc(WR_LENGTH);
    for (int i = 0; i < WR_LENGTH; i++) {
        data_wr[i] = i+1;
    }

    // prep_queue();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Starting main loop\n");
    ESP_ERROR_CHECK(i2c_new_slave_device(&i2c_config, &slave_handle));

    while (1) {
        prep_queue();
        printf("data: %d %d %d %d\n", (int)data_rd[0], (int)data_rd[1], (int)data_rd[2], (int)data_rd[3]);

        // printf("sent: %d %d %d %d\n", (int)data_wr[0], (int)data_wr[1], (int)data_wr[2], (int)data_wr[3]);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
void prep_queue() {
    // enable data recieve
    s_receive_queue = xQueueCreate(4, sizeof(i2c_slave_rx_done_event_data_t));
    i2c_slave_event_callbacks_t cbs = {
        .on_recv_done = i2c_slave_rx_done_callback,
    };
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slave_handle, &cbs, s_receive_queue));
    ESP_ERROR_CHECK(i2c_slave_receive(slave_handle, data_rd, RD_LENGTH));
    xQueueReceive(s_receive_queue, &rx_data, pdMS_TO_TICKS(10000));

    data_wr[0] = data_rd[0];
    data_wr[1] = data_rd[1];
    data_wr[2] = data_rd[2];
    data_wr[3] = data_rd[3];

    // data write
    ESP_ERROR_CHECK(i2c_slave_transmit(slave_handle, data_wr, 4, -1));

    // Gotta delete I2C device for some reason. This framework is shit
    // ESP_ERROR_CHECK(i2c_del_slave_device(slave_handle));
}
