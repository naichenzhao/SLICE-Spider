#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO GPIO_NUM_12
#define I2C_MASTER_SDA_IO GPIO_NUM_13

#define DEV_ADDR GPIO_NUM_13
int SW_REMAP[8] = {1, 0, 5, 4, 3, 2, 6, 7};

#define I2C_PORT 0x20
i2c_master_dev_handle_t dev_handle;

void switch_init(void) {

    i2c_master_bus_config_t i2c_mst_config_1 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config_1, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, I2C_PORT, -1));
    printf("IO Expander Found!\n");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_PORT,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    printf("Setup done\n");
}

uint8_t i2c_read_byte(uint8_t periph_address) {
    uint8_t buf[20] = {periph_address};
    uint8_t buffer[2];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, sizeof(buf), buffer, 2, -1));
    return buffer[0];
}

uint8_t read_switches() {
    return i2c_read_byte(0x00);
}

uint8_t read_switch(int num) {
    return (0x01) & (i2c_read_byte(0x00) >> SW_REMAP[num]);
}