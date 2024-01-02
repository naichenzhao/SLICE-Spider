#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include <string.h>

#include <i2c_comm.h>

#define DATA_LENGTH 512    /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128 /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS \
  20 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO GPIO_NUM_12
#define I2C_SLAVE_SDA_IO GPIO_NUM_13

#define I2C_SLAVE_NUM 0
#define I2C_SLAVE_TX_BUF_LEN 256  //(2 * DATA_LENGTH)
#define I2C_SLAVE_RX_BUF_LEN 256  //(2 * DATA_LENGTH)

#define ESP_SLAVE_ADDR 0x58
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define SLAVE_REQUEST_WAIT_MS 100

const uint8_t testCmd[15] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                             0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};

uint8_t outBuff[256];
uint16_t outBuffLen = 0;

uint8_t inBuff[256];
uint16_t inBuffLen = 0;


void i2c_slave_init() {
  i2c_port_t i2c_slave_port = I2C_SLAVE_NUM;
  i2c_config_t conf_slave;
  conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
  conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
  conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.mode = I2C_MODE_SLAVE;
  conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    
  
  i2c_param_config(i2c_slave_port, &conf_slave);
  i2c_driver_install(i2c_slave_port, conf_slave.mode,
                            I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);

  xTaskCreate(i2cs_test_task, "slave", 1024 * 2, (void *)1, 10, NULL);
//   xTaskCreate(send_task, "send_task", 1024 * 2, (void *)1, 10, NULL);
}

bool check_for_data() {
  uint32_t startMs = esp_timer_get_time() / 1000;
  size_t size =
      i2c_slave_read_buffer(I2C_SLAVE_NUM, inBuff, 1, 1000 / portTICK_PERIOD_MS);
  uint32_t stopMs = esp_timer_get_time() / 1000;
  if (size == 1) {
    if (inBuff[0] == 0x01) {
      uint8_t replBuff[2];
      replBuff[0] = (uint8_t)(outBuffLen >> 0);
      replBuff[1] = (uint8_t)(outBuffLen >> 8);
      int ret = i2c_reset_tx_fifo(I2C_SLAVE_NUM);
      if (ret != ESP_OK) {
          printf("failed to reset fifo\n");
      }
      i2c_slave_write_buffer(I2C_SLAVE_NUM, replBuff, 2,
                             1000 / portTICK_PERIOD_MS);
      printf("got len request, put(%d), waited: %ld ms\n", (int)outBuffLen,
             (int)stopMs - startMs);
      vTaskDelay(pdMS_TO_TICKS(SLAVE_REQUEST_WAIT_MS));
      return false;
    }
    if (inBuff[0] == 0x02) {
      int ret = i2c_reset_tx_fifo(I2C_SLAVE_NUM);
      if (ret != ESP_OK) {
          printf("failed to reset fifo\n");
      }
      i2c_slave_write_buffer(I2C_SLAVE_NUM, outBuff, outBuffLen,
                             1000 / portTICK_PERIOD_MS);
      outBuffLen = 0;
      printf("got write request, waited: %ld ms\n", (int)stopMs - startMs);
      vTaskDelay(pdMS_TO_TICKS(SLAVE_REQUEST_WAIT_MS));
      return false;
    }
    if (inBuff[0] > 0x02) {
      vTaskDelay(pdMS_TO_TICKS(10));
      size_t size_pl = i2c_slave_read_buffer(I2C_SLAVE_NUM, inBuff, inBuff[0],
                                             20 / portTICK_PERIOD_MS);
      inBuffLen = size_pl;
      printf("reading %d bytes, waited: %ld ms\n", (int)inBuff[0],
             (int)stopMs - startMs);
      return true;
    }
  }
  if (size > 1) {
      printf("weird, waited: %ld ms\n", (int)stopMs - startMs);
      inBuffLen = size;
      return true;
  }
  printf("nothing, len: %d, waited: %ld ms\n", (int)size, (int)stopMs - startMs);
  return false;
}

static void i2cs_test_task(void *arg) {
  while (1) {
    if (check_for_data()) {
        printf("got %d bytes:\n", (int)inBuffLen);
        // ESP_LOG_BUFFER_HEX(TAG, inBuff, inBuffLen);
        inBuffLen = 0;
    }
  }
  vTaskDelete(NULL);
}

static void send_task(void *arg) {
  while (1) {
      vTaskDelay((500) / portTICK_PERIOD_MS);
      memcpy(outBuff, testCmd, 15);
      outBuffLen = 15;
      printf("new data ready for send\n");
  }
  vTaskDelete(NULL);
}