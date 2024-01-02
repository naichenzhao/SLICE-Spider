// i2c.h
#ifndef I2C_h
#define I2C_h

void i2c_init(void);

uint8_t i2c_read_io();

void esp_WR_RD(uint8_t speed1, uint8_t speed2, uint8_t speed3, uint8_t speed4, uint8_t *data_wr);

#endif