// i2c_comm.h
#ifndef I2C_COMM_h
#define I2C_COMM_h

void i2c_slave_init();
bool check_for_data();

static void i2cs_test_task(void *arg);
static void send_task(void *arg);

#endif