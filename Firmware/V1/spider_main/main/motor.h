// motor.h
#ifndef MOTORS_h
#define MOTORS_h

void motor_init(void);

void set_motor(int number, float speed);

void home_motor_sw(int num);
void home_motor_enc(int num);

#endif