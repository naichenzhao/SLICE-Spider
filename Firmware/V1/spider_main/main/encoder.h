// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

void encoder_init();
void reset_encoder(int num);

int get_count(int num);
float get_angle(int num);

#endif