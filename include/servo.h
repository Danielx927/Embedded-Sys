#ifndef SERVO_H
#define SERVO_H

#include <stdio.h>

void set_servo_pulse(uint16_t us);

void init_servo(void);

float servo_pulse_to_angle(uint16_t pulse_us);

#endif