#ifndef SERVO_H
#define SERVO_H

#include <stdio.h>

// Public constants for servo configuration
#define SERVO_PIN            15
#define SERVO_LEFT_MAX       2000
#define SERVO_RIGHT_MAX      1000
#define SERVO_LEFT_START     1550  
#define SERVO_RIGHT_START    1450
#define SERVO_CENTER         1500
#define SERVO_STEP           10
#define SERVO_SCAN_DELAY_MS  500
#define SERVO_PULSE_PER_30DEG 260.0f  /* (1800-1500) or (1500-1200) = 300 */
#define SERVO_DEG_PER_PULSE   (30.0f / SERVO_PULSE_PER_30DEG)  /* 0.1 deg per µs */

void set_servo_pulse(uint16_t us);

void init_servo(void);

float servo_pulse_to_angle(uint16_t pulse_us);

#endif