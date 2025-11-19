/**
 * @file    motors.h
 * @brief   Motor control interface
 * @author  Embedded Systems
 * @date    2025
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize motor PWM channels
 */
void motors_init(void);

/**
 * @brief Set motor speeds with duty cycle
 * @param[in] left_duty  Left motor duty cycle (-1.0 to 1.0)
 * @param[in] right_duty Right motor duty cycle (-1.0 to 1.0)
 */
void motors_set_speed(float left_duty, float right_duty);

/**
 * @brief Stop both motors immediately
 */
void motors_stop(void);

#endif /* MOTORS_H */