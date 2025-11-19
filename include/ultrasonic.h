/**
 * @file    ultrasonic.h
 * @brief   Ultrasonic sensor and servo scanning interface
 * @author  Embedded Systems
 * @date    2025
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "config.h"

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize ultrasonic sensor and servo
 */
void ultrasonic_init(void);

/**
 * @brief Measure distance with ultrasonic sensor
 * @return Distance in centimeters, -1 if error
 */
float ultrasonic_measure_cm(void);

/**
 * @brief Set servo pulse width
 * @param[in] pulse_us Pulse width in microseconds (1000-2000)
 */
void ultrasonic_set_servo(uint16_t pulse_us);

/**
 * @brief Scan obstacle width dynamically
 * @param[out] left_extent  Left lateral extent (cm)
 * @param[out] right_extent Right lateral extent (cm)
 * @param[out] left_angle   Final left scan angle (degrees)
 * @param[out] right_angle  Final right scan angle (degrees)
 */
void ultrasonic_scan_obstacle(float *left_extent, 
                             float *right_extent,
                             float *left_angle, 
                             float *right_angle);

#endif /* ULTRASONIC_H */