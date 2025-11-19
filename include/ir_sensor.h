/**
 * @file    ir_sensor.h
 * @brief   IR line sensor interface
 * @author  Embedded Systems
 * @date    2025
 */

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "config.h"

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize line sensor ADC
 */
void ir_sensor_init(void);

/**
 * @brief Read and filter line sensor value
 * @return Filtered ADC value
 */
uint16_t ir_sensor_read_filtered(void);

/**
 * @brief Compute line error from sensor reading
 * @param[in] sensor_value Raw or filtered sensor value
 * @return Normalized error
 */
float ir_sensor_compute_error(uint16_t sensor_value);

#endif /* LINE_SENSOR_H */

