#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "config.h"
#include <stdint.h>

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize line sensor ADC
 */
void line_sensor_init(void);

/**
 * @brief Read and filter line sensor value
 * @return Filtered ADC value
 */
uint16_t line_sensor_read_filtered(void);

/**
 * @brief Compute line error from sensor reading
 * @param[in] sensor_value Raw or filtered sensor value
 * @return Normalized error
 */
float line_sensor_compute_error(uint16_t sensor_value);

#endif /* LINE_SENSOR_H */