#ifndef IMU_H
#define IMU_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/**
 * @brief Raw sensor data structure
 */
typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} sensor_data_t;

/**
 * @brief Moving average filter structure
 */
typedef struct
{
    int32_t accel_buffer[NUM_AXES][FILTER_WINDOW_SIZE];
    int32_t mag_buffer[NUM_AXES][FILTER_WINDOW_SIZE];
    uint8_t buffer_index;
    bool_t  buffer_full;
} moving_avg_filter_t;

/**
 * @brief Filtered sensor data structure
 */
typedef struct
{
    int32_t accel_x;
    int32_t accel_y;
    int32_t accel_z;
    int32_t mag_x;
    int32_t mag_y;
    int32_t mag_z;
} filtered_data_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

sensor_data_t g_raw_data;
moving_avg_filter_t g_filter;
filtered_data_t g_filtered_data;
/**
 * @brief Initialize IMU sensors (accelerometer and magnetometer)
 */
void imu_init(void);

void accel_init(void);
void mag_init(void);

/**
 * @brief Read raw accelerometer data
 * @param[out] p_data Pointer to store raw sensor data
 */
void imu_read_accel(sensor_data_t *p_data);

/**
 * @brief Read raw magnetometer data
 * @param[out] p_data Pointer to store raw sensor data
 */
void imu_read_mag(sensor_data_t *p_data);

/**
 * @brief Initialize moving average filter
 * @param[out] p_filter Pointer to filter structure
 */
void imu_filter_init(moving_avg_filter_t *p_filter);

/**
 * @brief Update moving average filter with new raw data
 * @param[in,out] p_filter Pointer to filter structure
 * @param[in]     p_raw    Pointer to raw sensor data
 */
void imu_filter_update(moving_avg_filter_t const *p_filter, sensor_data_t const * const p_raw);

/**
 * @brief Compute average from moving average filter
 * @param[in]  p_filter   Pointer to filter structure
 * @param[out] p_filtered Pointer to filtered data structure
 */
void imu_filter_average(const moving_avg_filter_t *p_filter, filtered_data_t *p_filtered);

/**
 * @brief Compute heading from magnetometer data
 * @param[in] p_data Pointer to filtered data
 * @return Heading in radians
 */
void imu_compute_heading(filtered_data_t const * const p_data, float *heading);

/**
 * @brief Calibrate initial heading
 * @param[in,out] p_filter   Pointer to filter structure
 * @param[out]    p_raw_data Pointer to raw data buffer
 * @return Calibrated heading in radians
 */
float imu_calibrate_heading(moving_avg_filter_t *p_filter, sensor_data_t *p_raw_data);

#endif /* IMU_H */