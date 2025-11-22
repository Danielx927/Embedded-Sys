#ifndef MULTICORE_OBSTACLE_H
#define MULTICORE_OBSTACLE_H

#include "config.h"
#include "pico/mutex.h"
#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/**
 * @brief Shared obstacle state structure (protected by mutex)
 */
typedef struct
{
    volatile bool obstacle_detected;
    volatile float obstacle_distance;
    volatile float obstacle_left_extent;
    volatile float obstacle_right_extent;
    volatile float obstacle_left_angle;
    volatile float obstacle_right_angle;
    volatile bool scan_complete;
    volatile bool obstacle_processed;
    volatile bool core0_stopped;
} obstacle_state_t;

/*******************************************************************************
 * GLOBAL VARIABLES (extern)
 ******************************************************************************/
extern mutex_t obstacle_mutex;
extern obstacle_state_t g_obstacle_state;
extern mutex_t obstacle_mutex;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize multicore obstacle detection system
 */
void multicore_obstacle_init(void);

/**
 * @brief Core 1 main loop - ultrasonic obstacle detection task
 */
void core1_obstacle_task(void);

/**
 * @brief Check if obstacle detected (thread-safe)
 * @param[out] distance Distance to obstacle (cm)
 * @return true if obstacle detected
 */
bool obstacle_check_detected(float *distance);

/**
 * @brief Wait for obstacle scan completion (thread-safe)
 * @param[out] left_extent  Left lateral extent (cm)
 * @param[out] right_extent Right lateral extent (cm)
 * @param[out] left_angle   Left scan angle (degrees)
 * @param[out] right_angle  Right scan angle (degrees)
 * @return true if scan complete
 */
bool obstacle_get_scan_results(float *left_extent, float *right_extent,
                               float *left_angle, float *right_angle);

/**
 * @brief Signal Core 0 has stopped (thread-safe)
 */
void obstacle_signal_core0_stopped(void);

/**
 * @brief Signal obstacle processing complete (thread-safe)
 */
void obstacle_signal_processed(void);

/**
 * @brief Reset obstacle flags after processing (thread-safe)
 */
void obstacle_reset(void);

#endif /* MULTICORE_OBSTACLE_H */
