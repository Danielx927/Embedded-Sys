/**
 * @file    navigation.h
 * @brief   Navigation state machine and control algorithms
 * @author  Embedded Systems
 * @date    2025
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "config.h"

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/**
 * @brief PID controller structure
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
} pid_controller_t;

/**
 * @brief Robot state enum
 */
typedef enum
{
    STATE_FOLLOWING,
    STATE_TURNING,
    STATE_RECOVERY,
    STATE_OBSTACLE_DETECTED,
    STATE_OBSTACLE_STOPPED,
    STATE_DETOUR_TURN_PERPENDICULAR,
    STATE_DETOUR_MOVE_SIDEWAYS,
    STATE_DETOUR_TURN_PARALLEL,
    STATE_DETOUR_MOVE_FORWARD,
    STATE_DETOUR_TURN_BACK,
    STATE_DETOUR_REJOIN,
    STATE_DETOUR_STABILIZE
} robot_state_t;

/**
 * @brief Turn direction enum
 */
typedef enum
{
    TURN_NONE,
    TURN_LEFT,
    TURN_RIGHT
} turn_dir_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Update PID controller
 * @param[in,out] pid   Pointer to PID structure
 * @param[in]     error Current error
 * @param[in]     dt    Delta time (seconds)
 * @return Control output
 */
float pid_update(pid_controller_t *pid, float error, float dt);

/**
 * @brief Compute smallest angle difference
 * @param[in] a Angle A (radians)
 * @param[in] b Angle B (radians)
 * @return Difference (-pi to pi)
 */
float angle_diff(float a, float b);

/**
 * @brief Execute IMU-controlled turn
 * @param[in] current_heading Current heading (radians)
 * @param[in] turn_direction  Turn direction (-1 = left, +1 = right)
 * @param[in] target_heading  Target heading (radians)
 * @return true if turn complete, false if still turning
 */
bool_t execute_imu_controlled_turn(float current_heading, 
                                   int turn_direction, 
                                   float target_heading);

/**
 * @brief Initialize navigation system
 * @param[out] speed_pid   Speed PID controller
 * @param[out] heading_pid Heading PID controller
 * @param[out] line_pid    Line PID controller
 */
void navigation_init(pid_controller_t *speed_pid,
                    pid_controller_t *heading_pid,
                    pid_controller_t *line_pid);

#endif /* NAVIGATION_H */