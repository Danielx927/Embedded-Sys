#include "navigation.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "utils.h"
#include "config.h"

void navigation_init(pid_controller_t *speed_pid,
                    pid_controller_t *heading_pid,
                    pid_controller_t *line_pid) {
    speed_pid->kp = SPEED_KP;
    speed_pid->ki = SPEED_KI;
    speed_pid->kd = SPEED_KD;
    speed_pid->integral = 0.0f;
    speed_pid->last_error = 0.0f;
    
    heading_pid->kp = HEADING_KP_RAMP;
    heading_pid->ki = HEADING_KI_RAMP;
    heading_pid->kd = HEADING_KD_RAMP;
    heading_pid->integral = 0.0f;
    heading_pid->last_error = 0.0f;
    
    line_pid->kp = LINE_KP;
    line_pid->ki = LINE_KI;
    line_pid->kd = LINE_KD;
    line_pid->integral = 0.0f;
    line_pid->last_error = 0.0f;
}

float pid_update(pid_controller_t *pid, float error, float dt)
{
    float p_term = pid->kp * error;
    pid->integral += error * dt;
    /* Anti-windup: Clamp integral */
    if (pid->integral > INTEGRAL_LIMIT) pid->integral = INTEGRAL_LIMIT;
    if (pid->integral < -INTEGRAL_LIMIT) pid->integral = -INTEGRAL_LIMIT;
    float i_term = pid->ki * pid->integral;
    float d_term = pid->kd * (error - pid->last_error) / dt;
    pid->last_error = error;
    return p_term + i_term + d_term;
}

float angle_diff(float a, float b)
{
    float diff = fmodf(a - b + M_PI, 2.0f * M_PI) - M_PI;
    return diff;
}

bool_t execute_imu_controlled_turn(float current_heading, int turn_direction, float target_heading) {
    uint32_t last_heading_check = 0;
    uint32_t now = millis_now();
    
    // Only check heading every 50ms (not every loop iteration)
    if (now - last_heading_check < 50) {
        return false;  // Still turning, don't check yet
    }
    
    last_heading_check = now;
    
    // Calculate how close we are to target
    float heading_error = angle_diff(target_heading, current_heading);
    float abs_error = fabsf(heading_error);
    
    // Print progress every 50ms
    printf("   Turn progress: Current=%.2f rad, Target=%.2f rad, Error=%.3f rad (%.1f°)\n",
           current_heading, target_heading, heading_error, abs_error * 180.0f / M_PI);
    
    // Check if we've reached the target heading (within tolerance)
    if (abs_error < DETOUR_HEADING_TOLERANCE) {
        printf("✅ IMU turn complete. Target: %.2f rad, Actual: %.2f rad, Error: %.3f rad\n", 
               target_heading, current_heading, heading_error);
        last_heading_check = 0;  // Reset for next turn
        return true;
    }
    
    return false;  // Still turning
}