#include "multicore_obstacle.h"
#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>
#include <stdbool.h>
#include "config.h"

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
mutex_t obstacle_mutex;
obstacle_state_t g_obstacle_state = {
    .obstacle_detected = false,
    .obstacle_distance = -1.0f,
    .obstacle_left_extent = 0.0f,
    .obstacle_right_extent = 0.0f,
    .obstacle_left_angle = 0.0f,
    .obstacle_right_angle = 0.0f,
    .scan_complete = false,
    .obstacle_processed = false,
    .core0_stopped = false
};

/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

/**
 * @brief Initialize multicore obstacle detection system
 */
void multicore_obstacle_init(void)
{
    mutex_init(&obstacle_mutex);
    
    // Initialize obstacle state
    mutex_enter_blocking(&obstacle_mutex);
    g_obstacle_state.obstacle_detected = false;
    g_obstacle_state.scan_complete = false;
    g_obstacle_state.obstacle_processed = false;
    g_obstacle_state.core0_stopped = false;
    mutex_exit(&obstacle_mutex);
    
    printf("Launching Core 1 for ultrasonic task...\n");
    multicore_launch_core1(core1_obstacle_task);
    sleep_ms(500);  // Give Core 1 time to initialize
}

/**
 * @brief Core 1 main loop - handles obstacle detection independently
 */
void core1_obstacle_task(void)
{
    printf("Core 1: Ultrasonic task started\n");
    
    // Initialize servo and ultrasonic on Core 1
    ultrasonic_init();
    
    // Set servo to center initially
    ultrasonic_set_servo(SERVO_CENTER);
    sleep_ms(500);
    
    while (true) {
        // Check center distance first (fast check)
        ultrasonic_set_servo(SERVO_CENTER);
        sleep_ms(50);  // Reduced from 100ms for faster response
        
        float dist_center = ultrasonic_measure_cm();
        
        // Only do full scan if obstacle detected
        if (dist_center > 0 && dist_center < OBSTACLE_THRESHOLD) {
            // Take mutex to update shared state
            mutex_enter_blocking(&obstacle_mutex);
            
            g_obstacle_state.obstacle_distance = dist_center;
            g_obstacle_state.obstacle_detected = true;
            g_obstacle_state.scan_complete = false;
            g_obstacle_state.core0_stopped = false;
            
            mutex_exit(&obstacle_mutex);
            
            printf("Core 1: Obstacle at %.1f cm - waiting for Core 0 to stop\n", dist_center);
            
            // Wait for Core 0 to acknowledge and stop the robot
            while (true) {
                mutex_enter_blocking(&obstacle_mutex);
                bool stopped = g_obstacle_state.core0_stopped;
                mutex_exit(&obstacle_mutex);
                
                if (stopped) {
                    break;
                }
                sleep_ms(10);
            }
            
            printf("Core 1: Core 0 stopped, starting scan\n");
            
            // Perform detailed scan
            float left_ext, right_ext, left_ang, right_ang;
            ultrasonic_scan_obstacle(&left_ext, &right_ext, &left_ang, &right_ang);
            
            // Update shared state with scan results
            mutex_enter_blocking(&obstacle_mutex);
            
            g_obstacle_state.obstacle_left_extent = left_ext;
            g_obstacle_state.obstacle_right_extent = right_ext;
            g_obstacle_state.obstacle_left_angle = left_ang;
            g_obstacle_state.obstacle_right_angle = right_ang;
            g_obstacle_state.scan_complete = true;
            
            mutex_exit(&obstacle_mutex);
            
            printf("Core 1: Scan complete - Left: %.1fcm, Right: %.1fcm\n", 
                   left_ext, right_ext);
            
            // Wait for Core 0 to process this obstacle before scanning again
            while (true) {
                mutex_enter_blocking(&obstacle_mutex);
                bool processed = g_obstacle_state.obstacle_processed;
                mutex_exit(&obstacle_mutex);
                
                if (processed) {
                    break;
                }
                sleep_ms(50);
            }
            
            // Reset for next obstacle
            mutex_enter_blocking(&obstacle_mutex);
            g_obstacle_state.obstacle_detected = false;
            g_obstacle_state.obstacle_processed = false;
            g_obstacle_state.core0_stopped = false;
            mutex_exit(&obstacle_mutex);
            
        } else {
            // No obstacle - sleep longer to reduce overhead
            sleep_ms(100);
        }
        
        // Return servo to center between scans
        ultrasonic_set_servo(SERVO_CENTER);
    }
}

/**
 * @brief Check if obstacle detected (thread-safe)
 */
bool obstacle_check_detected(float *distance)
{
    mutex_enter_blocking(&obstacle_mutex);
    bool detected = g_obstacle_state.obstacle_detected;
    if (distance) {
        *distance = g_obstacle_state.obstacle_distance;
    }
    mutex_exit(&obstacle_mutex);
    
    return detected;
}

/**
 * @brief Wait for obstacle scan completion (thread-safe)
 */
bool obstacle_get_scan_results(float *left_extent, float *right_extent,
                               float *left_angle, float *right_angle)
{
    mutex_enter_blocking(&obstacle_mutex);
    bool complete = g_obstacle_state.scan_complete;
    
    if (complete) {
        if (left_extent) *left_extent = g_obstacle_state.obstacle_left_extent;
        if (right_extent) *right_extent = g_obstacle_state.obstacle_right_extent;
        if (left_angle) *left_angle = g_obstacle_state.obstacle_left_angle;
        if (right_angle) *right_angle = g_obstacle_state.obstacle_right_angle;
    }
    
    mutex_exit(&obstacle_mutex);
    
    return complete;
}

/**
 * @brief Signal Core 0 has stopped (thread-safe)
 */
void obstacle_signal_core0_stopped(void)
{
    mutex_enter_blocking(&obstacle_mutex);
    g_obstacle_state.core0_stopped = true;
    mutex_exit(&obstacle_mutex);
}

/**
 * @brief Signal obstacle processing complete (thread-safe)
 */
void obstacle_signal_processed(void)
{
    mutex_enter_blocking(&obstacle_mutex);
    g_obstacle_state.obstacle_processed = true;
    mutex_exit(&obstacle_mutex);
}

/**
 * @brief Reset obstacle flags after processing (thread-safe)
 */
void obstacle_reset(void)
{
    mutex_enter_blocking(&obstacle_mutex);
    g_obstacle_state.obstacle_detected = false;
    g_obstacle_state.scan_complete = false;
    g_obstacle_state.obstacle_processed = true;
    g_obstacle_state.core0_stopped = false;
    mutex_exit(&obstacle_mutex);
}