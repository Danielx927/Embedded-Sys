#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "config.h"

/*******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/

/**
 * @brief Calculate servo angle from pulse width
 */
float servo_pulse_to_angle(uint16_t pulse_us)
{
    int16_t diff = pulse_us - SERVO_CENTER;
    return (float)diff * SERVO_DEG_PER_PULSE;
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

/**
 * @brief Initialize ultrasonic sensor and servo
 */
void ultrasonic_init(void)
{
    // Initialize servo
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_wrap(slice, 20000);
    pwm_set_enabled(slice, true);
    
    ultrasonic_set_servo(SERVO_CENTER);
    sleep_ms(700);
    
    // Initialize ultrasonic
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);
    
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

/**
 * @brief Measure distance with ultrasonic sensor
 */
float ultrasonic_measure_cm(void)
{
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
    
    absolute_time_t start = get_absolute_time();
    while (!gpio_get(ECHO_PIN)) {
        if (absolute_time_diff_us(start, get_absolute_time()) > 30000)
            return -1;
    }
    
    absolute_time_t echo_start = get_absolute_time();
    while (gpio_get(ECHO_PIN)) {
        if (absolute_time_diff_us(echo_start, get_absolute_time()) > 30000)
            return -1;
    }
    absolute_time_t echo_end = get_absolute_time();
    
    int64_t pulse_us = absolute_time_diff_us(echo_start, echo_end);
    return pulse_us * 0.0343f / 2.0f;
}

/**
 * @brief Set servo pulse width
 */
void ultrasonic_set_servo(uint16_t pulse_us)
{
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    uint chan  = pwm_gpio_to_channel(SERVO_PIN);
    uint16_t level = (pulse_us * 20000) / 20000;
    pwm_set_chan_level(slice, chan, level);
}

/**
 * @brief Dynamic obstacle width scanning
 */
void ultrasonic_scan_obstacle(float *left_extent, float *right_extent,
                              float *left_angle, float *right_angle)
{
    printf("\n--- Dynamic Obstacle Scanning ---\n");
    
    // Initialize scan parameters
    uint16_t servo_left = SERVO_LEFT_START;
    uint16_t servo_right = SERVO_RIGHT_START;
    
    float last_valid_left_dist = -1.0f;
    float last_valid_right_dist = -1.0f;
    float last_valid_left_angle = 0.0f;
    float last_valid_right_angle = 0.0f;
    float prev_left_dist = -1.0f;
    float prev_right_dist = -1.0f;
    
    int scan_iteration = 0;
    
    // Scan LEFT side dynamically
    printf("\nScanning LEFT side:\n");
    bool first_left = true;
    while (servo_left <= SERVO_LEFT_MAX) {
        ultrasonic_set_servo(servo_left);
        sleep_ms(SERVO_SCAN_DELAY_MS);
        
        float dist = ultrasonic_measure_cm();
        float angle = servo_pulse_to_angle(servo_left);
        
        printf("  Pulse=%u, Angle=%.1f°, Distance=%.1f cm", servo_left, angle, dist);
        
        if (dist > 0) {
            if (first_left) {
                last_valid_left_dist = dist;
                last_valid_left_angle = angle;
                prev_left_dist = dist;
                first_left = false;
                printf(" ✓ Valid (first reading)\n");
                scan_iteration++;
            } else {
                float distance_jump = dist - prev_left_dist;
                if (distance_jump <= DISTANCE_JUMP_THRESHOLD) {
                    last_valid_left_dist = dist;
                    last_valid_left_angle = angle;
                    prev_left_dist = dist;
                    printf(" ✓ Valid (Δ=%.1f cm)\n", distance_jump);
                    scan_iteration++;
                } else {
                    printf(" ✗ Jump too large (Δ=%.1f cm > %.1f cm threshold) - stopping\n",
                           distance_jump, DISTANCE_JUMP_THRESHOLD);
                    break;
                }
            }
        } else {
            printf(" ✗ Invalid reading - stopping left scan\n");
            break;
        }
        
        servo_left += SERVO_STEP;
    }
    
    // Scan RIGHT side dynamically
    printf("\nScanning RIGHT side:\n");
    servo_right = SERVO_RIGHT_START;
    bool first_right = true;
    while (servo_right >= SERVO_RIGHT_MAX) {
        ultrasonic_set_servo(servo_right);
        sleep_ms(SERVO_SCAN_DELAY_MS);
        
        float dist = ultrasonic_measure_cm();
        float angle = servo_pulse_to_angle(servo_right);
        
        printf("  Pulse=%u, Angle=%.1f°, Distance=%.1f cm", servo_right, angle, dist);
        
        if (dist > 0) {
            if (first_right) {
                last_valid_right_dist = dist;
                last_valid_right_angle = angle;
                prev_right_dist = dist;
                first_right = false;
                printf(" ✓ Valid (first reading)\n");
                scan_iteration++;
            } else {
                float distance_jump = dist - prev_right_dist;
                if (distance_jump <= DISTANCE_JUMP_THRESHOLD) {
                    last_valid_right_dist = dist;
                    last_valid_right_angle = angle;
                    prev_right_dist = dist;
                    printf(" ✓ Valid (Δ=%.1f cm)\n", distance_jump);
                    scan_iteration++;
                } else {
                    printf(" ✗ Jump too large (Δ=%.1f cm > %.1f cm threshold) - stopping\n",
                           distance_jump, DISTANCE_JUMP_THRESHOLD);
                    break;
                }
            }
        } else {
            printf(" ✗ Invalid reading - stopping right scan\n");
            break;
        }
        
        servo_right -= SERVO_STEP;
    }
    
    // Return to center
    ultrasonic_set_servo(SERVO_CENTER);
    sleep_ms(500);
    
    // Calculate Lateral Extents using the last valid measurements
    if (last_valid_left_dist > 0) {
        float angle_rad = fabsf(last_valid_left_angle) * M_PI / 180.0f;
        *left_extent = last_valid_left_dist * sinf(angle_rad);
        *left_angle = last_valid_left_angle;
    } else {
        *left_extent = -1.0f;
        *left_angle = 0.0f;
    }
    
    if (last_valid_right_dist > 0) {
        float angle_rad = fabsf(last_valid_right_angle) * M_PI / 180.0f;
        *right_extent = last_valid_right_dist * sinf(angle_rad);
        *right_angle = last_valid_right_angle;
    } else {
        *right_extent = -1.0f;
        *right_angle = 0.0f;
    }
    
    printf("\n--- Scan Complete ---\n");
    printf("Total scan iterations: %d\n", scan_iteration);
    printf("Left:  Last valid at %.1f° with %.1f cm (clearance: %.1f cm)\n",
           last_valid_left_angle, last_valid_left_dist, *left_extent);
    printf("Right: Last valid at %.1f° with %.1f cm (clearance: %.1f cm)\n",
           last_valid_right_angle, last_valid_right_dist, *right_extent);
}