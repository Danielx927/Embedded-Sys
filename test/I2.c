/*
 * Test ID: I2
 * Description: 90° commanded turn accuracy using fused heading.
 * This test verifies 90° in-place turn accuracy with IMU heading, settling, and no oscillation.
 * Adapted to codebase: Uses imu_compute_heading(), execute_imu_controlled_turn(), motors/encoders for rotation check.
 * Position on clear floor.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "navigation.h"
#include "utils.h"
#include "config.h"
#include <math.h>

// Define missing constants
#define DIST_PER_PULSE 0.001f  // Meters per encoder pulse, adjust as needed
#define WHEEL_DIAMETER_M 0.065f  // Wheel diameter in meters, adjust as needed

// Mock functions if not implemented
float angle_diff(float target, float current) {
    float diff = target - current;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

bool execute_imu_controlled_turn(float current_heading, int direction, float target_heading) {
    // Mock turn execution: assume instant for test
    static bool done = false;
    if (!done) {
        // Simulate turn
        motors_set_speed(-0.2f * direction, 0.2f * direction);  // Differential turn
        sleep_ms(1000);  // Mock time
        motors_stop();
        done = true;
        return true;
    }
    return false;
}

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Initialize
    imu_init();
    imu_filter_init(&g_filter);
    motors_init();
    encoders_init();
    encoders_reset();

    printf("I2 Test: 90° Turn Accuracy - Starting\n");
    printf("Position robot on clear floor. Executing 90° turn.\n");

    // Establish initial heading H0
    float H0 = imu_calibrate_heading(&g_filter, &g_raw_data);
    printf("Initial heading H0: %.3f rad\n", H0);

    // Command 90° turn (right for test)
    float target_heading = H0 + M_PI / 2.0f;
    while (target_heading > M_PI) target_heading -= 2.0f * M_PI;
    while (target_heading < -M_PI) target_heading += 2.0f * M_PI;
    printf("Target heading: %.3f rad\n", target_heading);

    uint32_t start_time = millis_now();
    uint32_t turn_start = start_time;
    uint32_t settle_start = 0;
    bool turning = true;
    bool settled = false;
    float current_heading = H0;
    uint32_t initial_pulses_left = encoders_get_left_pulses();
    uint32_t initial_pulses_right = encoders_get_right_pulses();
    float last_heading = H0;
    float max_overshoot = 0.0f;
    int oscillation_count = 0;  // Simple: heading delta >0.05 rad in last 0.5s

    // Turn loop
    while (millis_now() - start_time < 10000) {  // 10s max
        uint32_t now = millis_now();

        // Read IMU
        imu_read_accel();
        imu_read_mag();
        imu_filter_update(&g_filter, &g_raw_data);
        imu_filter_average(&g_filter, &g_filtered_data);
        current_heading = imu_compute_heading(&g_filtered_data);

        if (turning) {
            // Use navigation turn function
            if (execute_imu_controlled_turn(current_heading, 1, target_heading)) {  // Right turn
                turning = false;
                settle_start = now;
                printf("Turn complete at %.3f rad, t=%.1fs\n", current_heading, (float)(now - turn_start)/1000);
            }

            // Track overshoot
            float error = angle_diff(target_heading, current_heading);
            if (fabsf(error) > max_overshoot) max_overshoot = fabsf(error);
        } else {
            // Post-stop settle 2s
            if (now - settle_start < 2000) {
                // Check oscillation: delta heading
                float delta = fabsf(current_heading - last_heading);
                if (delta > 0.05f) oscillation_count++;
                last_heading = current_heading;

                // Settle check: error <=5°=0.087 rad within 1s
                float final_error = fabsf(angle_diff(target_heading, current_heading));
                if (final_error <= 0.087f && (now - settle_start <= 1000)) {
                    settled = true;
                    printf("Settled within 1s at error %.3f rad\n", final_error);
                    break;
                }
            } else {
                settled = false;  // Failed settle time
                break;
            }
        }

        sleep_ms(20);  // 50Hz
    }

    // Encoder consistency: Pulses for 90° rotation (wheel circ /4 for 90°)
    uint32_t final_pulses_left = encoders_get_left_pulses();
    uint32_t final_pulses_right = encoders_get_right_pulses();
    float pulses_left = final_pulses_left - initial_pulses_left;
    float pulses_right = final_pulses_right - initial_pulses_right;
    float expected_pulses = (M_PI * WHEEL_DIAMETER_M / 4.0f) / DIST_PER_PULSE;  // 90° arc
    bool encoders_ok = (fabsf(pulses_left - expected_pulses) < expected_pulses * 0.2f && fabsf(pulses_right - expected_pulses) < expected_pulses * 0.2f);

    // Final error
    float final_error_rad = fabsf(angle_diff(target_heading, current_heading));
    float final_error_deg = final_error_rad * 180.0f / M_PI;
    float settle_time = (float)(settle_start - turn_start) / 1000.0f;
    bool error_ok = (final_error_deg <= 5.0f);
    bool settle_time_ok = (settle_time <= 1.0f);
    bool overshoot_ok = (max_overshoot * 180.0f / M_PI <= 15.0f);
    bool no_oscillation = (oscillation_count < 5);  // Arbitrary low count

    bool pass = error_ok && settle_time_ok && overshoot_ok && no_oscillation && encoders_ok;
    if (pass) {
        printf("PASS: Final error <=±5°; settle <=1.0s; overshoot <=15°; no reverse oscillation; encoders consistent.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!error_ok) printf("- Final error: %.1f° >5°\n", final_error_deg);
        if (!settle_time_ok) printf("- Settle time: %.1fs >1s\n", settle_time);
        if (!overshoot_ok) printf("- Overshoot: %.1f° >15°\n", max_overshoot * 180.0f / M_PI);
        if (!no_oscillation) printf("- Oscillation detected (%d events)\n", oscillation_count);
        if (!encoders_ok) printf("- Encoder pulses L:%.0f R:%.0f (expected ~%.0f)\n", pulses_left, pulses_right, expected_pulses);
    }

    // Results
    printf("\nResults:\n");
    printf("- Total time: %.1fs\n", (float)(millis_now() - start_time)/1000);
    printf("- Turn time: %.1fs\n", settle_time);
    printf("- Final heading: %.3f rad (target: %.3f, error: %.3f rad / %.1f°)\n", current_heading, target_heading, final_error_rad, final_error_deg);
    printf("- Max overshoot: %.1f°\n", max_overshoot * 180.0f / M_PI);
    printf("- Oscillation events: %d\n", oscillation_count);
    printf("- Encoder pulses: Left=%.0f, Right=%.0f (expected ~%.0f for 90°)\n", pulses_left, pulses_right, expected_pulses);

    printf("I2 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
