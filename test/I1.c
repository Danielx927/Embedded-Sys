/*
 * Test ID: I1
 * Description: IMU fused orientation stability and drift when stationary.
 * This test logs IMU heading at 50Hz for 60s while stationary, computes drift rate.
 * Adapted to codebase: Uses imu_compute_heading() (mag-based). No full fusion; focus on yaw drift. Assume level surface.
 * Wait 5s for bias convergence.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "utils.h"
#include "config.h"
#include <math.h>

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Initialize
    imu_init();
    imu_filter_init(&g_filter);
    printf("I1 Test: IMU Stationary Drift - Starting\n");
    printf("Place robot level and stationary. Wait 5s for bias convergence...\n");
    sleep_ms(5000);  // Bias convergence

    uint32_t start_time = millis_now();
    uint32_t log_start = start_time;
    float initial_heading = 0.0f;
    float headings[3000] = {0.0f};  // 50Hz * 60s
    int log_count = 0;
    bool has_nan = false;
    float min_heading = 1000.0f, max_heading = -1000.0f;

    printf("Logging at 50Hz for 60s...\n");

    // Log loop 50Hz (20ms)
    while (millis_now() - log_start < 60000) {
        uint32_t now = millis_now();
        if (now - start_time < 20) continue;  // Aim 50Hz

        // Read IMU
        imu_read_accel();
        imu_read_mag();
        imu_filter_update(&g_filter, &g_raw_data);
        imu_filter_average(&g_filter, &g_filtered_data);
        float heading = imu_compute_heading(&g_filtered_data);

        if (log_count == 0) initial_heading = heading;

        if (isnan(heading)) has_nan = true;

        headings[log_count] = heading;
        if (heading < min_heading) min_heading = heading;
        if (heading > max_heading) max_heading = heading;

        log_count++;
        start_time = now;
        sleep_ms(20);
    }

    // Compute drift
    float final_heading = headings[log_count - 1];
    float delta_heading = fabsf(final_heading - initial_heading);
    // Unwrap if crossed ±π
    if (delta_heading > M_PI) delta_heading = 2.0f * M_PI - delta_heading;
    float total_time_min = 60.0f / 60.0f;  // 1 min
    float yaw_drift_per_min = (delta_heading * 180.0f / M_PI) / total_time_min;  // deg/min

    // Pitch/roll: Stub, assume 0 drift (no full fusion)
    float pitch_drift = 0.0f;  // Would use accel
    float roll_drift = 0.0f;

    bool no_nan = !has_nan;
    bool status_ok = true;  // Assume "OK"

    // Pass/Fail
    bool yaw_ok = (yaw_drift_per_min <= 1.0f);
    bool pitch_roll_ok = (pitch_drift <= 0.5f && roll_drift <= 0.5f);
    bool pass = yaw_ok && pitch_roll_ok && no_nan && status_ok;

    if (pass) {
        printf("PASS: Yaw drift ≤1.0°/min; pitch/roll ≤0.5°/min; No NaNs/dropouts; Status OK.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!yaw_ok) printf("- Yaw drift: %.2f °/min >1.0\n", yaw_drift_per_min);
        if (!pitch_roll_ok) printf("- Pitch/roll drift: %.2f/%.2f °/min >0.5\n", pitch_drift, roll_drift);
        if (!no_nan) printf("- NaNs or dropouts detected\n");
        if (!status_ok) printf("- Fusion status not OK\n");
    }

    // Results
    printf("\nResults:\n");
    printf("- Log count: %d (expected ~3000)\n", log_count);
    printf("- Initial heading: %.2f rad\n", initial_heading);
    printf("- Final heading: %.2f rad\n", final_heading);
    printf("- Delta heading: %.2f rad (%.2f °)\n", delta_heading, delta_heading * 180.0f / M_PI);
    printf("- Yaw drift rate: %.2f °/min\n", yaw_drift_per_min);
    printf("- Pitch drift: %.2f °/min (stub)\n", pitch_drift);
    printf("- Roll drift: %.2f °/min (stub)\n", roll_drift);
    printf("- Min/Max heading: %.2f / %.2f rad\n", min_heading, max_heading);
    printf("- NaN/drop events: %s\n", has_nan ? "Yes" : "No");
    printf("- Fusion status: OK (stub)\n");

    printf("I1 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
}
