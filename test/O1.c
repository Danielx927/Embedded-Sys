/*
 * Test ID: O1
 * Description: Ultrasonic distance sanity at key setpoints.
 * This test verifies ultrasonic sensor accuracy at 100, 200, 300, 500 mm by averaging 20 samples per position.
 * Adapted to codebase: Uses ultrasonic_init(), ultrasonic_measure_cm(). Manual target placement required.
 * User: Place flat target perpendicular at prompted distance, wait 5s for positioning.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic.h"
#include "config.h"
#include <math.h>

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Initialize
    ultrasonic_init();

    printf("O1 Test: Ultrasonic Distance Sanity - Starting\n");
    printf("Place flat perpendicular target at prompted distances.\n");

    float setpoints[] = {100.0f, 200.0f, 300.0f, 500.0f};
    int num_setpoints = 4;
    bool all_pass = true;

    for (int sp = 0; sp < num_setpoints; sp++) {
        float expected = setpoints[sp];
        printf("\n--- Setpoint %.0f mm ---\n", expected);
        printf("Place target at %.0f mm from sensor. Wait 5s...\n", expected);
        sleep_ms(5000);  // Positioning time

        float samples[20];
        float sum = 0.0f;
        for (int i = 0; i < 20; i++) {
            samples[i] = ultrasonic_measure_cm();
            if (samples[i] > 0) sum += samples[i];
            sleep_ms(50);
        }

        // Filter invalid (>0)
        int valid_count = 0;
        float valid_sum = 0.0f;
        for (int i = 0; i < 20; i++) {
            if (samples[i] > 0) {
                valid_sum += samples[i];
                valid_count++;
            }
        }
        if (valid_count == 0) {
            printf("FAIL: No valid readings at %.0f mm\n", expected);
            all_pass = false;
            continue;
        }

        float mean = valid_sum / (float)valid_count;
        float variance = 0.0f;
        for (int i = 0; i < 20; i++) {
            if (samples[i] > 0) {
                float diff = samples[i] - mean;
                variance += diff * diff;
            }
        }
        float std_dev = sqrtf(variance / (float)valid_count);
        float max_abs_error = 0.0f;
        int outlier_count = 0;
        float three_sigma = 3.0f * std_dev;
        for (int i = 0; i < 20; i++) {
            if (samples[i] > 0) {
                float abs_err = fabsf(samples[i] - expected);
                if (abs_err > max_abs_error) max_abs_error = abs_err;
                if (fabsf(samples[i] - mean) > three_sigma) outlier_count++;
            }
        }

        // Pass criteria
        float max_mean_error = (expected <= 300.0f) ? 10.0f : 20.0f;
        bool mean_ok = (fabsf(mean - expected) <= max_mean_error);
        bool sigma_ok = (std_dev <= 8.0f);
        bool no_outliers = (outlier_count == 0);

        bool pass = mean_ok && sigma_ok && no_outliers;
        all_pass = all_pass && pass;

        if (pass) {
            printf("PASS at %.0f mm\n", expected);
        } else {
            printf("FAIL at %.0f mm\n", expected);
        }
        printf("- Mean: %.1f mm (error: %.1f mm)\n", mean, fabsf(mean - expected));
        printf("- Std Dev: %.1f mm\n", std_dev);
        printf("- Max Abs Error: %.1f mm\n", max_abs_error);
        printf("- Outliers (>3σ): %d\n", outlier_count);

        // Print samples
        printf("- Samples: ");
        for (int i = 0; i < 20; i++) {
            printf("%.0f ", samples[i]);
        }
        printf("\n");
    }

    // Overall
    printf("\nO1 Test: %s\n", all_pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return all_pass ? 0 : 1;
