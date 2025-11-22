/*
 * Test ID: L1
 * Description: IR line sensor calibration; error sign/zero-crossing validated.
 * This test calibrates the IR line sensor over white/black patches and validates error computation across lateral positions.
 * Adapted to codebase: Uses line_sensor_read_filtered() for readings, computes baseline/gain, lateral error. Manual positioning required for offsets.
 * User must position robot laterally at each prompted offset (-30 to +30 mm) and wait 5s for reading.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "line_sensor.h"
#include "config.h"
#include <math.h>

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Initialize
    line_sensor_init();

    printf("L1 Test: IR Line Sensor Calibration - Starting\n");
    printf("Place robot on calibration board. Follow prompts for positioning.\n");

    // Step 1: Calibrate baseline over white and black
    printf("\n--- White Patch Calibration ---\n");
    printf("Position sensor over white patch. Wait 5s...\n");
    sleep_ms(5000);
    uint16_t white_raw = 0;
    for (int i = 0; i < 10; i++) {
        white_raw += line_sensor_read_filtered();
        sleep_ms(100);
    }
    white_raw /= 10;
    printf("White baseline: %u\n", white_raw);

    printf("\n--- Black Patch Calibration ---\n");
    printf("Position sensor over black patch. Wait 5s...\n");
    sleep_ms(5000);
    uint16_t black_raw = 0;
    for (int i = 0; i < 10; i++) {
        black_raw += line_sensor_read_filtered();
        sleep_ms(100);
    }
    black_raw /= 10;
    printf("Black baseline: %u\n", black_raw);

    if (white_raw <= black_raw) {
        printf("FAIL: White <= Black baseline (inverted?)\n");
        return 1;
    }

    float gain = 4095.0f / (white_raw - black_raw);  // Normalize to full scale
    printf("Gain: %.2f\n", gain);

    // Step 2: Lateral positions -30mm to +30mm step 5mm (13 positions)
    #define NUM_POS 13
    #define POSITIONS_MM -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30
    int positions[NUM_POS] = {POSITIONS_MM};
    uint16_t raw_values[NUM_POS] = {0};
    float calibrated[NUM_POS] = {0.0f};
    float errors[NUM_POS] = {0.0f};

    printf("\n--- Lateral Error Validation ---\n");
    for (int i = 0; i < NUM_POS; i++) {
        printf("Position sensor at %d mm offset from line center. Wait 5s for reading...\n", positions[i]);
        sleep_ms(5000);  // User positioning time

        uint16_t sum = 0;
        for (int j = 0; j < 10; j++) {
            sum += line_sensor_read_filtered();
            sleep_ms(100);
        }
        raw_values[i] = sum / 10;
        calibrated[i] = (raw_values[i] - black_raw) * gain;  // 0-4095 normalized
        errors[i] = line_sensor_compute_error(raw_values[i]);  // Use existing error func

        printf("Offset %d mm: Raw=%u, Calibrated=%.1f, Error=%.2f\n", positions[i], raw_values[i], calibrated[i], errors[i]);
    }

    // Step 3: Re-check endpoints
    printf("\n--- Endpoint Re-check ---\n");
    printf("Re-position over white patch. Wait 5s...\n");
    sleep_ms(5000);
    uint16_t white_recheck = 0;
    for (int i = 0; i < 10; i++) {
        white_recheck += line_sensor_read_filtered();
        sleep_ms(100);
    }
    white_recheck /= 10;

    printf("Re-position over black patch. Wait 5s...\n");
    sleep_ms(5000);
    uint16_t black_recheck = 0;
    for (int i = 0; i < 10; i++) {
        black_recheck += line_sensor_read_filtered();
        sleep_ms(100);
    }
    black_recheck /= 10;

    bool no_saturation = (white_recheck >= 0.95f * 4095 && black_recheck <= 0.05f * 4095);  // Near full scale

    // Compute zero-crossing and slope (linear fit: error vs position)
    // Simple two-point slope: left (-15mm), right (+15mm); zero = -intercept
    int left_idx = 3;  // -15mm
    int right_idx = 9; // +15mm
    float slope = (errors[right_idx] - errors[left_idx]) / (30.0f);  // mm
    float zero_crossing = -errors[left_idx] / slope + positions[left_idx];  // mm

    // Range: max calibrated - min / 4095 >= 0.8
    float max_cal = 0, min_cal = 4095;
    for (int i = 0; i < NUM_POS; i++) {
        if (calibrated[i] > max_cal) max_cal = calibrated[i];
        if (calibrated[i] < min_cal) min_cal = calibrated[i];
    }
    bool range_ok = ((max_cal - min_cal) / 4095.0f >= 0.8f);

    // Pass/Fail
    bool zero_ok = (fabsf(zero_crossing) <= 3.0f);
    bool slope_sign_ok = (slope > 0.0f);  // Positive error turns right (assume convention)
    bool pass = zero_ok && slope_sign_ok && no_saturation && range_ok;

    if (pass) {
        printf("PASS: Zero-crossing within ±3 mm; Error slope positive; No saturation; Calibrated range ≥80%%.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!zero_ok) printf("- Zero-crossing: %.1f mm (>±3 mm)\n", zero_crossing);
        if (!slope_sign_ok) printf("- Slope sign incorrect: %.3f\n", slope);
        if (!no_saturation) printf("- Saturation at endpoints (White:%u, Black:%u)\n", white_recheck, black_recheck);
        if (!range_ok) printf("- Range: %.1f%% <80%%\n", (max_cal - min_cal) / 4095.0f * 100);
    }

    // Results
    printf("\nResults:\n");
    printf("- Baselines: White=%u, Black=%u\n", white_raw, black_raw);
    printf("- Endpoints recheck: White=%u, Black=%u\n", white_recheck, black_recheck);
    printf("- Zero-crossing: %.1f mm\n", zero_crossing);
    printf("- Slope: %.3f (error per mm)\n", slope);
    printf("- Calibrated range: %.1f%% of full scale\n", (max_cal - min_cal) / 4095.0f * 100);
    for (int i = 0; i < NUM_POS; i++) {
        printf("- Pos %d mm: Raw=%u, Cal=%.1f, Err=%.2f\n", positions[i], raw_values[i], calibrated[i], errors[i]);
    }

    printf("L1 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
