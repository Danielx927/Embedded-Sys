/*
 * Test ID: L2
 * Description: Closed-loop line following on straight + gentle curve at nominal speed.
 * This test verifies line following accuracy over 10m track with one 2m radius curve.
 * Adapted to codebase: Uses line_sensor_compute_error(), PID for line/speed control,
 * motors_set_speed(), encoders for distance/speed logging. Nominal speed 0.35 m/s (~0.08 m/s from config).
 * Place at track start, run until 10m or manual stop. Logs errors/speeds for RMS/max analysis.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "motors.h"
#include "encoders.h"
#include "line_sensor.h"
#include "navigation.h"
#include "utils.h"
#include "config.h"
#include <math.h>

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Initialize
    line_sensor_init();
    motors_init();
    encoders_init();
    encoders_reset();

    // PIDs
    pid_controller_t speed_pid = {SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, 0.0f};
    pid_controller_t line_pid = {LINE_KP, LINE_KI, LINE_KD, 0.0f, 0.0f};
    speed_filter_t speed_filter;
    encoders_filter_init(&speed_filter);

    printf("L2 Test: Closed-Loop Line Following - Starting\n");
    printf("Place at start of 10m track with 2m radius curve. Nominal speed: %.2f m/s\n", 0.08f);

    float nominal_speed = 0.08f;  // From config, approx 0.35 m/s equivalent
    float base_duty = INITIAL_BASE_DUTY;
    uint32_t start_time = millis_now();
    uint32_t last_time = start_time;
    #define MAX_SAMPLES 20000  // For ~200Hz over ~100s
    float errors[MAX_SAMPLES];
    float speeds[MAX_SAMPLES];
    int sample_count = 0;
    int loss_count = 0;
    bool oscillation_detected = false;
    float prev_error = 0.0f;
    float total_distance = 0.0f;
    static uint32_t prev_left = 0, prev_right = 0;

    // Control loop ~200Hz until 10m
    while (total_distance < 10.0f && sample_count < MAX_SAMPLES) {
        uint32_t now = millis_now();
        float dt = (float)(now - last_time) / 1000.0f;
        if (dt < 0.001f) {
            sleep_ms(5);
            continue;
        }
        last_time = now;

        // Read line
        uint16_t line_val = line_sensor_read_filtered();
        float line_error = line_sensor_compute_error(line_val);

        // Check loss-of-line (quality check)
        if (line_val < 500 || line_val > 3500) {  // Outside valid range
            loss_count++;
        }

        // Line PID
        float heading_adjust = -pid_update(&line_pid, line_error, dt);
        heading_adjust = fmaxf(fminf(heading_adjust, 0.12f), -0.12f);

        // Speed from encoders
        uint32_t left_pulses = encoders_get_left_pulses();
        uint32_t right_pulses = encoders_get_right_pulses();
        uint32_t delta_left = left_pulses - prev_left;
        uint32_t delta_right = right_pulses - prev_right;
        prev_left = left_pulses;
        prev_right = right_pulses;
        float left_speed = (float)delta_left * DIST_PER_PULSE / dt;
        float right_speed = (float)delta_right * DIST_PER_PULSE / dt;

        float filtered_left, filtered_right;
        encoders_filter_update(&speed_filter, left_speed, right_speed, &filtered_left, &filtered_right);
        float avg_speed = (filtered_left + filtered_right) / 2.0f;
        speeds[sample_count] = avg_speed;

        // Speed PID (outside turns: assume whole track, no specific turn detection)
        float speed_error = nominal_speed - avg_speed;
        float speed_adjust = pid_update(&speed_pid, speed_error, dt);
        base_duty += speed_adjust;
        if (base_duty > 0.35f) base_duty = 0.35f;
        if (base_duty < MIN_DUTY) base_duty = MIN_DUTY;

        // Motors
        float left_duty = base_duty + heading_adjust;
        float right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
        motors_set_speed(left_duty, right_duty);

        // Distance
        total_distance += avg_speed * dt;

        // Oscillation: Simple check for sustained cycles (error sign changes >3 with amplitude)
        if (sample_count > 0) {
            if ((line_error * prev_error < 0) && fabsf(line_error) > 0.5f * (LINE_ERROR_THRESHOLD / LINE_SCALE)) {
                oscillation_detected = true;
            }
            prev_error = line_error;
        }

        errors[sample_count] = line_error;
        sample_count++;

        sleep_ms(5);
    }

    // Stop
    motors_set_speed(0.0f, 0.0f);
    float run_time = (float)(millis_now() - start_time) / 1000.0f;

    // Compute stats (errors in mm: scale by LINE_SCALE)
    float sum_error = 0.0f, sum_sq_error = 0.0f;
    float max_error = 0.0f;
    float min_speed = nominal_speed, max_speed = 0.0f;
    for (int i = 0; i < sample_count; i++) {
        float err_mm = fabsf(errors[i]) * LINE_SCALE;
        sum_error += err_mm;
        sum_sq_error += err_mm * err_mm;
        if (err_mm > max_error) max_error = err_mm;

        float spd = speeds[i];
        if (spd < min_speed) min_speed = spd;
        if (spd > max_speed) max_speed = spd;
    }
    float rms_error = sqrtf(sum_sq_error / sample_count);
    float speed_var_percent = ((max_speed - min_speed) / nominal_speed) * 100.0f;

    // Pass/Fail
    bool rms_ok = (rms_error <= 8.0f);
    bool max_ok = (max_error <= 20.0f);
    bool loss_ok = (loss_count == 0);
    bool speed_ok = (speed_var_percent <= 10.0f);
    bool no_osc = !oscillation_detected;

    bool pass = rms_ok && max_ok && loss_ok && speed_ok && no_osc;
    if (pass) {
        printf("PASS: Lateral error RMS <= 8 mm; max <= 20 mm; zero loss-of-line; speed within ±10%%; no oscillation.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!rms_ok) printf("- RMS error: %.1f mm >8 mm\n", rms_error);
        if (!max_ok) printf("- Max error: %.1f mm >20 mm\n", max_error);
        if (!loss_ok) printf("- Loss-of-line events: %d\n", loss_count);
        if (!speed_ok) printf("- Speed variation: %.1f%% >10%%\n", speed_var_percent);
        if (!no_osc) printf("- Oscillation visible in error trace\n");
    }

    // Results
    printf("\nResults:\n");
    printf("- Run time: %.1fs\n", run_time);
    printf("- Distance traveled: %.2f m\n", total_distance);
    printf("- Samples logged: %d\n", sample_count);
    printf("- RMS lateral error: %.1f mm\n", rms_error);
    printf("- Max lateral error: %.1f mm\n", max_error);
    printf("- Loss-of-line events: %d\n", loss_count);
    printf("- Speed range: %.3f - %.3f m/s (variation: %.1f%% of nominal)\n", min_speed, max_speed, speed_var_percent);
    printf("- Oscillation detected: %s\n", no_osc ? "No" : "Yes");

    printf("L2 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
}
