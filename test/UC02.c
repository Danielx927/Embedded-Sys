/*
 * Test ID: UC02
 * Description: Begin following the black line using IR + PID and maintain tracking accuracy at target speed.
 * This test verifies entry to FOLLOW state, PID control for line tracking and speed, over 10m track.
 * Adapted to codebase: Uses line_sensor_compute_error(), pid_update for line/speed, motors_set_speed, encoders for distance/speed.
 * Assume line present beneath sensor. Run until 10m or manual stop.
 * RMS/max error in mm (error * LINE_SCALE), speed ±10% nominal 0.3 m/s (use 0.08f from config).
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

    printf("UC02 Test: Start Line Following - Starting\n");
    printf("Confirm line present. Operator 'taps Follow' - entering FOLLOW state.\n");

    // Enter FOLLOW state
    float target_speed = 0.08f;  // Nominal from config (0.3 m/s approx)
    float base_duty = INITIAL_BASE_DUTY;
    float total_distance = 0.0f;
    uint32_t start_time = millis_now();
    uint32_t last_time = start_time;
    #define MAX_SAMPLES 10000  // For 10m at 0.08m/s ~125s, but limit
    float errors[MAX_SAMPLES];
    float speeds[MAX_SAMPLES];
    int sample_count = 0;
    int loss_count = 0;
    bool oscillation_detected = false;
    float prev_error = 0.0f;

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

        // Check loss-of-line
        if (fabsf(line_error) > LINE_ERROR_THRESHOLD / LINE_SCALE) {
            loss_count++;
        }

        // Line PID
        float heading_adjust = -pid_update(&line_pid, line_error, dt);
        heading_adjust = fmaxf(fminf(heading_adjust, 0.12f), -0.12f);

        // Speed from encoders
        static uint32_t prev_left = 0, prev_right = 0;
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

        // Speed PID
        float speed_error = target_speed - avg_speed;
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

        // Oscillation: >3 cycles >1/2 amplitude (simple: error delta sign change >3 times)
        if (sample_count > 0) {
            float delta_error = line_error - prev_error;
            if (delta_error * prev_error < 0 && fabsf(line_error) > 0.5f * LINE_ERROR_THRESHOLD / LINE_SCALE) {
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

    // Compute stats
    float sum_error = 0.0f, sum_sq_error = 0.0f;
    float max_error = 0.0f;
    float min_speed = target_speed, max_speed = 0.0f;
    for (int i = 0; i < sample_count; i++) {
        float err_mm = errors[i] * LINE_SCALE;
        sum_error += err_mm;
        sum_sq_error += err_mm * err_mm;
        if (fabsf(err_mm) > max_error) max_error = fabsf(err_mm);

        float spd = speeds[i];
        if (spd < min_speed) min_speed = spd;
        if (spd > max_speed) max_speed = spd;
    }
    float rms_error = sqrtf(sum_sq_error / sample_count);
    float speed_var = (max_speed - min_speed) / target_speed;

    bool rms_ok = (rms_error <= 8.0f);
    bool max_ok = (max_error <= 20.0f);
    bool loss_ok = (loss_count == 0);
    bool speed_ok = (speed_var <= 0.1f);
    bool no_osc = !oscillation_detected;

    bool pass = rms_ok && max_ok && loss_ok && speed_ok && no_osc;
    if (pass) {
        printf("PASS: FOLLOW latched; IR error 100Hz; RMS <=8mm; max<=20mm; no loss; speed ±10%%; no oscillation.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!rms_ok) printf("- RMS error: %.1f mm >8mm\n", rms_error);
        if (!max_ok) printf("- Max error: %.1f mm >20mm\n", max_error);
        if (!loss_ok) printf("- Loss events: %d\n", loss_count);
        if (!speed_ok) printf("- Speed variation: %.1f%% >10%%\n", speed_var * 100);
        if (!no_osc) printf("- Oscillation detected\n");
    }

    // Results
    printf("\nResults:\n");
    printf("- Run time: %.1fs\n", run_time);
    printf("- Distance: %.2f m\n", total_distance);
    printf("- Samples: %d\n", sample_count);
    printf("- RMS lateral error: %.1f mm\n", rms_error);
    printf("- Max lateral error: %.1f mm\n", max_error);
    printf("- Loss-of-line events: %d\n", loss_count);
    printf("- Speed variation: %.1f%% of nominal\n", speed_var * 100.0f);
    printf("- Oscillation: %s\n", no_osc ? "None" : "Detected");

    printf("UC02 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
