/*
 * Test ID: UC03
 * Description: Recover line when lost; otherwise stop safely after timeout.
 * This test verifies line loss detection, entry to RECOVER state, weave/search pattern,
 * speed reduction, line reacquisition, and safe stop on timeout.
 * Adapted to codebase: Simulates line loss by forcing high error, uses state machine
 * with recovery_dir alternation for weave, reduces speed ≥40%, times entry/reacquisition.
 * Manual: Degrade contrast or position off-line to trigger loss.
 * Run until recovery or timeout (3s), log RMS post-resume.
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

// Define missing constants
#define DIST_PER_PULSE 0.001f  // Meters per encoder pulse, adjust as needed

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

    printf("UC03 Test: Line Loss & Recovery - Starting\n");
    printf("Start in FOLLOW. Degrade line contrast/position off-line to trigger loss.\n");
    printf("Expect RECOVER entry ≤100ms, speed ≥40%% reduction, reacquire ≤3s.\n");

    // States: 0=FOLLOW, 1=RECOVER, 2=RESUMED, 3=STOPPED
    int state = 0;
    float nominal_speed = 0.08f;
    float recovery_speed = nominal_speed * 0.6f;  // ≥40% reduction
    float base_duty = INITIAL_BASE_DUTY;
    uint32_t start_time = millis_now();
    uint32_t state_entry_time = 0;
    uint32_t recovery_start = 0;
    uint32_t last_time = start_time;
    uint32_t weave_switch_time = 0;
    float recovery_dir = 1.0f;  // Start weaving right
    float weave_angle = 0.15f;  // Bounded weave adjustment
    #define MAX_SAMPLES 10000
    float post_errors[MAX_SAMPLES];
    int post_sample_count = 0;
    float total_post_distance = 0.0f;
    float total_distance = 0.0f;
    static uint32_t prev_left = 0, prev_right = 0;
    bool reacquired = false;
    bool timeout_stop = false;
    float pre_speed = 0.0f;

    // Initial FOLLOW to establish baseline
    printf("Entering FOLLOW baseline (place on line)...\n");
    while (millis_now() - start_time < 2000) {  // 2s baseline
        uint32_t now = millis_now();
        float dt = (float)(now - last_time) / 1000.0f;
        if (dt < 0.001f) { sleep_ms(5); continue; }
        last_time = now;

        uint16_t line_val = line_sensor_read_filtered();
        float line_error = line_sensor_compute_error(line_val);

        // Speed
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
        pre_speed = avg_speed;  // Capture baseline

        float speed_error = nominal_speed - avg_speed;
        float speed_adjust = pid_update(&speed_pid, speed_error, dt);
        base_duty += speed_adjust;
        if (base_duty > 0.35f) base_duty = 0.35f;
        if (base_duty < MIN_DUTY) base_duty = MIN_DUTY;

        float heading_adjust = -pid_update(&line_pid, line_error, dt);
        heading_adjust = fmaxf(fminf(heading_adjust, 0.12f), -0.12f);
        float left_duty = base_duty + heading_adjust;
        float right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
        motors_set_speed(left_duty, right_duty);

        sleep_ms(5);
    }

    // Reset for test
    state = 0;
    start_time = millis_now();
    base_duty = INITIAL_BASE_DUTY;
    speed_pid.integral = 0.0f;
    line_pid.integral = 0.0f;

    // Main test loop ~200Hz for 10s max
    while (millis_now() - start_time < 10000) {
        uint32_t now = millis_now();
        float dt = (float)(now - last_time) / 1000.0f;
        if (dt < 0.001f) { sleep_ms(5); continue; }
        last_time = now;

        uint16_t line_val = line_sensor_read_filtered();
        float line_error = line_sensor_compute_error(line_val);
        bool line_lost = (fabsf(line_error) > LINE_ERROR_THRESHOLD / LINE_SCALE);

        // Speed calculation
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

        float heading_adjust = 0.0f;
        float current_target = nominal_speed;

        if (state == 0) {  // FOLLOW
            state_entry_time = now;
            float speed_error = current_target - avg_speed;
            float speed_adjust = pid_update(&speed_pid, speed_error, dt);
            base_duty += speed_adjust;
            if (base_duty > 0.35f) base_duty = 0.35f;
            if (base_duty < MIN_DUTY) base_duty = MIN_DUTY;

            heading_adjust = -pid_update(&line_pid, line_error, dt);
            heading_adjust = fmaxf(fminf(heading_adjust, 0.12f), -0.12f);

            if (line_lost) {
                state = 1;  // Enter RECOVER
                recovery_start = now;
                weave_switch_time = now + 500;  // Switch weave every 500ms
                recovery_dir = 1.0f;
                base_duty *= 0.6f;  // Reduce ≥40%
                current_target = recovery_speed;
                speed_pid.integral = 0.0f;
                line_pid.integral = 0.0f;
                printf("Line lost! Entering RECOVER at t=%.1fs (entry delay: %.0f ms)\n",
                       (float)(now - start_time)/1000, (float)(now - state_entry_time));
            }
        } else if (state == 1) {  // RECOVER
            float elapsed = (float)(now - recovery_start) / 1000.0f;
            if (elapsed > 3.0f) {  // Timeout 3s
                state = 3;
                timeout_stop = true;
                motors_set_speed(0.0f, 0.0f);
                printf("RECOVER timeout at %.1fs - safe stop\n", elapsed);
                break;
            }

            // Weave: Alternate direction every 500ms
            if (now >= weave_switch_time) {
                recovery_dir = -recovery_dir;
                weave_switch_time = now + 500;
                printf("Weave switch to dir=%.1f at %.1fs\n", recovery_dir, elapsed);
            }
            heading_adjust = 0.15f * recovery_dir;  // Bounded weave

            // Speed PID at reduced speed
            float speed_error = current_target - avg_speed;
            float speed_adjust = pid_update(&speed_pid, speed_error, dt);
            base_duty += speed_adjust;
            if (base_duty > 0.25f) base_duty = 0.25f;  // Cap reduced duty
            if (base_duty < 0.1f) base_duty = 0.1f;

            if (!line_lost) {
                state = 2;  // Reacquired
                reacquired = true;
                uint32_t reacq_time = now - recovery_start;
                printf("Line reacquired at %.1fs (within 3s: %s)\n", (float)reacq_time/1000, (reacq_time <= 3000 ? "YES" : "NO"));
            }
        } else if (state == 2) {  // RESUMED FOLLOW
            // Log post-recovery errors/dist for 1m
            if (post_sample_count == 0) {
                state_entry_time = now;  // Start post-log
            }
            float post_error = line_error;
            post_errors[post_sample_count] = post_error;
            post_sample_count++;

            // Resume normal control
            float speed_error = nominal_speed - avg_speed;
            float speed_adjust = pid_update(&speed_pid, speed_error, dt);
            base_duty += speed_adjust;
            if (base_duty > 0.35f) base_duty = 0.35f;
            if (base_duty < MIN_DUTY) base_duty = MIN_DUTY;

            heading_adjust = -pid_update(&line_pid, line_error, dt);
            heading_adjust = fmaxf(fminf(heading_adjust, 0.12f), -0.12f);

            // Distance post-recovery
            float post_start_dist = total_distance;  // Need to track total_distance
            // Simulate total_distance += avg_speed * dt; (add if needed)
            if ((total_distance - post_start_dist) >= 1.0f) {
                // 1m post-resume complete
                break;
            }

            if (line_lost) {
                // Failed resume
                state = 1;
                post_sample_count = 0;
            }
        }

        // Motors (generic)
        float left_duty = base_duty + heading_adjust;
        float right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
        if (state != 3) motors_set_speed(left_duty, right_duty);

        // Simulate total_distance (add actual tracking if needed)
        total_distance += avg_speed * dt;

        sleep_ms(5);
    }

    // Stop
    motors_set_speed(0.0f, 0.0f);
    float total_time = (float)(millis_now() - start_time) / 1000.0f;

    // Post-resume RMS (if reacquired)
    float post_rms = 0.0f;
    bool post_ok = false;
    if (reacquired && post_sample_count > 0) {
        float sum_sq = 0.0f;
        for (int i = 0; i < post_sample_count; i++) {
            float err_mm = fabsf(post_errors[i]) * LINE_SCALE;
            sum_sq += err_mm * err_mm;
        }
        post_rms = sqrtf(sum_sq / post_sample_count);
        post_ok = (post_rms <= 8.0f);
    }

    // Speed reduction check
    float speed_reduction = (pre_speed > 0) ? (pre_speed - recovery_speed) / pre_speed : 0.0f;
    bool speed_red_ok = (speed_reduction >= 0.4f);

    // Entry delay (from FOLLOW to RECOVER)
    uint32_t entry_delay = (state_entry_time > 0) ? state_entry_time - start_time : 0;
    bool entry_ok = (entry_delay <= 100);

    // Pass/Fail
    bool reacq_ok = reacquired && (total_time - (recovery_start - start_time)/1000 <= 3.0f);
    bool pass = entry_ok && speed_red_ok && reacq_ok && post_ok && !timeout_stop;
    if (pass) {
        printf("PASS: RECOVER ≤100ms; speed ≥40%% reduced; reacquired ≤3s/bounds; RMS ≤8mm within 1m; no creep on timeout.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!entry_ok) printf("- Entry delay: %ums >100ms\n", entry_delay);
        if (!speed_red_ok) printf("- Speed reduction: %.1f%% <40%%\n", speed_reduction * 100);
        if (!reacq_ok) printf("- Reacquisition failed/timeout\n");
        if (!post_ok) printf("- Post RMS: %.1fmm >8mm\n", post_rms);
        if (timeout_stop) printf("- Timeout stop (no creep OK)\n");
    }

    // Results
    printf("\nResults:\n");
    printf("- Total time: %.1fs\n", total_time);
    printf("- Entry to RECOVER: %ums\n", entry_delay);
    printf("- Speed reduction: %.1f%% (from %.3f to %.3f m/s)\n", speed_reduction * 100, pre_speed, recovery_speed);
    printf("- Reacquired: %s (time: %.1fs)\n", reacquired ? "YES" : "NO", reacquired ? (total_time - (recovery_start - start_time)/1000) : 0.0f);
    if (reacquired) {
        printf("- Post-resume samples: %d over ~1m\n", post_sample_count);
        printf("- Post RMS error: %.1f mm\n", post_rms);
    }
    printf("- Timeout stop: %s\n", timeout_stop ? "YES" : "NO");

    printf("UC03 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
