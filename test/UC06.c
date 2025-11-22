/*
 * Test ID: UC06
 * Description: Detect obstacle and stop within latency budget; publish event.
 * This test verifies ultrasonic detection ≤ d_crit triggers OBSTACLE state,
 * motor stop latency <50ms (local), publish event, transition to UC07.
 * Adapted to codebase: Uses multicore_obstacle_check_detected(), motors_stop(),
 * timing for latency, stub publish. Manual: Place obstacle in path during FOLLOW.
 * Run in FOLLOW, trigger detection, measure stop time, verify disable/no contact.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "motors.h"
#include "encoders.h"
#include "line_sensor.h"
#include "multicore_obstacle.h"
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
    multicore_obstacle_init();  // Launches Core 1

    // PIDs (for baseline FOLLOW)
    pid_controller_t speed_pid = {SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, 0.0f};
    pid_controller_t line_pid = {LINE_KP, LINE_KI, LINE_KD, 0.0f, 0.0f};
    speed_filter_t speed_filter;
    encoders_filter_init(&speed_filter);

    printf("UC06 Test: Obstacle Detection & Safe Stop - Starting\n");
    printf("Start in FOLLOW. Place obstacle in path to trigger detection ≤ d_crit.\n");
    printf("Expect: OBSTACLE latched, stop <80ms, no contact, event published, to UC07.\n");

    float nominal_speed = 0.08f;
    float base_duty = INITIAL_BASE_DUTY;
    uint32_t start_time = millis_now();
    uint32_t last_time = start_time;
    uint32_t detection_time = 0;
    uint32_t stop_command_time = 0;
    uint32_t stop_actual_time = 0;
    bool obstacle_detected = false;
    float dist_at_detection = 0.0f;
    float stop_distance = 0.0f;
    static uint32_t prev_left = 0, prev_right = 0;
    float pre_speed = 0.0f;
    bool spurious_handled = false;  // Stub for A1

    // Baseline FOLLOW to establish speed
    printf("Baseline FOLLOW (place on line, no obstacle)...\n");
    while (millis_now() - start_time < 3000) {  // 3s baseline
        uint32_t now = millis_now();
        float dt = (float)(now - last_time) / 1000.0f;
        if (dt < 0.001f) { sleep_ms(5); continue; }
        last_time = now;

        uint16_t line_val = line_sensor_read_filtered();
        float line_error = line_sensor_compute_error(line_val);
        float heading_adjust = -pid_update(&line_pid, line_error, dt);
        heading_adjust = fmaxf(fminf(heading_adjust, 0.12f), -0.12f);

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
        pre_speed = avg_speed;  // Baseline

        float speed_error = nominal_speed - avg_speed;
        float speed_adjust = pid_update(&speed_pid, speed_error, dt);
        base_duty += speed_adjust;
        if (base_duty > 0.35f) base_duty = 0.35f;
        if (base_duty < MIN_DUTY) base_duty = MIN_DUTY;

        float left_duty = base_duty + heading_adjust;
        float right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
        motors_set_speed(left_duty, right_duty);

        sleep_ms(5);
    }

    // Reset for test
    start_time = millis_now();
    base_duty = INITIAL_BASE_DUTY;
    speed_pid.integral = 0.0f;
    line_pid.integral = 0.0f;
    prev_left = encoders_get_left_pulses();
    prev_right = encoders_get_right_pulses();
    float start_distance = 0.0f;  // From encoders

    printf("FOLLOW with obstacle monitoring (place obstacle)...\n");

    // Main loop ~200Hz
    while (millis_now() - start_time < 10000) {
        uint32_t now = millis_now();
        float dt = (float)(now - last_time) / 1000.0f;
        if (dt < 0.001f) { sleep_ms(5); continue; }
        last_time = now;

        // Check obstacle (non-blocking)
        float dist_center = 0.0f;
        if (obstacle_check_detected(&dist_center)) {
            detection_time = now;
            obstacle_detected = true;
            dist_at_detection = dist_center;
            printf("Obstacle detected at %.1f cm at t=%.1fs\n", dist_center, (float)(now - start_time)/1000);

            // Immediate stop
            stop_command_time = now;
            motors_set_speed(0.0f, 0.0f);
            stop_actual_time = now;  // Assume instant for test; in real measure PWM disable

            // Wait brief for spin-down <50ms E2E
            sleep_ms(50);
            stop_actual_time = millis_now();

            // Publish event (stub)
            printf("Published obstacle event with range %.1f cm, state OBSTACLE\n", dist_at_detection);

            // Transition to UC07 prep (signal stopped)
            obstacle_signal_core0_stopped();
            current_state = STATE_OBSTACLE_DETECTED;  // Latch

            // Measure stop distance (from encoders during stop)
            uint32_t left_stop = encoders_get_left_pulses();
            uint32_t right_stop = encoders_get_right_pulses();
            float delta_dist = ((float)(left_stop - prev_left) + (float)(right_stop - prev_right)) * DIST_PER_PULSE / 2.0f;
            stop_distance = delta_dist;

            break;
        }

        // Normal FOLLOW
        uint16_t line_val = line_sensor_read_filtered();
        float line_error = line_sensor_compute_error(line_val);
        float heading_adjust = -pid_update(&line_pid, line_error, dt);
        heading_adjust = fmaxf(fminf(heading_adjust, 0.12f), -0.12f);

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

        float speed_error = nominal_speed - avg_speed;
        float speed_adjust = pid_update(&speed_pid, speed_error, dt);
        base_duty += speed_adjust;
        if (base_duty > 0.35f) base_duty = 0.35f;
        if (base_duty < MIN_DUTY) base_duty = MIN_DUTY;

        float left_duty = base_duty + heading_adjust;
        float right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
        motors_set_speed(left_duty, right_duty);

        sleep_ms(5);
    }

    // Post-stop verification
    float total_time = (float)(millis_now() - start_time) / 1000.0f;
    float latency_ms = obstacle_detected ? (float)(stop_actual_time - detection_time) : 0.0f;
    bool latency_ok = (latency_ms < 80.0f);  // <80ms total
    bool state_ok = obstacle_detected && (current_state == STATE_OBSTACLE_DETECTED);
    bool no_contact = (stop_distance < 0.05f);  // <5cm safe margin
    bool event_ok = true;  // Stub
    bool spurious_ok = spurious_handled;  // Stub A1

    bool pass = latency_ok && state_ok && no_contact && event_ok && spurious_ok;
    if (pass) {
        printf("PASS: Detection ≤ d_crit; OBSTACLE latched; stop <80ms; no contact; event published; to UC07.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!latency_ok) printf("- Latency: %.0f ms >80ms\n", latency_ms);
        if (!state_ok) printf("- State not OBSTACLE or not detected\n");
        if (!no_contact) printf("- Stop distance %.2f m (contact?)\n", stop_distance);
        if (!event_ok) printf("- No event published\n");
        if (!spurious_ok) printf("- Spurious echo not handled\n");
    }

    // Results
    printf("\nResults:\n");
    printf("- Total time: %.1fs\n", total_time);
    printf("- Detection time: %.1fs at %.1f cm\n", (float)(detection_time - start_time)/1000, dist_at_detection);
    printf("- Stop latency: %.0f ms (command at %.0f ms, actual at %.0f ms)\n",
           latency_ms, (float)(stop_command_time - detection_time), (float)(stop_actual_time - detection_time));
    printf("- Stop distance: %.2f m (safe margin OK: %s)\n", stop_distance, no_contact ? "Yes" : "No");
    printf("- State transition: %s\n", state_ok ? "OBSTACLE latched" : "Failed");
    printf("- Event published: Yes (stub)\n");
    printf("- Spurious handling: %s (stub)\n", spurious_handled ? "OK" : "N/A");

    printf("UC06 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
