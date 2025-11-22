/*
 * Test ID: B1
 * Description: Decode a Code-39 "LEFT/RIGHT" at nominal speed without duplicate triggers.
 * This test verifies barcode detection and decoding during line following, with debounce to prevent duplicates.
 * Adapted to codebase: Uses barcode_init(), barcode_check_and_process() in control loop, assumes physical barcode passage. Run for 30s at nominal speed.
 * Expected: Single "LEFT" or "RIGHT" decode, command queued (turn_dir set), continue FOLLOW.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "motors.h"
#include "encoders.h"
#include "line_sensor.h"
#include "barcode.h"
#include "navigation.h"
#include "utils.h"
#include "config.h"
#include <string.h>

// Define missing constants
#define DIST_PER_PULSE 0.001f  // Meters per encoder pulse, adjust as needed

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Initialize
    line_sensor_init();
    barcode_init();
    motors_init();
    encoders_init();
    encoders_reset();

    // PIDs and filter
    pid_controller_t speed_pid = {SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, 0.0f};
    pid_controller_t line_pid = {LINE_KP, LINE_KI, LINE_KD, 0.0f, 0.0f};
    speed_filter_t speed_filter;
    encoders_filter_init(&speed_filter);

    printf("B1 Test: Barcode Decode at Nominal Speed - Starting\n");
    printf("Drive over single Code-39 'LEFT/RIGHT' barcode. Run for 30s.\n");

    float nominal_speed = 0.08f;
    float base_duty = INITIAL_BASE_DUTY;
    uint32_t start_time = millis_now();
    uint32_t last_time = start_time;
    uint32_t last_barcode_check = start_time;
    uint32_t last_decode_time = 0;
    int decode_count = 0;
    char last_token[10] = {0};
    turn_dir_t last_turn_dir = TURN_NONE;
    uint32_t last_turn_start = 0;
    bool command_queued = false;
    bool continue_follow = true;

    // Control loop ~200Hz for 30s
    while (millis_now() - start_time < 30000) {
        uint32_t now = millis_now();
        float dt = (float)(now - last_time) / 1000.0f;
        if (dt < 0.001f) {
            sleep_ms(5);
            continue;
        }
        last_time = now;

        // Read line for following
        uint16_t line_val = line_sensor_read_filtered();
        float line_error = line_sensor_compute_error(line_val);
        float heading_adjust = -pid_update(&line_pid, line_error, dt);
        heading_adjust = fmaxf(fminf(heading_adjust, 0.12f), -0.12f);

        // Speed control
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
        float speed_error = nominal_speed - avg_speed;
        float speed_adjust = pid_update(&speed_pid, speed_error, dt);
        base_duty += speed_adjust;
        if (base_duty > 0.35f) base_duty = 0.35f;
        if (base_duty < MIN_DUTY) base_duty = MIN_DUTY;

        // Barcode check every 100ms (debounce window 1s)
        if (now - last_barcode_check >= 100) {
            last_barcode_check = now;
            turn_dir_t turn_dir;
            uint32_t turn_start;
            if (barcode_check_and_process(now, &turn_dir, &turn_start)) {
                decode_count++;
                last_decode_time = now;
                strcpy(last_token, decoded_barcode);
                last_turn_dir = turn_dir;
                last_turn_start = turn_start;
                command_queued = true;
                printf("Barcode decoded: '%s' at t=%.1fs, turn_dir=%d, start=%u\n", last_token, (float)now/1000, turn_dir, turn_start);
            }
        }

        // Motor control (continue FOLLOW)
        float left_duty = base_duty + heading_adjust;
        float right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
        motors_set_speed(left_duty, right_duty);

        // If command queued, simulate continue FOLLOW (no state change for test)
        if (command_queued && continue_follow) {
            printf("Command queued, continuing FOLLOW\n");
            continue_follow = false;  // Log once
        }

        sleep_ms(5);
    }

    // Stop
    motors_set_speed(0.0f, 0.0f);

    // Check duplicates: No more decodes within 1s of last
    bool no_duplicate = (decode_count <= 1) || (last_decode_time - start_time > 1000);  // Simple check; assume single if count=1
    bool token_match = (strcmp(last_token, "LEFT") == 0 || strcmp(last_token, "RIGHT") == 0);
    bool debounce_ok = (decode_count == 1);  // Exactly one in window
    bool state_continue = command_queued && continue_follow;  // Continued after

    // Pass/Fail
    bool pass = (decode_count == 1) && token_match && no_duplicate && debounce_ok && state_continue;
    if (pass) {
        printf("PASS: Exactly one decode; token matches; no duplicate in 1s; command queued; continue FOLLOW.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (decode_count != 1) printf("- Decode events: %d (expected 1)\n", decode_count);
        if (!token_match) printf("- Token '%s' mismatch\n", last_token);
        if (!no_duplicate) printf("- Duplicate within 1s\n");
        if (!debounce_ok) printf("- Debounce failed\n");
        if (!state_continue) printf("- Did not continue FOLLOW\n");
    }

    // Results
    printf("\nResults:\n");
    printf("- Decode count: %d\n", decode_count);
    printf("- Last token: '%s'\n", last_token);
    printf("- Last turn_dir: %d, start: %u ms\n", last_turn_dir, last_turn_start);
    printf("- Command queued: %s\n", command_queued ? "Yes" : "No");
    printf("- Continue FOLLOW after: %s\n", state_continue ? "Yes" : "No");
    if (decode_count > 0) {
        printf("- Event timestamp: %.1fs\n", (float)last_decode_time / 1000);
    }

    printf("B1 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
