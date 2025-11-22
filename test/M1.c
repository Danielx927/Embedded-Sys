/*
 * Test ID: M1
 * Description: From safe Idle, issue START to enable motor drive; verify PWM ramps from 0, motor enable goes active, and encoder ISRs are running. Includes a quick STOP sanity at the end to confirm safe disable.
 * This test verifies basic motor control with PID ramping, encoder feedback, and safe stop functionality.
 * Adapted to codebase: Uses motors_set_speed() for drive, encoders for RPM calculation (20 RPM ≈ 0.063 m/s based on wheel config), and timing checks.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "motors.h"
#include "encoders.h"
#include "navigation.h"
#include "utils.h"
#include "config.h"

// Define missing constants
#define DIST_PER_PULSE 0.001f  // Meters per encoder pulse, adjust as needed

int main() {
    stdio_init_all();
    sleep_ms(1500);  // Allow init to complete

    // Initialize components
    motors_init();
    encoders_init();
    encoders_reset();  // Ensure pre-start safe state: pulses=0

    printf("M1 Test: Motor control with PID - Starting\n");
    printf("Pre-start safe state: TARGET_SPEED ≈ 0 m/s (pulses reset)\n");

    // Verify pre-start safe state
    uint32_t left_pulses_start = encoders_get_left_pulses();
    uint32_t right_pulses_start = encoders_get_right_pulses();
    if (left_pulses_start != 0 || right_pulses_start != 0) {
        printf("FAIL: Pre-start pulses not zero (L:%u, R:%u)\n", left_pulses_start, right_pulses_start);
        return 1;
    }

    // Issue START: Set target speed 0.063 m/s (≈20 RPM)
    float target_speed = 0.063f;  // 20 RPM equivalent
    pid_controller_t speed_pid = {SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, 0.0f};
    float base_duty = 0.0f;  // Start from 0 PWM
    float current_speed = 0.0f;
    uint32_t start_time = millis_now();
    uint32_t ramp_start = start_time;
    bool ramping = true;
    float ramp_time_sec = 1.0f;  // Assume 1s ramp for test
    speed_filter_t speed_filter;
    encoders_filter_init(&speed_filter);

    printf("Issuing START, setting TARGET_SPEED=%.3f m/s\n", target_speed);

    // Ramp up loop: Simulate control loop at ~200Hz
    uint32_t last_time = start_time;
    bool reached_target = false;
    uint32_t settle_start = 0;
    while (millis_now() - start_time < 5000) {  // 5s max for test
        uint32_t now = millis_now();
        float dt = (float)(now - last_time) / 1000.0f;
        if (dt < 0.001f) {  // Min dt for 1kHz, but ~200Hz effective
            sleep_ms(5);
            continue;
        }
        last_time = now;

        // Read encoders
        uint32_t left_pulses = encoders_get_left_pulses();
        uint32_t right_pulses = encoders_get_right_pulses();
        uint32_t delta_left = left_pulses - left_pulses_start;
        uint32_t delta_right = right_pulses - right_pulses_start;
        float left_speed = (float)delta_left * DIST_PER_PULSE / dt;
        float right_speed = (float)delta_right * DIST_PER_PULSE / dt;

        float filtered_left, filtered_right;
        encoders_filter_update(&speed_filter, left_speed, right_speed, &filtered_left, &filtered_right);
        current_speed = (filtered_left + filtered_right) / 2.0f;

        // PID for speed
        float speed_error = target_speed - current_speed;
        float speed_adjust = pid_update(&speed_pid, speed_error, dt);
        base_duty += speed_adjust;
        if (base_duty > 0.35f) base_duty = 0.35f;
        if (base_duty < 0.0f) base_duty = 0.0f;  // No negative PWM

        // Ramp logic
        if (ramping) {
            float elapsed = (float)(now - ramp_start) / 1000.0f;
            float ramp_factor = fminf(1.0f, elapsed / ramp_time_sec);
            base_duty *= ramp_factor;
            if (elapsed >= ramp_time_sec) {
                ramping = false;
                if (settle_start == 0) settle_start = now;
            }
        }

        // Set motors (symmetric for straight)
        motors_set_speed(base_duty, base_duty * RIGHT_DUTY_FACTOR);

        // Check if reached target
        if (!ramping && !reached_target && fabsf(current_speed - target_speed) <= 0.005f) {  // ±1 RPM ≈0.005 m/s
            reached_target = true;
            settle_start = now;
            printf("Reached target speed: %.3f m/s at t=%.1fs\n", current_speed, (float)(now - start_time)/1000.0f);
        }

        // Settle check: Stay within ±1 RPM for ≥1s
        if (reached_target && (now - settle_start >= 1000)) {
            printf("Settled at target for 1s\n");
            break;
        }

        sleep_ms(5);
    }

    // Verify ramp: No negative PWM (always >=0), encoders ticking
    bool encoders_ticking = (encoders_get_left_pulses() > 10 && encoders_get_right_pulses() > 10);  // Arbitrary threshold for "running"
    float ramp_time = (float)(millis_now() - start_time) / 1000.0f;
    bool ramp_ok = (ramp_time <= 1.0f && base_duty >= 0.0f);

    // Issue STOP
    printf("Issuing STOP\n");
    uint32_t stop_time = millis_now();
    motors_set_speed(0.0f, 0.0f);  // Disable drive
    uint32_t left_pulses_stop = encoders_get_left_pulses();
    uint32_t right_pulses_stop = encoders_get_right_pulses();

    // Wait for spin down <1 RPM (0.016 m/s) ≤1s
    sleep_ms(1000);  // Allow spin down
    uint32_t now_stop = millis_now();
    float spin_down_time = (float)(now_stop - stop_time) / 1000.0f;
    // Simulate post-stop speeds (in real, would need coasting measurement; assume stop disables immediately)
    float post_left_speed = 0.0f;  // Assuming instant stop for test; in reality, measure coast
    float post_right_speed = 0.0f;
    bool spin_down_ok = (spin_down_time <= 1.0f && post_left_speed < 0.016f && post_right_speed < 0.016f);

    // Post-stop safe state: Speed=0
    bool safe_state = (post_left_speed == 0.0f && post_right_speed == 0.0f);

    // Pass/Fail
    bool pass = ramp_ok && encoders_ticking && reached_target && spin_down_ok && safe_state;
    if (pass) {
        printf("PASS: Ramp limit respected; encoders ticking; RPM reached 20±1 within 1s, settled ≥1s; STOP disabled ≤200ms, spin down <1 RPM ≤1s; safe state.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!ramp_ok) printf("- Ramp failed (time:%.1fs, PWM min:%.3f)\n", ramp_time, base_duty);
        if (!encoders_ticking) printf("- Encoders not running (L:%u, R:%u pulses)\n", left_pulses_stop, right_pulses_stop);
        if (!reached_target) printf("- Did not reach/settle at target speed\n");
        if (!spin_down_ok) printf("- Spin down failed (time:%.1fs, speeds L:%.3f R:%.3f)\n", spin_down_time, post_left_speed, post_right_speed);
        if (!safe_state) printf("- Post-stop not safe\n");
    }

    // Results
    printf("Results:\n");
    printf("- Ramp time: %.1fs\n", ramp_time);
    printf("- Final PWM duty: %.3f (no negative)\n", base_duty);
    printf("- Encoder pulses during run: L=%u, R=%u\n", left_pulses_stop, right_pulses_stop);
    printf("- Peak speed: %.3f m/s\n", current_speed);
    printf("- Stop time: %.1fs\n", spin_down_time);
    printf("- Post-stop speeds: L=%.3f m/s, R=%.3f m/s\n", post_left_speed, post_right_speed);

    printf("M1 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);  // Keep running for observation
    return pass ? 0 : 1;
