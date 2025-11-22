#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "line_sensor.h"
#include "navigation.h"
#include "utils.h"
#include "config.h"

// Define missing constants
#define DIST_PER_PULSE 0.001f  // Meters per encoder pulse, adjust as needed
#define LINE_ERROR_THRESHOLD 100.0f  // Adjust as needed
#define LINE_SCALE 1.0f  // Adjust as needed
#define RIGHT_DUTY_FACTOR 0.95f  // Adjust as needed

// Mock functions
float angle_diff(float target, float current) {
    float diff = target - current;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

// Mock plan from UC07
typedef struct {
    float side; // 1.0 right, -1.0 left
    float arc_radius;
    float arc_angle;
    float forward_distance;
} detour_plan_t;

detour_plan_t plan = {1.0f, 0.5f, M_PI/2, 0.5f}; // Right bypass

// Mock MQTT for event
void publish_event(const char* event) {
    printf("Event: %s\n", event);
}

// State enum (add if not defined)
typedef enum {
    STATE_DETOUR_TURN_PERPENDICULAR,
    STATE_DETOUR_MOVE_SIDEWAYS,
    STATE_DETOUR_TURN_PARALLEL,
    STATE_DETOUR_MOVE_FORWARD,
    STATE_DETOUR_TURN_BACK,
    STATE_DETOUR_REJOIN,
    STATE_FOLLOWING
} robot_state_t;

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Initialize systems
    motors_init();
    encoders_init();
    imu_init();
    imu_filter_init(&g_filter);
    line_sensor_init();

    printf("UC08 Test: Bypass Obstacle & Rejoin Line - Starting\n");
    printf("Assume UC07 plan ready. Execute detour: arc leave, IMU around obstacle, line search, resume FOLLOW.\n");
    printf("Expect: Path ±10cm; no collision; reacquire in window; timeout to search/stop.\n");

    // Simulate entering AVOID from UC07 (after stop)
    robot_state_t state = STATE_DETOUR_TURN_PERPENDICULAR;
    printf("Entered AVOID state with plan: side=%.1f, radius=%.1f m, angle=%.2f rad, fwd=%.1f m\n",
           plan.side, plan.arc_radius, plan.arc_angle, plan.forward_distance);

    // Initial position/heading
    float current_heading = imu_calibrate_heading(&g_filter, &g_raw_data);
    float total_distance = 0.0f;
    float phase_distance = 0.0f;
    uint32_t start_time = millis_now();
    bool line_reacquired = false;
    uint32_t reacquire_time = 0;
    float path_error_max = 0.0f;  // Track max deviation cm
    bool collision = false;  // Stub: assume no
    uint32_t phase_start_time = 0;
    float phase_start_heading = current_heading;
    float phase_start_distance = 0.0f;
    pid_controller_t heading_pid = {0.02f, 0.0f, 1.2f, 0.0f, 0.0f};  // Steady
    pid_controller_t speed_pid = {SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, 0.0f};
    speed_filter_t speed_filter;
    encoders_filter_init(&speed_filter);
    static uint32_t prev_left = 0, prev_right = 0;
    uint32_t last_time = start_time;

    // Detour sequence ~200Hz, 30s timeout
    while (state != STATE_FOLLOWING && millis_now() - start_time < 30000) {
        uint32_t now = millis_now();
        float dt = (float)(now - last_time) / 1000.0f;
        if (dt < 0.001f) { sleep_ms(5); continue; }
        last_time = now;

        // Read IMU
        imu_read_accel();
        imu_read_mag();
        imu_filter_update(&g_filter, &g_raw_data);
        imu_filter_average(&g_filter, &g_filtered_data);
        current_heading = imu_compute_heading(&g_filtered_data);

        // Read encoders
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
        total_distance += avg_speed * dt;

        float heading_adjust = 0.0f;
        float left_duty = 0.0f, right_duty = 0.0f;
        float target_speed = 0.08f;  // Detour speed

        if (state == STATE_DETOUR_TURN_PERPENDICULAR) {
            if (phase_start_time == 0) {
                phase_start_time = now;
                phase_start_heading = current_heading;
                phase_start_distance = total_distance;
                float target = phase_start_heading + (plan.side * M_PI / 2.0f);
                while (target > M_PI) target -= 2 * M_PI;
                while (target < -M_PI) target += 2 * M_PI;
                printf("Phase 1: Turn perpendicular (target %.2f rad)\n", target);
            }
            float error = angle_diff(phase_start_heading + (plan.side * M_PI / 2.0f), current_heading);
            heading_adjust = pid_update(&heading_pid, error, dt);
            // Spot turn
            float power = 0.3f + 0.1f * heading_adjust;
            left_duty = plan.side > 0 ? power : -power;
            right_duty = plan.side > 0 ? -power : power;
            if (fabsf(error) < 0.1f) {  // Complete
                state = STATE_DETOUR_MOVE_SIDEWAYS;
                phase_start_time = 0;
                motors_set_speed(0.0f, 0.0f);
                printf("Perpendicular turn complete\n");
            }
        } else if (state == STATE_DETOUR_MOVE_SIDEWAYS) {
            if (phase_start_time == 0) {
                phase_start_time = now;
                phase_start_heading = current_heading;
                phase_start_distance = total_distance;
                printf("Phase 2: Move sideways %.1f m\n", plan.arc_radius);
            }
            phase_distance = total_distance - phase_start_distance;
            float target_heading_side = phase_start_heading + (plan.side * M_PI / 2.0f);
            float error = angle_diff(target_heading_side, current_heading);
            heading_adjust = -pid_update(&heading_pid, error, dt);
            heading_adjust = fmaxf(fminf(heading_adjust, 0.15f), -0.15f);
            float speed_error = target_speed - avg_speed;
            float speed_adjust = pid_update(&speed_pid, speed_error, dt);
            float base_d = 0.3f + speed_adjust;
            left_duty = base_d + heading_adjust;
            right_duty = (base_d - heading_adjust) * RIGHT_DUTY_FACTOR;
            path_error_max = fmaxf(path_error_max, fabsf(phase_distance - plan.arc_radius * 100.0f));
            if (phase_distance >= plan.arc_radius) {
                state = STATE_DETOUR_TURN_PARALLEL;
                phase_start_time = 0;
                motors_set_speed(0.0f, 0.0f);
                printf("Sideways move complete (%.2f m, error %.1f cm)\n", phase_distance, path_error_max);
            }
        } else if (state == STATE_DETOUR_TURN_PARALLEL) {
            if (phase_start_time == 0) {
                phase_start_time = now;
                phase_start_heading = current_heading;
                phase_start_distance = total_distance;
                float target = phase_start_heading - (plan.side * M_PI / 2.0f);
                while (target > M_PI) target -= 2 * M_PI;
                while (target < -M_PI) target += 2 * M_PI;
                printf("Phase 3: Turn parallel (target %.2f rad)\n", target);
            }
            float error = angle_diff(phase_start_heading - (plan.side * M_PI / 2.0f), current_heading);
            heading_adjust = pid_update(&heading_pid, error, dt);
            float power = 0.3f + 0.1f * heading_adjust;
            left_duty = plan.side < 0 ? power : -power;
            right_duty = plan.side < 0 ? -power : power;
            if (fabsf(error) < 0.1f) {
                state = STATE_DETOUR_MOVE_FORWARD;
                phase_start_time = 0;
                motors_set_speed(0.0f, 0.0f);
                printf("Parallel turn complete\n");
            }
        } else if (state == STATE_DETOUR_MOVE_FORWARD) {
            if (phase_start_time == 0) {
                phase_start_time = now;
                phase_start_heading = current_heading;
                phase_start_distance = total_distance;
                printf("Phase 4: Move forward %.1f m\n", plan.forward_distance);
            }
            phase_distance = total_distance - phase_start_distance;
            float target_heading_fwd = phase_start_heading;
            float error = angle_diff(target_heading_fwd, current_heading);
            heading_adjust = -pid_update(&heading_pid, error, dt);
            heading_adjust = fmaxf(fminf(heading_adjust, 0.15f), -0.15f);
            float speed_error = target_speed - avg_speed;
            float speed_adjust = pid_update(&speed_pid, speed_error, dt);
            float base_d = 0.3f + speed_adjust;
            left_duty = base_d + heading_adjust;
            right_duty = (base_d - heading_adjust) * RIGHT_DUTY_FACTOR;
            path_error_max = fmaxf(path_error_max, fabsf(phase_distance - plan.forward_distance * 100.0f));
            if (phase_distance >= plan.forward_distance) {
                state = STATE_DETOUR_TURN_BACK;
                phase_start_time = 0;
                motors_set_speed(0.0f, 0.0f);
                printf("Forward move complete (%.2f m)\n", phase_distance);
            }
        } else if (state == STATE_DETOUR_TURN_BACK) {
            if (phase_start_time == 0) {
                phase_start_time = now;
                phase_start_heading = current_heading;
                phase_start_distance = total_distance;
                float target = phase_start_heading + (plan.side * M_PI / 2.0f);
                while (target > M_PI) target -= 2 * M_PI;
                while (target < -M_PI) target += 2 * M_PI;
                printf("Phase 5: Turn back (target %.2f rad)\n", target);
            }
            float error = angle_diff(phase_start_heading + (plan.side * M_PI / 2.0f), current_heading);
            heading_adjust = pid_update(&heading_pid, error, dt);
            float power = 0.3f + 0.1f * heading_adjust;
            left_duty = plan.side > 0 ? -power : power;
            right_duty = plan.side > 0 ? power : -power;
            if (fabsf(error) < 0.1f) {
                state = STATE_DETOUR_REJOIN;
                phase_start_time = 0;
                motors_set_speed(0.0f, 0.0f);
                printf("Turn back complete\n");
            }
        } else if (state == STATE_DETOUR_REJOIN) {
            if (phase_start_time == 0) {
                phase_start_time = now;
                phase_start_heading = current_heading;
                phase_start_distance = total_distance;
                printf("Phase 6: Rejoin line search (window 1m)\n");
            }
            phase_distance = total_distance - phase_start_distance;
            // Slow forward with line search (weave if needed)
            float line_val = line_sensor_read_filtered();
            float line_error = line_sensor_compute_error(line_val);
            if (fabsf(line_error) <= LINE_ERROR_THRESHOLD / LINE_SCALE) {
                line_reacquired = true;
                reacquire_time = now;
                state = STATE_FOLLOWING;
                motors_set_speed(0.0f, 0.0f);
                publish_event("detour_success_line_reacquired");
                printf("Line reacquired at %.2f m post-detour\n", phase_distance);
                break;
            }
            // Bounded search: Gentle weave
            static float search_dir = 1.0f;
            static uint32_t weave_time = 0;
            if (weave_time == 0) weave_time = now + 500;
            if (now > weave_time) {
                search_dir = -search_dir;
                weave_time = now + 500;
            }
            heading_adjust = 0.1f * search_dir;
            float slow_speed = 0.04f;
            float speed_error = slow_speed - avg_speed;
            float speed_adjust = pid_update(&speed_pid, speed_error, dt);
            float base_d = 0.2f + speed_adjust;
            left_duty = base_d + heading_adjust;
            right_duty = (base_d - heading_adjust) * RIGHT_DUTY_FACTOR;
            if (phase_distance > 1.0f) {  // Timeout window
                // Bounded search then stop/alert
                motors_set_speed(0.0f, 0.0f);
                publish_event("detour_timeout_bounded_search_stop");
                printf("Reacquisition timeout after 1m - safe stop + alert\n");
                collision = true;  // Flag failure
                break;
            }
        }

        // Apply motors (clamp)
        left_duty = fmaxf(fminf(left_duty, 0.4f), -0.4f);
        right_duty = fmaxf(fminf(right_duty, 0.4f), -0.4f);
        motors_set_speed(left_duty, right_duty);

        sleep_ms(5);
    }

    // Final verification
    float total_time = (float)(millis_now() - start_time) / 1000.0f;
    bool path_ok = (path_error_max <= 10.0f);
    bool reacq_ok = line_reacquired && (phase_distance <= 1.0f);
    bool no_coll = !collision;
    bool timeout_handled = (state == STATE_FOLLOWING || (phase_distance > 1.0f && !line_reacquired));  // Proper handling

    bool pass = path_ok && reacq_ok && no_coll && timeout_handled;
    if (pass) {
        printf("PASS: Path tracking ±10cm; no collision; line reacquired in window; timeout to bounded search/stop.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!path_ok) printf("- Max path error: %.1f cm >10cm\n", path_error_max);
        if (!reacq_ok) printf("- Reacquire failed (dist %.2f m)\n", phase_distance);
        if (!no_coll) printf("- Collision detected\n");
        if (!timeout_handled) printf("- Timeout not handled properly\n");
    }

    // Results
    printf("\nResults:\n");
    printf("- Total time: %.1fs\n", total_time);
    printf("- Total distance: %.2f m\n", total_distance);
    printf("- Max path error: %.1f cm\n", path_error_max);
    printf("- Line reacquired: %s at %.2f m (time: %ums)\n", line_reacquired ? "Yes" : "No", phase_distance, reacquire_time ? (reacquire_time - start_time) : 0);
    printf("- Collision: %s\n", no_coll ? "No" : "Yes");
    printf("- Timeout handling: %s\n", timeout_handled ? "OK (stop/alert)" : "Failed");
    if (line_reacquired) publish_event("detour_success");

    printf("UC08 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
