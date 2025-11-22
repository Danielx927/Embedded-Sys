#include "pico/stdlib.h"
#include "pico/mutex.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "imu.h"
#include "motors.h"
#include "line_sensor.h"
#include "barcode.h"
#include "encoders.h"
#include "multicore_obstacle.h"
#include "navigation.h"
#include "utils.h"
#include "config.h"

static void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == RIGHT_IR_DIGITAL_GPIO) {
        ir_isr(gpio, events);
    } else if (gpio == LEFT_ENCODER || gpio == RIGHT_ENCODER) {
        encoder_callback(gpio, events);
    }
}

mutex_t obstacle_mutex;

// Use extern for obstacle state from multicore_obstacle
extern obstacle_state_t g_obstacle_state;

float target_heading = 0.0f;
pid_controller_t speed_pid = {SPEED_KP, SPEED_KI, SPEED_KD, 0.0f, 0.0f};
pid_controller_t heading_pid = {HEADING_KP_RAMP, HEADING_KI_RAMP, HEADING_KD_RAMP, 0.0f, 0.0f};
pid_controller_t line_pid = {LINE_KP, LINE_KI, LINE_KD, 0.0f, 0.0f};
speed_filter_t g_speed_filter;

// Navigation
robot_state_t current_state = STATE_FOLLOWING;
robot_state_t next_detour_state = STATE_FOLLOWING;
uint32_t stabilize_start_time = 0;
float total_distance = 0.0f;
uint32_t last_lost_time = 0;
float recovery_dir = 0.0f;  // 1.0 for right, -1.0 for left
uint32_t recovery_disabled_until = 0;
uint32_t turn_start_time = 0;  // Timestamp for when to start the turn after delay
uint32_t turn_end_time = 0;    // Timestamp for when turn should complete
uint32_t recovery_start_time = 0;  // New: Timestamp when recovery started
bool recovery_switched = false;    // New: Flag to track if direction switched
int recovery_switch_count = 0;     // New: Counter for switches per recovery attempt
bool recovery_session_active = false;  // New: Flag for active recovery session
int recovery_following_entries = 0;  // New: Counter for FOLLOWING entries in session
uint32_t obstacle_stop_start = 0;
uint32_t obstacle_count = 0;
//static const char* state_names[] = {"FOLLOWING", "TURNING", "RECOVERY"};

// Detour navigation variables
float saved_line_heading = 0.0f;        // Heading before starting detour
float detour_target_heading = 0.0f;     // Target heading for current turn
float detour_sideways_distance = 0.0f;  // Distance to move perpendicular (cm)
float detour_parallel_distance = 0.0f;  // Distance to move parallel (cm)
float detour_start_distance = 0.0f;     // Odometry at start of move
int detour_direction = 0;               // -1 for left, +1 for right
uint32_t detour_phase_start_time = 0;   // Start time for current detour phase
float detour_phase_start_heading = 0.0f; // Heading at start of turn phase

turn_dir_t scheduled_turn = TURN_NONE;

void setup() {
    stdio_init_all();
    sleep_ms(1500);

    mutex_init(&obstacle_mutex);

    // IMU initialization
    imu_init();
    imu_filter_init(&g_filter);

    // Line sensor
    line_sensor_init();

    // Multicore obstacle detection (launches Core 1)
    multicore_obstacle_init();

    // Barcode
    barcode_init();

    // Motors
    motors_init();

    // Encoders
    encoders_init();
    
    // Register the master GPIO callback ONCE for all GPIOs
    gpio_set_irq_callback(&gpio_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // Speed filter
    encoders_filter_init(&g_speed_filter);

    // PIDs
    navigation_init(&speed_pid, &heading_pid, &line_pid);

    // Calibrate heading (assume robot is still)
    target_heading = imu_calibrate_heading(&g_filter, &g_raw_data);
    printf("Initial heading calibrated: %.2f rad\n", target_heading);
    printf("Core 0: Main control loop starting\n");
}

void loop() {
    static uint32_t last_time_ms = 0;
    static uint32_t last_print_ms = 0;
    static float base_duty = INITIAL_BASE_DUTY;
    static float current_target_speed = 0.0f;
    static float ramp_start_time = 0.0f;
    static bool ramping = true;
    static uint32_t prev_left_pulses = 0;
    static uint32_t prev_right_pulses = 0;
    static float left_distance = 0.0f;
    static float right_distance = 0.0f;
    static uint32_t line_lost_counter = 0;
    static uint8_t recovery_counter = 0;

    uint32_t now_ms = millis_now();
    float dt = (float)(now_ms - last_time_ms) / 1000.0f;
    
    // Prevent division by zero or very small dt (reduced threshold for faster loop)
    if (dt < 0.0005f) return;
    
    last_time_ms = now_ms;

    if (ramping) {
        if (ramp_start_time == 0.0f) ramp_start_time = (float)now_ms / 1000.0f;
        float elapsed = (float)now_ms / 1000.0f - ramp_start_time;
        current_target_speed = TARGET_SPEED_MS * fminf(1.0f, elapsed / RAMP_TIME_SEC);
        if (elapsed >= RAMP_TIME_SEC) {
            ramping = false;
            heading_pid.kp = HEADING_KP_STEADY;
            heading_pid.ki = HEADING_KI_STEADY;
            heading_pid.kd = HEADING_KD_STEADY;
            // Ramp complete - no message printed
        }
    } else {
        // Only set base target speed if NOT in recovery with switched direction
        if (!(current_state == STATE_RECOVERY && recovery_switched)) {
            current_target_speed = TARGET_SPEED_MS;
        }
    }

    // Read sensors
    imu_read_accel();
    imu_read_mag();
    imu_filter_update(&g_filter, &g_raw_data);
    imu_filter_average(&g_filter, &g_filtered_data);

    float heading = imu_compute_heading(&g_filtered_data);

    // Read line IR with filtering using module
    uint16_t line_filtered = line_sensor_read_filtered();
    float line_error = line_sensor_compute_error(line_filtered);
    
    // For debug print: get raw if needed
    adc_select_input(LEFT_IR_ADC_CHANNEL);
    uint16_t line_raw = adc_read();

    // Check barcode timeout using modular function
    turn_dir_t detected_turn;
    uint32_t barcode_turn_start;
    if (barcode_check_and_process(now_ms, &detected_turn, &barcode_turn_start)) {
        scheduled_turn = detected_turn;
        turn_start_time = barcode_turn_start;
        if (scheduled_turn == TURN_LEFT) {
            printf("Decoded barcode: Left turn scheduled in %d ms\n", TURN_DELAY_MS);
        } else if (scheduled_turn == TURN_RIGHT) {
            printf("Decoded barcode: Right turn scheduled in %d ms\n", TURN_DELAY_MS);
        }
    }

    // Check if it's time to start the turn
    if (turn_start_time > 0 && now_ms >= turn_start_time) {
        current_state = STATE_TURNING;
        turn_end_time = now_ms + TURN_DURATION_MS;  // Set when turn should end
        current_target_speed = RECOVERY_SPEED_SWITCHED;  // Use 0.16 m/s for faster turn
        if (scheduled_turn == TURN_LEFT) {
            printf("Starting left turn (duration: %d ms)\n", TURN_DURATION_MS);
        } else if (scheduled_turn == TURN_RIGHT) {
            printf("Starting right turn (duration: %d ms)\n", TURN_DURATION_MS);
        }
        turn_start_time = 0;  // Reset the timer
    }

    // Compute speeds and distances using module
    uint32_t left_pulses = encoders_get_left_pulses();
    uint32_t right_pulses = encoders_get_right_pulses();
    uint32_t delta_left = left_pulses - prev_left_pulses;
    uint32_t delta_right = right_pulses - prev_right_pulses;
    prev_left_pulses = left_pulses;
    prev_right_pulses = right_pulses;

    float left_speed = (float)delta_left * DIST_PER_PULSE / dt;
    float right_speed = (float)delta_right * DIST_PER_PULSE / dt;

    float filtered_left_speed, filtered_right_speed;
    encoders_filter_update(&g_speed_filter, left_speed, right_speed, &filtered_left_speed, &filtered_right_speed);

    left_distance += (float)delta_left * DIST_PER_PULSE;
    right_distance += (float)delta_right * DIST_PER_PULSE;
    total_distance = (left_distance + right_distance) / 2.0f;

    float average_speed = (filtered_left_speed + filtered_right_speed) / 2.0f;
    float speed_error = current_target_speed - average_speed;
    float speed_adjust = pid_update(&speed_pid, speed_error, dt);
    base_duty += speed_adjust;
    if (base_duty > 0.35f) base_duty = 0.35f;  /* Allow some speed increase while preventing stalls */
    if (base_duty < MIN_DUTY) base_duty = MIN_DUTY;

    // State machine
    float heading_adjust = 0.0f;

    if (current_state == STATE_FOLLOWING) {
        // Check for obstacles (non-blocking check from Core 1)
        // bool check_obstacle = false;
        float dist_center = 0.0f;
        bool scan_ready = false;
        
        if (obstacle_check_detected(&dist_center)) {
            // STOP IMMEDIATELY when obstacle detected
            motors_set_speed(0.0f, 0.0f);
            
            float left_extent, right_extent, left_angle, right_angle;
            scan_ready = obstacle_get_scan_results(&left_extent, &right_extent, &left_angle, &right_angle);
            
            if (!scan_ready) {
                // First detection - signal Core 1 we've stopped
                printf("\n⚠️  OBSTACLE DETECTED at %.1f cm! Stopped - waiting for scan...\n", dist_center);
                
                obstacle_signal_core0_stopped();
                
                // Wait for scan to complete (Core 1 is scanning)
                while (true) {
                    sleep_ms(50);
                    scan_ready = obstacle_get_scan_results(&left_extent, &right_extent, &left_angle, &right_angle);
                    
                    if (scan_ready) {
                        break;
                    }
                }
            }
            
            // Scan is complete, process results
            current_state = STATE_OBSTACLE_DETECTED;
            printf("\n⚠️  Scan complete! Processing obstacle...\n");
            sleep_ms(200);
            
            // Determine recommended path based on valid readings
            const char* chosen_path = "UNKNOWN";
            float total_width = 0.0f;
            bool left_valid = (left_extent > 0);
            bool right_valid = (right_extent > 0);
            
            if (left_valid && right_valid) {
                // Both readings valid - calculate total obstacle width
                total_width = left_extent + right_extent;
                
                // Choose the side with less extent
                if (left_extent < right_extent) {
                    chosen_path = "LEFT";
                } else if (right_extent < left_extent) {
                    chosen_path = "RIGHT";
                } else {
                    chosen_path = "EQUAL";
                }
            } else if (left_valid && !right_valid) {
                // Only left is clear
                chosen_path = "LEFT (Right blocked/no clear path)";
                total_width = left_extent;
            } else if (!left_valid && right_valid) {
                // Only right is clear
                chosen_path = "RIGHT (Left blocked/no clear path)";
                total_width = right_extent;
            } else {
                // Both invalid - obstacle is very close
                chosen_path = "BLOCKED (No clear path detected)";
                total_width = 0.0f;
            }
            
            obstacle_count++;
            
            // Print obstacle report AFTER scanning is complete
            printf("\n========== OBSTACLE REPORT #%lu ==========\n", obstacle_count);
            printf("Detection Distance: %.1f cm\n", dist_center);
            printf("\n--- Scan Results ---\n");
            
            if (left_valid) {
                printf("Left:  Final angle=%.1f°, Lateral Extent=%.1f cm\n", 
                       left_angle, left_extent);
            } else {
                printf("Left:  No valid reading (obstacle too close or blocked)\n");
            }
            
            if (right_valid) {
                printf("Right: Final angle=%.1f°, Lateral Extent=%.1f cm\n", 
                       right_angle, right_extent);
            } else {
                printf("Right: No valid reading (obstacle too close or blocked)\n");
            }
            
            if (left_valid && right_valid) {
                printf("\n✓ Total Obstacle Width: %.1f cm (%.1f cm left + %.1f cm right)\n", 
                       total_width, left_extent, right_extent);
            } else if (left_valid || right_valid) {
                printf("\n✓ Available Clearance: %.1f cm\n", total_width);
            } else {
                printf("\n✗ No clearance detected - obstacle very close or sensor issue\n");
            }
            
            printf("\nRecommended Path: %s\n", chosen_path);
            
            printf("\n--- Robot State at Detection ---\n");
            printf("Total Distance Travelled: %.2f m\n", total_distance);
            printf("Current Speed: %.2f m/s (Left: %.2f, Right: %.2f)\n", 
                   average_speed, filtered_left_speed, filtered_right_speed);
            printf("Current Heading: %.2f rad (Target: %.2f rad)\n", heading, target_heading);
            printf("Line Position: Raw=%u, Filtered=%u, Error=%.2f\n", 
                   line_raw, line_filtered, line_error);
            printf("Accel: X=%ld, Y=%ld, Z=%ld\n", 
                   g_filtered_data.accel_x, g_filtered_data.accel_y, g_filtered_data.accel_z);
            printf("Mag: X=%ld, Y=%ld, Z=%ld\n", 
                   g_filtered_data.mag_x, g_filtered_data.mag_y, g_filtered_data.mag_z);
            printf("=========================================\n\n");
            
            // Store detour parameters
            saved_line_heading = heading;  // Save current heading before detour
            
            // Determine which side to go around and calculate distances
            if (left_valid && right_valid) {
                if (left_extent < right_extent) {
                    detour_direction = -1;  // Go left
                    detour_sideways_distance = left_extent + DETOUR_SAFETY_MARGIN_CM;
                } else {
                    detour_direction = 1;   // Go right
                    detour_sideways_distance = right_extent + DETOUR_SAFETY_MARGIN_CM;
                }
            } else if (left_valid) {
                detour_direction = -1;
                detour_sideways_distance = left_extent + DETOUR_SAFETY_MARGIN_CM;
            } else if (right_valid) {
                detour_direction = 1;
                detour_sideways_distance = right_extent + DETOUR_SAFETY_MARGIN_CM;
            } else {
                // No valid path - default to right with minimum distance
                detour_direction = 1;
                detour_sideways_distance = DETOUR_MAX_SIDEWAYS_CM;
            }
            
            // Apply maximum cap to prevent excessive detours from sensor noise
            if (detour_sideways_distance > DETOUR_MAX_SIDEWAYS_CM) {
                printf("⚠️  Capping sideways distance from %.1f cm to %.1f cm\n", 
                       detour_sideways_distance, DETOUR_MAX_SIDEWAYS_CM);
                detour_sideways_distance = DETOUR_MAX_SIDEWAYS_CM;
            }
            
            detour_parallel_distance = DETOUR_PARALLEL_DIST_CM;
            
            printf("🔄 INITIATING DETOUR MANEUVER\n");
            printf("   Direction: %s\n", detour_direction == -1 ? "LEFT" : "RIGHT");
            printf("   Sideways distance: %.1f cm\n", detour_sideways_distance);
            printf("   Parallel distance: %.1f cm\n", detour_parallel_distance);
            printf("   Saved heading: %.2f rad\n\n", saved_line_heading);
            
            obstacle_stop_start = now_ms;
            current_state = STATE_OBSTACLE_STOPPED;
            
            // Signal Core 1 that obstacle has been processed
            obstacle_signal_processed();
        } else if (fabsf(line_error) > LINE_ERROR_THRESHOLD / LINE_SCALE) {
            // Debounce line loss detection (reduced from 5 to 3 for faster response)
            if (now_ms >= recovery_disabled_until) {
                line_lost_counter++;
                if (line_lost_counter >= 3) {
                    current_state = STATE_RECOVERY;
                    line_lost_counter = 0;  // Reset counter to prevent carryover on re-entry
                    // Positive error = seeing white/high value = line is to the LEFT → turn LEFT (negative dir)
                    // Negative error = seeing black/low value = line is to the RIGHT → turn RIGHT (positive dir)
                    recovery_dir = (line_error > 0.0f) ? -1.0f : 1.0f;
                    // Line lost recovery - no message printed
                }
            }
        } else if (fabsf(line_error) > 1.05f) {  // Threshold for "very white" readings (very off the line)
            // Debounce line loss detection (increased to 7 consecutive for stricter entry into recovery)
            if (now_ms >= recovery_disabled_until) {
                line_lost_counter++;
                if (line_lost_counter >= 10) {
                    current_state = STATE_RECOVERY;
                    line_lost_counter = 0;  // Reset counter to prevent carryover on re-entry
                    // Positive error = seeing white/high value = line is to the LEFT → turn LEFT (negative dir)
                    // Negative error = seeing black/low value = line is to the RIGHT → turn RIGHT (positive dir)
                    recovery_dir = (line_error > 0.0f) ? -1.0f : 1.0f;
                    // Line lost recovery - no message printed
                }
            }
        } else {
            line_lost_counter = 0;
            last_lost_time = now_ms;
            
            // Line PID: positive error = drifting to white side, need to turn back (negative adjust for left turn)
            heading_adjust = -pid_update(&line_pid, line_error, dt);
            
            // Limit heading adjustment during line following
            if (heading_adjust > 0.12f) heading_adjust = 0.12f;  /* Reduced from 0.15f for more controlled turns */
            if (heading_adjust < -0.12f) heading_adjust = -0.12f;
        }
    } else if (current_state == STATE_TURNING) {
        // Use fixed turn rate based on scheduled turn direction
        if (scheduled_turn == TURN_LEFT) {
            heading_adjust = RECOVERY_TURN_RATE_SWITCHED * (-1.0f);  // Left turn at 0.2 rate
        } else if (scheduled_turn == TURN_RIGHT) {
            heading_adjust = RECOVERY_TURN_RATE_SWITCHED * (1.0f);   // Right turn at 0.2 rate
        }
        
        // Check if turn duration has elapsed
        if (now_ms >= turn_end_time) {
            printf("Turn complete (time-based)\n");
            current_state = STATE_FOLLOWING;
            current_target_speed = TARGET_SPEED_MS;
            target_heading = heading;  // Update to new heading
            line_lost_counter = 0;
            scheduled_turn = TURN_NONE;  // Reset turn direction
            
            // Reset PID integrals
            heading_pid.integral = 0.0f;
            line_pid.integral = 0.0f;
            speed_pid.integral = 0.0f;
            speed_pid.last_error = 0.0f;
            
            recovery_disabled_until = now_ms + RECOVERY_DISABLE_TIME_MS;  // Disable recovery after turn
        }
    } else if (current_state == STATE_RECOVERY) {
        // Initialize recovery variables if not already set (moved here to fix timing bug)
        if (recovery_start_time == 0) {
            recovery_start_time = now_ms;
            recovery_switched = false;
            if (!recovery_session_active) {
                recovery_session_active = true;
                recovery_switch_count = 0;  // Reset counter only at start of new session
                recovery_following_entries = 0;  // Reset FOLLOWING counter
            }
        }
        
        // Use increased speed if direction has been switched
        if (recovery_switched) {
            current_target_speed = RECOVERY_SPEED_SWITCHED;
        }
        
        // Use increased turn rate if direction has been switched
        float current_turn_rate = recovery_switched ? RECOVERY_TURN_RATE_SWITCHED : RECOVERY_TURN_RATE;
        heading_adjust = current_turn_rate * recovery_dir;
        
        // Check for recovery timeout with enhanced curve detection
        uint32_t recovery_elapsed = now_ms - recovery_start_time;
        if (recovery_switch_count < 1 && !recovery_switched && recovery_elapsed > RECOVERY_TIMEOUT_MS) {
            recovery_dir = -recovery_dir;
            recovery_start_time = now_ms;
            recovery_switched = true;
            recovery_switch_count++;
            current_target_speed = RECOVERY_SPEED_SWITCHED;  // Set speed immediately when switching
            printf("Recovery direction switched - increasing speed to %.2f m/s\n", RECOVERY_SPEED_SWITCHED);
        }
        
        // Check if line is recovered with debouncing
        if (fabsf(line_error) <= LINE_ERROR_THRESHOLD / LINE_SCALE) {
            recovery_counter++;
            if (recovery_counter >= 3) {
                // Line recovered - no message printed
                current_state = STATE_FOLLOWING;
                recovery_counter = 0;
                line_lost_counter = 0;
                target_heading = heading;
                current_target_speed = TARGET_SPEED_MS;  // Reset to normal speed
                
                // Reset ALL recovery session variables
                recovery_start_time = 0;        // CRITICAL: Reset timestamp first
                recovery_switched = false;      // Reset switch flag
                recovery_switch_count = 0;      // Reset counter
                recovery_session_active = false; // End session
                recovery_following_entries = 0;  // Reset for future sessions
                
                // Reset PID integrals
                heading_pid.integral = 0.0f;
                line_pid.integral = 0.0f;
                speed_pid.integral = 0.0f;
                speed_pid.last_error = 0.0f;
            }
        } else {
            recovery_counter = 0;
        }
    } else if (current_state == STATE_OBSTACLE_STOPPED) {
        // Brief pause before starting detour maneuver
        uint32_t elapsed = now_ms - obstacle_stop_start;
        if (elapsed >= 1000) {  // 1 second pause
            printf("✅ Starting detour: Stabilizing IMU...\n");
            
            current_state = STATE_DETOUR_STABILIZE;
            next_detour_state = STATE_DETOUR_TURN_PERPENDICULAR;
            stabilize_start_time = now_ms;
        }
        
        // Keep motors stopped during pause
        motors_set_speed(0.0f, 0.0f);
        return;
    } else if (current_state == STATE_DETOUR_STABILIZE) {
        motors_set_speed(0.0f, 0.0f);
        if (now_ms - stabilize_start_time >= 500) { // 500ms stabilization
             current_state = next_detour_state;
             detour_phase_start_time = 0; // Reset for next phase
        }
    } else if (current_state == STATE_DETOUR_TURN_PERPENDICULAR) {
        // Step 1: Turn 90° perpendicular to the line (IMU-controlled turn)
        // NO PID - pure open-loop turn with IMU verification
        
        // Initialize on entry
        if (detour_phase_start_time == 0) {
            detour_phase_start_time = now_ms;
            detour_phase_start_heading = heading;
            
            // Calculate target heading using empirical deltas
            if (detour_direction < 0) {
                // Left turn
                detour_target_heading = saved_line_heading - PERP_LEFT_DELTA;
            } else {
                // Right turn
                detour_target_heading = saved_line_heading + PERP_RIGHT_DELTA;
            }
            
            // Normalize to [-π, π]
            while (detour_target_heading > M_PI) detour_target_heading -= 2.0f * M_PI;
            while (detour_target_heading < -M_PI) detour_target_heading += 2.0f * M_PI;
            
            printf("✅ Starting perpendicular turn\n");
            printf("   From: %.2f rad, To: %.2f rad (%.1f°)\n", 
                   heading, detour_target_heading, detour_target_heading * 180.0f / M_PI);
        }
        
        // Apply constant turn rate (same as barcode turns)
        if (detour_direction < 0) {
            heading_adjust = -DETOUR_SPOT_TURN_DUTY;  // Turn left (negative adjust)
        } else {
            heading_adjust = DETOUR_SPOT_TURN_DUTY;   // Turn right (positive adjust)
        }
        
        // Check if turn complete using IMU feedback
        if (execute_imu_controlled_turn(heading, detour_direction, detour_target_heading)) {
            detour_start_distance = total_distance;
            // saved_line_heading = heading;  // REMOVED: Keep original line heading
            detour_phase_start_time = 0;  // Reset for next phase
            current_state = STATE_DETOUR_MOVE_SIDEWAYS;
            printf("✅ Perpendicular turn complete. Moving sideways %.1f cm...\n", 
                   detour_sideways_distance);
        }
        
    } else if (current_state == STATE_DETOUR_MOVE_SIDEWAYS) {
        // Step 2: Move sideways (perpendicular to line) by detour_sideways_distance
        
        float distance_moved = (total_distance - detour_start_distance) * 100.0f; // to cm
        
        if (distance_moved >= detour_sideways_distance) {
            detour_start_distance = total_distance;
            current_state = STATE_DETOUR_STABILIZE;
            next_detour_state = STATE_DETOUR_TURN_PARALLEL;
            stabilize_start_time = now_ms;
            printf("✅ Sideways movement complete (%.1f cm). Stabilizing...\n", distance_moved);
        } else {
            // BYPASS speed controller - use fixed duty directly
            base_duty = 0.40f;
            
            // Move straight with gentle heading hold
            float heading_error = angle_diff(detour_target_heading, heading);
            heading_adjust = -DETOUR_HEADING_CORRECTION * heading_error;
            
            // Clamp to prevent shakiness
            if (heading_adjust > 0.08f) heading_adjust = 0.08f;
            if (heading_adjust < -0.08f) heading_adjust = -0.08f;
            
            // Apply motor control BEFORE speed PID section
            float left_duty = base_duty + heading_adjust;
            float right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
            
            if (left_duty > 1.0f) left_duty = 1.0f;
            if (left_duty < -1.0f) left_duty = -1.0f;
            if (right_duty > 1.0f) right_duty = 1.0f;
            if (right_duty < -1.0f) right_duty = -1.0f;
            
            motors_set_speed(left_duty, right_duty);
            
            // Print debug and skip rest of loop
            if (now_ms - last_print_ms >= SAMPLE_DELAY_MS) {
                last_print_ms = now_ms;
                printf("Stored widths=%d, Total dist: %.2fm\n",
                       num_widths, total_distance);
            }
            sleep_ms(5);
            return;  // Skip speed PID and motor control at end of loop
        }
        
    } else if (current_state == STATE_DETOUR_TURN_PARALLEL) {
        // Step 3: Turn back to original heading (parallel to line)
        // NO PID - pure open-loop turn with IMU verification
        
        // Initialize on entry
        if (detour_phase_start_time == 0) {
            detour_phase_start_time = now_ms;
            detour_phase_start_heading = heading;
            
            // Calculate target heading for opposite turn (back to parallel)
            // We want to face the original line direction
            detour_target_heading = saved_line_heading;
            
            // Normalize to [-π, π]
            while (detour_target_heading > M_PI) detour_target_heading -= 2.0f * M_PI;
            while (detour_target_heading < -M_PI) detour_target_heading += 2.0f * M_PI;
            
            printf("✅ Starting parallel turn\n");
            printf("   From: %.2f rad, To: %.2f rad (%.1f°)\n", 
                   heading, detour_target_heading, detour_target_heading * 180.0f / M_PI);
        }
        
        // Apply constant turn rate (opposite direction, same as barcode)
        if (detour_direction < 0) {
            heading_adjust = DETOUR_SPOT_TURN_DUTY;   // Turn right (back to parallel)
        } else {
            heading_adjust = -DETOUR_SPOT_TURN_DUTY;  // Turn left (back to parallel)
        }
        
        // Check if turn complete using IMU feedback
        if (execute_imu_controlled_turn(heading, -detour_direction, detour_target_heading)) {  // Opposite direction
            detour_start_distance = total_distance;
            // saved_line_heading = heading;  // REMOVED: Keep original line heading
            detour_phase_start_time = 0;  // Reset for next phase
            current_state = STATE_DETOUR_MOVE_FORWARD;
            printf("✅ Parallel turn complete. Moving forward %.1f cm...\n", 
                   detour_parallel_distance);
        }
        
    } else if (current_state == STATE_DETOUR_MOVE_FORWARD) {
        // Step 4: Move forward (parallel to line) by detour_parallel_distance
        
        float distance_moved = (total_distance - detour_start_distance) * 100.0f;
        
        if (distance_moved >= detour_parallel_distance) {
            detour_start_distance = total_distance;
            current_state = STATE_DETOUR_STABILIZE;
            next_detour_state = STATE_DETOUR_TURN_BACK;
            stabilize_start_time = now_ms;
            printf("✅ Forward movement complete (%.1f cm). Stabilizing...\n", distance_moved);
        } else {
            // BYPASS speed controller - use fixed duty directly
            base_duty = 0.40f;
            
            // Move straight with gentle heading hold
            float heading_error = angle_diff(detour_target_heading, heading);
            heading_adjust = -DETOUR_HEADING_CORRECTION * heading_error;
            
            if (heading_adjust > 0.08f) heading_adjust = 0.08f;
            if (heading_adjust < -0.08f) heading_adjust = -0.08f;
            
            // Apply motor control BEFORE speed PID section
            float left_duty = base_duty + heading_adjust;
            float right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
            
            if (left_duty > 1.0f) left_duty = 1.0f;
            if (left_duty < -1.0f) left_duty = -1.0f;
            if (right_duty > 1.0f) right_duty = 1.0f;
            if (right_duty < -1.0f) right_duty = -1.0f;
            
            motors_set_speed(left_duty, right_duty);
            
            // Print debug and skip rest of loop
            if (now_ms - last_print_ms >= SAMPLE_DELAY_MS) {
                last_print_ms = now_ms;
                printf("Stored widths=%d, Total dist: %.2fm\n",
                       num_widths, total_distance);
            }
            sleep_ms(5);
            return;  // Skip speed PID and motor control at end of loop
        }
        
    } else if (current_state == STATE_DETOUR_TURN_BACK) {
        // Step 5: Turn back toward the line (65° instead of 90°)
        // NO PID - pure open-loop turn with IMU verification
        
        // Initialize on entry
        if (detour_phase_start_time == 0) {
            detour_phase_start_time = now_ms;
            detour_phase_start_heading = heading;
            
            // Calculate target heading for 65° turn (65/90 of perpendicular turn)
            if (detour_direction < 0) {
                // Left detour: Turn Right 65° (65/90 of right perpendicular turn)
                detour_target_heading = saved_line_heading + (PERP_RIGHT_DELTA * 65.0f / 90.0f);
            } else {
                // Right detour: Turn Left 65° (65/90 of left perpendicular turn)
                detour_target_heading = saved_line_heading - (PERP_LEFT_DELTA * 65.0f / 90.0f);
            }
            
            // Normalize to [-π, π]
            while (detour_target_heading > M_PI) detour_target_heading -= 2.0f * M_PI;
            while (detour_target_heading < -M_PI) detour_target_heading += 2.0f * M_PI;
            
            printf("✅ Starting 65° turn back to line\n");
            printf("   From: %.2f rad, To: %.2f rad (%.1f°)\n", 
                   heading, detour_target_heading, detour_target_heading * 180.0f / M_PI);
        }
        
        // Apply constant turn rate (opposite direction from first turn, same as barcode)
        if (detour_direction < 0) {
            heading_adjust = DETOUR_SPOT_TURN_DUTY;   // Turn right (opposite of initial left turn)
        } else {
            heading_adjust = -DETOUR_SPOT_TURN_DUTY;  // Turn left (opposite of initial right turn)
        }
        
        // Check if turn complete using IMU feedback
        if (execute_imu_controlled_turn(heading, -detour_direction, detour_target_heading)) {  // Opposite direction
            detour_start_distance = total_distance;
            detour_phase_start_time = 0;
            current_state = STATE_DETOUR_REJOIN;
            printf("✅ 65° turn back complete. Moving to rejoin line...\n");
        }
    } else if (current_state == STATE_DETOUR_REJOIN) {
        // Step 6: Move back to line while monitoring IR sensor
        
        float distance_moved = (total_distance - detour_start_distance) * 100.0f;
        
        // Check if line is detected
        if (fabsf(line_error) < 0.6f) {  // Line found (scaled error < 600)
            printf("🎯 Line re-acquired! Resuming line following...\n");
            current_state = STATE_FOLLOWING;
            target_heading = heading;
            current_target_speed = TARGET_SPEED_MS;
            recovery_disabled_until = now_ms + RECOVERY_DISABLE_TIME_MS;
            line_lost_counter = 0;
            
            // CRITICAL: Reset obstacle flags so Core 1 doesn't trigger immediately
            obstacle_reset();
            
            // Reset PIDs
            speed_pid.integral = 0.0f;
            speed_pid.last_error = 0.0f;
            heading_pid.integral = 0.0f;
            line_pid.integral = 0.0f;
        } else if (distance_moved >= detour_sideways_distance + 10.0f) {
            // Moved far enough, should have found line
            printf("⚠️  Line not found after %.1f cm. Returning to FOLLOWING anyway...\n", 
                   distance_moved);
            current_state = STATE_FOLLOWING;
            target_heading = heading;
            current_target_speed = TARGET_SPEED_MS;
            
            // CRITICAL: Reset obstacle flags so Core 1 doesn't trigger immediately
            obstacle_reset();
        } else {
            // BYPASS speed controller - use fixed duty directly
            base_duty = 0.40f;
            
            // Move straight with gentle heading hold
            float heading_error = angle_diff(detour_target_heading, heading);
            heading_adjust = -DETOUR_HEADING_CORRECTION * heading_error;
            
            if (heading_adjust > 0.08f) heading_adjust = 0.08f;
            if (heading_adjust < -0.08f) heading_adjust = -0.08f;
            
            // Apply motor control BEFORE speed PID section
            float left_duty = base_duty + heading_adjust;
            float right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
            
            if (left_duty > 1.0f) left_duty = 1.0f;
            if (left_duty < -1.0f) left_duty = -1.0f;
            if (right_duty > 1.0f) right_duty = 1.0f;
            if (right_duty < -1.0f) right_duty = -1.0f;
            
            motors_set_speed(left_duty, right_duty);
            
            // Print debug and skip rest of loop
            if (now_ms - last_print_ms >= SAMPLE_DELAY_MS) {
                last_print_ms = now_ms;
                printf("Stored widths=%d, Total dist: %.2fm\n",
                       num_widths, total_distance);
            }
            sleep_ms(5);
            return;  // Skip speed PID and motor control at end of loop
        }
    }

    // Apply controls
    float left_duty, right_duty;

    if (current_state == STATE_DETOUR_TURN_PERPENDICULAR || 
        current_state == STATE_DETOUR_TURN_PARALLEL || 
        current_state == STATE_DETOUR_TURN_BACK) {
        // Spot turn logic for detour turns (motors opposing)
        // heading_adjust is set to +/- DETOUR_SPOT_TURN_DUTY (0.4)
        // If heading_adjust is negative (Left turn), we want Left Back (-), Right Forward (+)
        // left_duty = -0.4, right_duty = 0.4
        // If heading_adjust is positive (Right turn), we want Left Forward (+), Right Back (-)
        // left_duty = 0.4, right_duty = -0.4
        
        left_duty = heading_adjust;
        right_duty = -heading_adjust;
    } else {
        left_duty = base_duty + heading_adjust;
        right_duty = (base_duty - heading_adjust) * RIGHT_DUTY_FACTOR;
    }

    if (left_duty > 1.0f) left_duty = 1.0f;
    if (left_duty < -1.0f) left_duty = -1.0f;
    if (right_duty > 1.0f) right_duty = 1.0f;
    if (right_duty < -1.0f) right_duty = -1.0f;

    motors_set_speed(left_duty, right_duty);

    // Print debug info periodically
    if (now_ms - last_print_ms >= SAMPLE_DELAY_MS) {
        last_print_ms = now_ms;
        printf("Stored widths=%d, Total dist: %.2fm\n",
               num_widths, total_distance);
    }

    sleep_ms(5);  // Reduced delay for 200Hz control loop (increased from 10ms)
}

int main(void) {
    setup();
    while (true) {
        loop();
    }
    return 0;
}
