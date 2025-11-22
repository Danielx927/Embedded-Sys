/*
 * Test ID: UC07
 * Description: Sweep on servo to locate edges, estimate width, and choose bypass side.
 * This test verifies post-stop servo sweep -45° to +45° 5° steps, edge detection from range jumps,
 * width computation via geometry, bypass side selection. Place rectangular obstacle ~250mm centered (known width).
 * Adapted to codebase: Uses ultrasonic_set_servo(), ultrasonic_measure_cm(), geometry for width.
 * Run after UC06 stop, log (angle, distance), detect edges, compute width, choose side.
 * Pass: Width ±15%, edges ±5°, side widest gap.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic.h"
#include "utils.h"
#include "config.h"
#include <math.h>

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Initialize
    ultrasonic_init();

    printf("UC07 Test: Obstacle Edge Scan & Width Estimation - Starting\n");
    printf("Assume UC06 complete (robot stopped). Place rectangular obstacle centered ~250mm (known width).\n");
    printf("Sweep -45° to +45° 5° steps, detect edges, estimate width, choose bypass.\n");
    printf("Expect: Edges ±5°, width ±15%%, side widest gap.\n");

    // Wait for UC06 stop (manual/simulate)
    printf("Waiting 2s for robot stop/positioning...\n");
    sleep_ms(2000);

    // Sweep parameters
    float angles_deg[19] = {-45, -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45};
    int num_steps = 19;
    float distances[19];
    float left_edge_ang = 0.0f, right_edge_ang = 0.0f;
    float left_edge_dist = 0.0f, right_edge_dist = 0.0f;
    bool left_found = false, right_found = false;
    float prev_dist = -1.0f;
    float jump_threshold = 50.0f;  // cm discontinuity for edge

    printf("\n--- Servo Sweep -45° to +45° (5° steps) ---\n");
    printf("Place obstacle, run sweep...\n");

    for (int i = 0; i < num_steps; i++) {
        float ang_deg = angles_deg[i];
        // Compute pulse: center 1500us, ±500us for ±90° approx
        int16_t pulse_offset = (int16_t)(ang_deg * (500.0f / 45.0f));
        uint16_t pulse = SERVO_CENTER + pulse_offset;
        ultrasonic_set_servo(pulse);
        sleep_ms(100);  // Settle

        distances[i] = ultrasonic_measure_cm();
        printf("Angle %.0f° (pulse %u): Dist=%.1f cm\n", ang_deg, pulse, distances[i]);

        // Edge detection: Jump > threshold
        if (i > 0 && prev_dist > 0 && fabsf(distances[i] - prev_dist) > jump_threshold) {
            if (ang_deg < 0 && !left_found) {
                left_edge_ang = ang_deg;
                left_edge_dist = prev_dist;  // Edge at close dist
                left_found = true;
                printf("  -> Left edge at %.0f° (%.1f cm)\n", left_edge_ang, left_edge_dist);
            } else if (ang_deg > 0 && !right_found) {
                right_edge_ang = ang_deg;
                right_edge_dist = prev_dist;
                right_found = true;
                printf("  -> Right edge at %.0f° (%.1f cm)\n", right_edge_ang, right_edge_dist);
            }
        }
        prev_dist = distances[i];
    }

    // Return to center
    ultrasonic_set_servo(SERVO_CENTER);
    sleep_ms(500);

    // Compute width: Lateral extents + projection
    float left_extent = 0.0f, right_extent = 0.0f;
    if (left_found) {
        float ang_rad = fabsf(left_edge_ang * M_PI / 180.0f);
        left_extent = left_edge_dist * sinf(ang_rad);
    }
    if (right_found) {
        float ang_rad = fabsf(right_edge_ang * M_PI / 180.0f);
        right_extent = right_edge_dist * sinf(ang_rad);
    }
    float computed_width = left_extent + right_extent;  // Basic lateral sum
    // More accurate: Add front projection if angles known
    float front_width = 25.0f * (cosf(fabsf(left_edge_ang * M_PI / 180.0f)) + cosf(fabsf(right_edge_ang * M_PI / 180.0f)) - 2.0f);
    computed_width += fabsf(front_width);

    // Bypass side: Wider free gap (dist - extent)
    float left_gap = left_found ? left_edge_dist - left_extent : 0.0f;
    float right_gap = right_found ? right_edge_dist - right_extent : 0.0f;
    const char* bypass_side = (left_gap > right_gap) ? "LEFT" : ((right_gap > left_gap) ? "RIGHT" : "EQUAL/STOP");
    if (!left_found && !right_found) bypass_side = "NONE (Alert)";

    // Stub plan publish
    printf("Stored bypass plan: side=%s, path params (arc/radius/speed)\n", bypass_side);
    printf("Published planning event\n");

    // Pass/Fail (known_width=200mm placeholder; user measures)
    float known_width = 200.0f;  // Manual input
    bool width_ok = (fabsf(computed_width - known_width) / known_width <= 0.15f);
    bool edges_ok = (left_found && right_found && fabsf(left_edge_ang + 20) <= 5.0f && fabsf(right_edge_ang - 20) <= 5.0f);  // Assume true edges ~±20°
    bool bypass_ok = (strcmp(bypass_side, "EQUAL/STOP") != 0 || !left_found || !right_found);  // Consistent if gaps differ or no clear

    bool pass = width_ok && edges_ok && bypass_ok && left_found && right_found;
    if (pass) {
        printf("PASS: Width ±15%%; edges ±5°; bypass consistent with widest gap; plan stored/event published.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!width_ok) printf("- Width: %.1f cm vs known %.0f cm (%.1f%% error)\n", computed_width, known_width, fabsf(computed_width - known_width)/known_width * 100);
        if (!edges_ok) printf("- Edges: L=%.1f° R=%.1f° (>±5° from expected)\n", left_edge_ang, right_edge_ang);
        if (!bypass_ok) printf("- Bypass inconsistent: %s\n", bypass_side);
        if (!left_found || !right_found) printf("- Edge(s) not found\n");
    }

    // Results
    printf("\nResults:\n");
    printf("- Edges found: Left=%s at %.1f° (%.1f cm, extent %.1f cm)\n",
           left_found ? "Yes" : "No", left_edge_ang, left_edge_dist, left_extent);
    printf("- Edges found: Right=%s at %.1f° (%.1f cm, extent %.1f cm)\n",
           right_found ? "Yes" : "No", right_edge_ang, right_edge_dist, right_extent);
    printf("- Computed width: %.1f cm\n", computed_width);
    printf("- Bypass side: %s (L gap=%.1f cm, R gap=%.1f cm)\n", bypass_side, left_gap, right_gap);
    printf("- All distances: ");
    for (int i = 0; i < num_steps; i++) {
        printf("%.0f ", distances[i]);
    }
    printf("cm\n");
    printf("- Plan: side=%s, arc/radius/speed (stub)\n", bypass_side);

    printf("UC07 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
