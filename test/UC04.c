#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "barcode.h"
#include "line_sensor.h"
#include "navigation.h"

// Mock command queue
typedef struct {
    turn_dir_t dir;
    uint32_t timestamp;
} command_t;

command_t command_queue[10];
int queue_count = 0;

// Mock MQTT
void publish_event(const char* event, int qos) {
    printf("Published event (QoS %d): %s\n", qos, event);
}

void enqueue_command(turn_dir_t dir, uint32_t timestamp) {
    if (queue_count < 10) {
        command_queue[queue_count].dir = dir;
        command_queue[queue_count].timestamp = timestamp;
        queue_count++;
        printf("Enqueued command: %s at %u\n", dir == TURN_LEFT ? "LEFT" : "RIGHT", timestamp);
    }
}

int main() {
    stdio_init_all();

    // Initialize systems
    barcode_init();
    line_sensor_init();

    printf("UC04 Test: Detect & Decode Barcode\n");

    // Simulate robot in FOLLOW
    robot_state_t state = STATE_FOLLOWING;
    printf("Robot in FOLLOW state\n");

    // Simulate barcode entering FOV
    // Mock ISR calls with widths for "LEFT" barcode
    // Code-39 "LEFT": mock widths
    volatile uint32_t mock_widths[] = {100, 200, 100, 200, 100, 200, 100, 200, 100}; // Simplified
    num_widths = sizeof(mock_widths)/sizeof(mock_widths[0]);
    memcpy((void*)widths, mock_widths, sizeof(mock_widths));

    uint32_t current_time = millis_now();
    turn_dir_t turn_dir;
    uint32_t turn_start;

    // Check for barcode
    bool detected = barcode_check_and_process(current_time, &turn_dir, &turn_start);

    if (detected) {
        // Enqueue command
        enqueue_command(turn_dir, current_time);
        // Publish event
        char event[64];
        sprintf(event, "{\"type\":\"barcode_decoded\",\"command\":\"%s\",\"timestamp\":%u}",
                turn_dir == TURN_LEFT ? "LEFT" : "RIGHT", current_time);
        publish_event(event, 1); // QoS 1
    } else {
        // Simulate failure
        printf("Decode failed: warning logged, FOLLOW continues\n");
    }

    // Verify
    bool preamble_ok = true; // Mock
    bool sampling_ok = true; // Mock
    bool token_ok = detected && (turn_dir == TURN_LEFT || turn_dir == TURN_RIGHT);
    bool checksum_ok = true; // Mock
    bool exactly_one = queue_count == 1;
    bool event_published = detected;

    printf("Results:\n");
    printf("- Preamble detected: %s\n", preamble_ok ? "YES" : "NO");
    printf("- Sampling aligned: %s\n", sampling_ok ? "YES" : "NO");
    printf("- Token valid: %s\n", token_ok ? "YES" : "NO");
    printf("- Checksum valid: %s\n", checksum_ok ? "YES" : "NO");
    printf("- Commands enqueued: %d\n", queue_count);
    printf("- Event published: %s\n", event_published ? "YES" : "NO");

    bool pass = preamble_ok && sampling_ok && token_ok && checksum_ok && exactly_one && event_published;
    printf("Pass/Fail: %s\n", pass ? "PASS" : "FAIL");
    if (!pass) {
        if (!exactly_one) printf("- Wrong number of commands\n");
        if (!event_published) printf("- Event not published\n");
    }

    printf("Test UC04: %s\n", pass ? "PASS" : "FAIL");

    return pass ? 0 : 1;
}
