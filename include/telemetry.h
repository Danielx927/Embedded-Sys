#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float speed_mps;          // average speed
    float distance_m;         // total distance
    float heading_deg;        // filtered heading to show on dashboard
    float line_error;         // line error for debugging

    const char *state;        // "BOOT", "LINE_FOLLOW", "OBSTACLE", etc
    uint32_t barcode;         // last decoded barcode
    bool barcode_valid;       // true if barcode field is valid
} telemetry_snapshot_t;

/*
 * Initialize Wi Fi and MQTT (non blocking).
 * Call once during startup (after stdio_init_all()).
 */
void telemetry_init(void);

/*
 * Periodic function:
 *  - Maintains Wi Fi connection
 *  - Maintains MQTT connection
 *  - Publishes snapshot at a fixed rate when connected
 *
 * now_ms is typically to_ms_since_boot(get_absolute_time()).
 */
void telemetry_step(const telemetry_snapshot_t *snap, uint32_t now_ms);

/*
 * One shot helpers for events, if you want to publish immediately.
 * They are optional. You can also rely only on telemetry_step.
 */
void telemetry_publish_barcode(uint32_t code);
void telemetry_force_publish(const telemetry_snapshot_t *snap);

void telemetry_publish_obstacle(float dist_center_cm, float left_extent_cm, float right_extent_cm, const char *chosen_path);

#endif
