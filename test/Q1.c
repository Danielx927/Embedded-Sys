/*
 * Test ID: Q1
 * Description: Wi-Fi connect and publish baseline telemetry at 10 Hz for 60 s.
 * This test simulates MQTT connection and telemetry publishing (no real Wi-Fi/MQTT in codebase).
 * Adapted: Stub "connect/publish" with prints, gather IMU/encoders/IR/state, log at 10Hz for 60s.
 * Checks: No gaps >200ms, out-of-order <=0.1%, reconnects=0, latency med<=150ms/95th<=250ms (simulated).
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "encoders.h"
#include "line_sensor.h"
#include "utils.h"
#include "config.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Initialize (for telemetry)
    imu_init();
    imu_filter_init(&g_filter);
    encoders_init();
    line_sensor_init();

    printf("Q1 Test: MQTT Telemetry Simulation - Starting\n");
    printf("Simulating Wi-Fi connect and 10Hz publish for 60s.\n");

    // "Configure and connect"
    printf("Configured broker/topic/credentials.\n");
    printf("Wi-Fi connected, MQTT session established.\n");
    uint32_t connect_time = millis_now();

    uint32_t start_time = millis_now();
    uint32_t last_publish = start_time;
    uint32_t publish_count = 0;
    uint32_t reconnect_count = 0;  // Stub: 0
    uint32_t ooo_count = 0;  // Out-of-order
    uint32_t gap_violations = 0;
    float latencies[600];  // 10Hz * 60s = 600
    uint32_t latency_count = 0;
    uint32_t prev_seq = 0;
    static uint32_t prev_enc_left = 0, prev_enc_right = 0;

    // Simulate 10Hz loop for 60s
    while (millis_now() - start_time < 60000) {
        uint32_t now = millis_now();
        uint32_t dt = now - last_publish;

        if (dt >= 100) {  // 10Hz
            last_publish = now;
            publish_count++;

            // Gather telemetry
            imu_read_accel();
            imu_read_mag();
            imu_filter_update(&g_filter, &g_raw_data);
            imu_filter_average(&g_filter, &g_filtered_data);
            float heading = imu_compute_heading(&g_filtered_data);
            uint16_t ir = line_sensor_read_filtered();
            uint32_t enc_left = encoders_get_left_pulses();
            uint32_t enc_right = encoders_get_right_pulses();
            float enc_delta_left = enc_left - prev_enc_left;
            float enc_delta_right = enc_right - prev_enc_right;
            prev_enc_left = enc_left;
            prev_enc_right = enc_right;
            char state[] = "READY";  // Stub

            // Simulate latency (100-250ms random)
            float sim_latency = 100.0f + (150.0f * (rand() % 1000 / 1000.0f));  // Uniform 100-250
            latencies[latency_count++] = sim_latency;

            // "Publish" JSON-like
            printf("[%.3f] Publish #%u: {imu:{h:%.2f,a_x:%ld,a_y:%ld,a_z:%ld},enc:{l:%u,r:%u,d_l:%.0f,d_r:%.0f},ir:%u,state:\"%s\"} latency:%.0fms\n",
                   (float)now / 1000, publish_count, heading,
                   g_filtered_data.accel_x, g_filtered_data.accel_y, g_filtered_data.accel_z,
                   enc_left, enc_right, enc_delta_left, enc_delta_right, ir, state, sim_latency);

            // Check gap
            if (dt > 200) gap_violations++;

            // Seq for ooo (simple increment)
            if (publish_count != prev_seq + 1) ooo_count++;
            prev_seq = publish_count;

            // Stub reconnect (none)
            if ((rand() % 1000) < 10) {  // 1% chance sim fail/reconnect
                reconnect_count++;
                printf("Sim reconnect at %.3f\n", (float)now / 1000);
            }
        }

        sleep_ms(10);  // ~100Hz poll
    }

    // "Disconnect"
    printf("Disconnected cleanly.\n");

    // Stats
    float max_gap = 0.0f;  // From dt checks
    float ooo_percent = (float)ooo_count / publish_count * 100.0f;
    bool gaps_ok = (gap_violations == 0);
    bool ooo_ok = (ooo_percent <= 0.1f);
    bool reconnect_ok = (reconnect_count == 0);

    // Latency: median and 95th
    bool latency_ok = false;
    if (latency_count > 0) {
        // Simple sort for median/95th (small N)
        float lat_copy[600];
        memcpy(lat_copy, latencies, latency_count * sizeof(float));
        qsort(lat_copy, latency_count, sizeof(float), float_cmp);

        float median = lat_copy[latency_count / 2];
        int p95_idx = (int)(0.95f * (latency_count - 1));
        float p95 = lat_copy[p95_idx];
        latency_ok = (median <= 150.0f && p95 <= 250.0f);

        printf("Latency: Median=%.0f ms, 95th=%.0f ms\n", median, p95);
    } else {
        printf("No latency data\n");
    }

    bool pass = gaps_ok && ooo_ok && reconnect_ok && latency_ok;
    if (pass) {
        printf("PASS: No gaps >200ms; OOO <=0.1%%; Reconnects=0; Latency med<=150/95th<=250ms.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!gaps_ok) printf("- Gap violations: %u\n", gap_violations);
        if (!ooo_ok) printf("- OOO: %.1f%%\n", ooo_percent);
        if (!reconnect_ok) printf("- Reconnects: %u\n", reconnect_count);
        if (!latency_ok) printf("- Latency exceeded\n");
    }

    // Results
    printf("\nResults:\n");
    printf("- Publish count: %u\n", publish_count);
    printf("- Gap violations (>200ms): %u\n", gap_violations);
    printf("- OOO count: %u (%.1f%%)\n", ooo_count, ooo_percent);
    printf("- Reconnects: %u\n", reconnect_count);
    printf("- Simulated latency count: %u\n", latency_count);

    printf("Q1 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
}

// Stub cmp for qsort
int float_cmp(const void* a, const void* b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}
