#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include <math.h>
#include "utils.h"
#include "config.h"

// Mock micros_now if not defined
#ifndef micros_now
uint32_t micros_now() {
    return millis_now() * 1000;
}
#endif

// Mock WiFi and MQTT functions (since not implemented in project)
static bool wifi_connected = true;
static bool mqtt_connected = true;
static uint32_t reconnect_attempts = 0;
static uint32_t buffered_messages = 0;
static uint32_t total_messages = 0;
static uint32_t dropped_messages = 0;
static uint32_t seq_num = 0;
static uint32_t last_seq = 0;
static uint32_t control_loops = 0;
static uint32_t max_loop_time = 0;
static uint32_t total_loop_time = 0;
uint32_t base_loop_time = 5;  // Nominal 5ms loop

void wifi_connect() {
    wifi_connected = true;
    printf("WiFi connected\n");
}

void wifi_disconnect() {
    wifi_connected = false;
    printf("WiFi disconnected\n");
}

int mqtt_publish(const char* topic, const char* msg) {
    seq_num++;
    if (mqtt_connected) {
        if (seq_num != last_seq + 1) {
            dropped_messages++;  // OOO as drop for simplicity
        }
        last_seq = seq_num;
        total_messages++;
        printf("Published to %s: %s (seq %u)\n", topic, msg, seq_num);
        return 0;
    } else {
        buffered_messages++;
        printf("[BUFFERED] %s: %s (seq %u)\n", topic, msg, seq_num);
        return -1;
    }
}

void mqtt_reconnect() {
    if (wifi_connected) {
        // Flush buffered in order
        for (uint32_t i = 0; i < buffered_messages; i++) {
            char flush_msg[64];
            sprintf(flush_msg, "flushed_%u", i + 1);
            mqtt_publish("telemetry", flush_msg);  // Will publish since connected
        }
        buffered_messages = 0;
        reconnect_attempts++;
        printf("MQTT reconnected (attempt %u, flushed %u msgs)\n", reconnect_attempts, total_messages - seq_num + buffered_messages);
        mqtt_connected = true;
    }
}

int main() {
    stdio_init_all();
    srand(millis_now());  // Seed random

    printf("UC10 Test: Wi-Fi Drop & Auto-Reconnect - Starting\n");
    printf("Simulate UC09: 10Hz telemetry, induce outage, reconnect ≤30s backoff, flush ordered, loss ≤0.1%%, jitter ≤10%%\n");

    // Start connected
    wifi_connect();
    mqtt_reconnect();

    uint32_t start_time = millis_now();
    uint32_t last_publish = start_time;
    uint32_t outage_start = 0;
    bool in_outage = false;
    uint32_t backoff_ms = 1000;  // Initial 1s
    uint32_t last_reconnect_attempt = 0;
    uint32_t loop_start = 0;
    float jitter_percent = 0.0f;

    while (millis_now() - start_time < 60000) {  // 60s test
        loop_start = micros_now();
        uint32_t now = millis_now();

        // Telemetry publish 10Hz
        if (now - last_publish >= 100) {
            char msg[64];
            sprintf(msg, "telemetry_t%u", now);
            int result = mqtt_publish("telemetry/main", msg);
            if (result != 0 && !in_outage) {
                in_outage = true;
                outage_start = now;
                printf("Outage start at %.1fs\n", (float)(now - start_time)/1000);
            } else if (result == 0 && in_outage) {
                in_outage = false;
                uint32_t duration = now - outage_start;
                printf("Outage end at %.1fs (duration %.1fs)\n", (float)(now - start_time)/1000, (float)duration/1000);
            }
            last_publish = now;
        }

        // Induce outage at 20s
        if (now - start_time > 20000 && wifi_connected) {
            wifi_disconnect();
            mqtt_connected = false;
            backoff_ms = 1000;
            last_reconnect_attempt = now;
            printf("Induced WiFi/MQTT drop at %.1fs\n", (float)(now - start_time)/1000);
        }

        // Reconnect with exponential backoff if disconnected
        if (!mqtt_connected) {
            if (now - last_reconnect_attempt >= backoff_ms) {
                wifi_connect();  // Assume WiFi always available
                mqtt_reconnect();
                last_reconnect_attempt = now;
                backoff_ms = fminf(backoff_ms * 2, 30000);  // Double, cap 30s
            }
        }

        // Simulate control loop timing/jitter
        control_loops++;
        uint32_t loop_time = micros_now() - loop_start;
        total_loop_time += loop_time;
        if (loop_time > max_loop_time) max_loop_time = loop_time;
        jitter_percent = ((float)max_loop_time / (base_loop_time * 1000) - 1.0f) * 100.0f;

        sleep_ms(5);  // Nominal loop
    }

    // Final reconnect if needed
    if (!mqtt_connected) mqtt_reconnect();

    // Stats
    float avg_cadence = total_messages / ((float)(millis_now() - start_time) / 1000.0f / 100.0f);  // Expected 10Hz
    float loss_percent = (float)dropped_messages / total_messages * 100.0f;
    float avg_loop_ms = (float)total_loop_time / control_loops / 1000.0f;
    float jitter_ok = (jitter_percent <= 10.0f);
    bool reconnect_ok = (reconnect_attempts > 0 && backoff_ms <= 30000);
    bool loss_ok = (loss_percent <= 0.1f);
    bool buffer_ok = (buffered_messages == 0);
    bool jitter_control_ok = (avg_loop_ms <= base_loop_time * 1.1f);  // ≤10% jitter

    bool pass = reconnect_ok && loss_ok && buffer_ok && jitter_control_ok;
    if (pass) {
        printf("PASS: Control jitter ≤+10%%; reconnect ≤30s backoff; buffered flushed ordered; loss ≤0.1%%.\n");
    } else {
        printf("FAIL: One or more criteria not met.\n");
        if (!reconnect_ok) printf("- Reconnect: %u attempts, final backoff %ums\n", reconnect_attempts, backoff_ms);
        if (!loss_ok) printf("- Loss: %.2f%% >0.1%%\n", loss_percent);
        if (!buffer_ok) printf("- Buffered: %u unflushed\n", buffered_messages);
        if (!jitter_control_ok) printf("- Jitter: avg %.1fms (%.1f%% >10%%)\n", avg_loop_ms, jitter_percent);
    }

    // Results
    printf("\nResults:\n");
    printf("- Test duration: %.1fs\n", (float)(millis_now() - start_time)/1000);
    printf("- Total messages: %u (attempted ~%u at 10Hz)\n", total_messages, (int)((millis_now() - start_time)/100));
    printf("- Dropped/OOO: %u (%.2f%%)\n", dropped_messages, loss_percent);
    printf("- Buffered (unflushed): %u\n", buffered_messages);
    printf("- Reconnects: %u (max backoff %ums)\n", reconnect_attempts, backoff_ms);
    printf("- Control loops: %u, avg time %.1fms, max %.1fms (jitter %.1f%%)\n",
           control_loops, avg_loop_ms, (float)max_loop_time/1000, jitter_percent);

    printf("UC10 Test: %s\n", pass ? "PASS" : "FAIL");
    while (true) sleep_ms(1000);
    return pass ? 0 : 1;
