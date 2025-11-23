#include "telemetry.h"
#include "config.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/sync.h"

#include "lwip/ip_addr.h"
#include "lwip/apps/mqtt.h"

#include <stdio.h>
#include <string.h>

/* Internal state */

typedef enum {
    TEL_STATE_IDLE = 0,
    TEL_STATE_WIFI_CONNECTING,
    TEL_STATE_MQTT_CONNECTING,
    TEL_STATE_RUNNING,
    TEL_STATE_ERROR
} tel_state_t;

static tel_state_t g_state = TEL_STATE_IDLE;
static mqtt_client_t *g_client = NULL;

static uint32_t g_last_pub_ms = 0;
static uint32_t g_last_retry_ms = 0;
static bool g_mqtt_connected = false;

/* Forward declarations */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status);
static void mqtt_publish_snapshot(const telemetry_snapshot_t *snap);

/* Build broker IP from config */
static ip_addr_t get_broker_ip(void)
{
    ip_addr_t ip;
    IP4_ADDR(&ip, MQTT_BROKER_IP0, MQTT_BROKER_IP1,
                  MQTT_BROKER_IP2, MQTT_BROKER_IP3);
    return ip;
}

void telemetry_init(void)
{
    // Init Wi Fi chip and enable station mode
    if (cyw43_arch_init()) {
        g_state = TEL_STATE_ERROR;
        return;
    }

    cyw43_arch_enable_sta_mode();

    g_state = TEL_STATE_WIFI_CONNECTING;
    g_last_pub_ms = 0;
    g_last_retry_ms = 0;
    g_mqtt_connected = false;
}

/* Helper to check Wi Fi link */
static bool wifi_is_up(void)
{
    int status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
    return status == CYW43_LINK_UP;
}

/* Try to connect to Wi Fi, non blocking at high level */
static void try_wifi_connect(void)
{
    if (wifi_is_up()) {
        return;
    }

    int rc = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID,
        WIFI_PASSWORD,
        CYW43_AUTH_WPA2_AES_PSK,
        10000
    );

    if (rc == 0) {
        g_state = TEL_STATE_MQTT_CONNECTING;
    } else {
        g_state = TEL_STATE_ERROR;
    }
}

/* MQTT connection callback */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg,
                               mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;

    if (status == MQTT_CONNECT_ACCEPTED) {
        g_mqtt_connected = true;
        g_state = TEL_STATE_RUNNING;
    } else {
        g_mqtt_connected = false;
        g_state = TEL_STATE_ERROR;
    }
}

/* Try to connect to MQTT broker */
static void try_mqtt_connect(void)
{
    if (!wifi_is_up()) {
        g_state = TEL_STATE_WIFI_CONNECTING;
        g_mqtt_connected = false;
        return;
    }

    if (!g_client) {
        g_client = mqtt_client_new();
        if (!g_client) {
            g_state = TEL_STATE_ERROR;
            return;
        }
    }

    ip_addr_t broker = get_broker_ip();

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico-robocar";
    ci.keep_alive = 60;

    err_t err = mqtt_client_connect(
        g_client,
        &broker,
        1883,
        mqtt_connection_cb,
        NULL,
        &ci
    );

    if (err != ERR_OK) {
        g_state = TEL_STATE_ERROR;
        g_mqtt_connected = false;
    }
}

/* Publish helper: string */
static void mqtt_pub_str(const char *topic, const char *payload)
{
    if (!g_client || !g_mqtt_connected) return;

    mqtt_publish(
        g_client,
        topic,
        payload,
        (u16_t)strlen(payload),
        0,          /* qos 0 */
        0,          /* retain = 0 */
        NULL,       /* no callback */
        NULL        /* no callback arg */
    );
}

/* Publish full snapshot into topics expected by dashboard.html */
static void mqtt_publish_snapshot(const telemetry_snapshot_t *snap)
{
    char buf[64];

    /* car/telemetry/state */
    if (snap->state) {
        mqtt_pub_str("car/telemetry/state", snap->state);
    }

    /* car/telemetry/speed */
    snprintf(buf, sizeof(buf), "%.3f", snap->speed_mps);
    mqtt_pub_str("car/telemetry/speed", buf);

    /* car/telemetry/distance */
    snprintf(buf, sizeof(buf), "%.3f", snap->distance_m);
    mqtt_pub_str("car/telemetry/distance", buf);

    /* car/telemetry/heading */
    snprintf(buf, sizeof(buf), "%.2f", snap->heading_deg);
    mqtt_pub_str("car/telemetry/heading", buf);

    /* car/telemetry/line_error */
    snprintf(buf, sizeof(buf), "%.3f", snap->line_error);
    mqtt_pub_str("car/telemetry/line_error", buf);

    /* car/telemetry/barcode */
    if (snap->barcode_valid) {
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)snap->barcode);
        mqtt_pub_str("car/telemetry/barcode", buf);
    }
}

/* Public API */

void telemetry_step(const telemetry_snapshot_t *snap, uint32_t now_ms)
{
    if (!snap) return;

    switch (g_state) {
    case TEL_STATE_IDLE:
        // Should not happen after init, but try Wi Fi anyway
        g_state = TEL_STATE_WIFI_CONNECTING;
        break;

    case TEL_STATE_WIFI_CONNECTING:
        if (now_ms - g_last_retry_ms >= TELEMETRY_RETRY_MS) {
            g_last_retry_ms = now_ms;
            try_wifi_connect();
        }
        break;

    case TEL_STATE_MQTT_CONNECTING:
        if (!wifi_is_up()) {
            g_state = TEL_STATE_WIFI_CONNECTING;
            g_mqtt_connected = false;
            break;
        }
        if (now_ms - g_last_retry_ms >= TELEMETRY_RETRY_MS) {
            g_last_retry_ms = now_ms;
            try_mqtt_connect();
        }
        break;

    case TEL_STATE_RUNNING:
        if (!wifi_is_up()) {
            g_state = TEL_STATE_WIFI_CONNECTING;
            g_mqtt_connected = false;
            break;
        }

        if (!g_mqtt_connected) {
            g_state = TEL_STATE_MQTT_CONNECTING;
            break;
        }

        if (now_ms - g_last_pub_ms >= TELEMETRY_PERIOD_MS) {
            g_last_pub_ms = now_ms;
            mqtt_publish_snapshot(snap);
        }
        break;

    case TEL_STATE_ERROR:
    default:
        // Simple recovery strategy: retry from Wi Fi after some time
        if (now_ms - g_last_retry_ms >= TELEMETRY_RETRY_MS) {
            g_last_retry_ms = now_ms;
            g_state = TEL_STATE_WIFI_CONNECTING;
        }
        break;
    }
}

void telemetry_publish_barcode(uint32_t code)
{
    if (!g_client || !g_mqtt_connected) return;

    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)code);
    mqtt_pub_str("car/telemetry/barcode", buf);
}

void telemetry_publish_obstacle(float dist_center_cm,
                                float left_extent_cm,
                                float right_extent_cm,
                                const char *chosen_path)
{
    if (!g_client || !g_mqtt_connected) return;

    char buf[64];

    /* Center distance */
    snprintf(buf, sizeof(buf), "%.1f", dist_center_cm);
    mqtt_pub_str("car/telemetry/obstacle/distance", buf);

    /* Left extent */
    snprintf(buf, sizeof(buf), "%.1f", left_extent_cm);
    mqtt_pub_str("car/telemetry/obstacle/left_extent", buf);

    /* Right extent */
    snprintf(buf, sizeof(buf), "%.1f", right_extent_cm);
    mqtt_pub_str("car/telemetry/obstacle/right_extent", buf);

    /* Chosen path text: "LEFT", "RIGHT", "BLOCKED", etc */
    if (chosen_path) {
        mqtt_pub_str("car/telemetry/obstacle/path", chosen_path);
    }
}

void telemetry_force_publish(const telemetry_snapshot_t *snap)
{
    if (!snap) return;
    if (!g_client || !g_mqtt_connected) return;

    mqtt_publish_snapshot(snap);
}
