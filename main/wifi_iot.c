#include "wifi_iot.h"
#include "app_config.h"
#include "sensors.h"
#include "control_logic.h"
#include "dht22.h"
#include "pzem.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_task_wdt.h"
#include "cJSON.h"
#include <string.h>
#include <inttypes.h>

#ifdef CONFIG_OTA_ENABLED
#include "esp_https_ota.h"
#endif

static const char *TAG = "WIFI";

// ---------------------------------------------------------------------------
// Wi-Fi state
// ---------------------------------------------------------------------------
#define WIFI_CONNECTED_BIT    BIT0

#define RECONNECT_BASE_MS     1000
#define RECONNECT_MAX_MS      60000

static EventGroupHandle_t  s_wifi_eg        = NULL;
static bool                s_connected      = false;
static esp_timer_handle_t  s_reconnect_timer = NULL;
static int                 s_retry_count    = 0;

// ---------------------------------------------------------------------------
// Task intervals
// ---------------------------------------------------------------------------
#define IOT_UPLOAD_MS        30000   // 30 s
#define ENERGY_PERSIST_MS   300000   // 5 min
#define OTA_CHECK_MS        600000   // 10 min

// ---------------------------------------------------------------------------
// NVS energy persistence
// ---------------------------------------------------------------------------
#define NVS_NAMESPACE  "solar"
#define NVS_KEY_ENERGY "energy_wh"

static void nvs_save_energy(float energy_wh)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, NVS_KEY_ENERGY, (uint32_t)energy_wh);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "NVS energy saved: %.0f Wh", energy_wh);
}

static float nvs_load_energy(void)
{
    nvs_handle_t h;
    uint32_t stored = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, NVS_KEY_ENERGY, &stored);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "NVS energy loaded: %" PRIu32 " Wh", stored);
    return (float)stored;
}

// ---------------------------------------------------------------------------
// HTTP POST
// ---------------------------------------------------------------------------
static void http_post_json(const char *url, const char *body, int body_len)
{
    esp_http_client_config_t cfg = {
        .url                = url,
        .method             = HTTP_METHOD_POST,
        .timeout_ms         = 8000,
        .crt_bundle_attach  = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) { ESP_LOGW(TAG, "http_client_init failed"); return; }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, body_len);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status < 200 || status >= 300) {
            ESP_LOGW(TAG, "HTTP %d from %s", status, url);
        } else {
            ESP_LOGI(TAG, "HTTP POST OK (%d bytes, HTTP %d)", body_len, status);
        }
    } else {
        ESP_LOGW(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

// ---------------------------------------------------------------------------
// IoT upload task — JSON POST every 30 s, energy NVS every 5 min
// ---------------------------------------------------------------------------
void iot_task(void *arg)
{
    esp_task_wdt_add(NULL);

    uint32_t last_upload_ms  = 0;
    uint32_t last_persist_ms = 0;

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        // --- Energy NVS persistence (every 5 min, regardless of Wi-Fi) ---
        if ((now_ms - last_persist_ms) >= ENERGY_PERSIST_MS) {
            last_persist_ms = now_ms;
            pzem_reading_t p = {0};
            pzem_get_last(&p);
            if (p.valid) {
                nvs_save_energy(p.energy);
            }
        }

        // --- JSON upload every 30 s when connected ---
        if (s_connected && (now_ms - last_upload_ms) >= IOT_UPLOAD_MS) {
            last_upload_ms = now_ms;

            sensor_data_t   s = {0};
            control_state_t c = {0};
            dht22_reading_t d = {0};
            pzem_reading_t  p = {0};

            sensors_get_last(&s);
            control_logic_get_state(&c);
            dht22_get_last(&d);
            pzem_get_last(&p);

            cJSON *root = cJSON_CreateObject();
            cJSON_AddNumberToObject(root, "uptime_s",
                                    (double)(esp_timer_get_time() / 1000000LL));
            cJSON_AddStringToObject(root, "sys_state",   system_state_name(c.sys_state));
            cJSON_AddNumberToObject(root, "alert_flags", (double)c.alert_flags);

            cJSON *bat = cJSON_AddObjectToObject(root, "battery");
            cJSON_AddNumberToObject(bat, "voltage_v", (double)s.bat_voltage);
            cJSON_AddStringToObject(bat, "source",    charging_source_name(c.source));
            cJSON_AddBoolToObject  (bat, "relay_on",  c.relay_on);

            cJSON *solar = cJSON_AddObjectToObject(root, "solar");
            cJSON_AddNumberToObject(solar, "voltage_v", (double)s.solar_voltage);
            cJSON_AddBoolToObject  (solar, "present",   s.solar_present);
            cJSON_AddBoolToObject  (solar, "daylight",  s.is_daylight);

            cJSON *grid = cJSON_AddObjectToObject(root, "grid");
            cJSON_AddNumberToObject(grid, "voltage_v", (double)s.grid_voltage);
            cJSON_AddBoolToObject  (grid, "present",   s.grid_present);

            cJSON *env = cJSON_AddObjectToObject(root, "environment");
            cJSON_AddNumberToObject(env, "temperature_c", (double)d.temperature_c);
            cJSON_AddNumberToObject(env, "humidity_pct",  (double)d.humidity_pct);
            cJSON_AddBoolToObject  (env, "dht_valid",     d.valid);

            cJSON *ac = cJSON_AddObjectToObject(root, "ac_load");
            cJSON_AddNumberToObject(ac, "voltage_v",    (double)p.voltage);
            cJSON_AddNumberToObject(ac, "current_a",    (double)p.current);
            cJSON_AddNumberToObject(ac, "power_w",      (double)p.power);
            cJSON_AddNumberToObject(ac, "energy_wh",    (double)p.energy);
            cJSON_AddNumberToObject(ac, "frequency_hz", (double)p.frequency);
            cJSON_AddNumberToObject(ac, "power_factor", (double)p.pf);

            char *body = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);

            if (body) {
                http_post_json(CONFIG_IOT_ENDPOINT, body, (int)strlen(body));
                free(body);
            }
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---------------------------------------------------------------------------
// OTA task — checks for new firmware every 10 min (build-time opt-in)
// ---------------------------------------------------------------------------
void ota_task(void *arg)
{
#ifdef CONFIG_OTA_ENABLED
    uint32_t last_ota_ms = 0;
    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (s_connected && (now_ms - last_ota_ms) >= OTA_CHECK_MS) {
            last_ota_ms = now_ms;
            ESP_LOGI(TAG, "OTA check → %s", CONFIG_OTA_URL);

            esp_http_client_config_t http_cfg = {
                .url            = CONFIG_OTA_URL,
                .timeout_ms     = 15000,
                .keep_alive_enable = true,
            };
            esp_https_ota_config_t ota_cfg = { .http_config = &http_cfg };

            esp_err_t err = esp_https_ota(&ota_cfg);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "OTA success — restarting in 1 s");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            } else {
                ESP_LOGW(TAG, "OTA skipped/failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(60000));
    }
#else
    vTaskDelete(NULL);
#endif
}

// ---------------------------------------------------------------------------
// Wi-Fi reconnect helpers
// ---------------------------------------------------------------------------
static void reconnect_timer_cb(void *arg)
{
    esp_wifi_connect();
}

static void schedule_reconnect(void)
{
    // Exponential backoff: 1 s, 2 s, 4 s … capped at 60 s
    uint32_t delay_ms = (uint32_t)(RECONNECT_BASE_MS) << s_retry_count;
    if (delay_ms == 0 || delay_ms > RECONNECT_MAX_MS) delay_ms = RECONNECT_MAX_MS;
    if (s_retry_count < 6) s_retry_count++;
    ESP_LOGI(TAG, "Reconnect in %" PRIu32 " ms (attempt %d)", delay_ms, s_retry_count);
    esp_timer_start_once(s_reconnect_timer, (uint64_t)delay_ms * 1000ULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            s_connected = false;
            xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);
            schedule_reconnect();
            break;
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_connected   = true;
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

// ---------------------------------------------------------------------------
bool wifi_is_connected(void) { return s_connected; }

// ---------------------------------------------------------------------------
esp_err_t wifi_iot_init(void)
{
    float saved_energy = nvs_load_energy();
    (void)saved_energy;  // informational only; PZEM holds its own counter

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    s_wifi_eg = xEventGroupCreate();
    if (!s_wifi_eg) return ESP_ERR_NO_MEM;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    esp_timer_create_args_t timer_args = {
        .callback = reconnect_timer_cb,
        .name     = "wifi_recon",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_reconnect_timer));

    wifi_config_t wifi_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strlcpy((char *)wifi_cfg.sta.ssid,     CONFIG_WIFI_SSID,
            sizeof(wifi_cfg.sta.ssid));
    strlcpy((char *)wifi_cfg.sta.password, CONFIG_WIFI_PASSWORD,
            sizeof(wifi_cfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA started: SSID=\"%s\"  endpoint=%s",
             CONFIG_WIFI_SSID, CONFIG_IOT_ENDPOINT);

    xTaskCreate(iot_task, "iot", 8192, NULL, 3, NULL);
    xTaskCreate(ota_task, "ota", 8192, NULL, 3, NULL);

    return ESP_OK;
}
