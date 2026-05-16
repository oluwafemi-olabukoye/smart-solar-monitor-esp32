#include "control_logic.h"
#include "app_config.h"
#include "relay_control.h"
#include "buzzer.h"
#include "dht22.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "CTRL";

// Spinlock protects s_state writes (sensor_task) vs reads (iot_task).
static portMUX_TYPE      s_state_mux       = portMUX_INITIALIZER_UNLOCKED;
static control_state_t   s_state           = {.source = SRC_NONE};
static charging_source_t s_pending         = SRC_NONE;
static uint32_t          s_pending_since_ms = 0;

// ---------------------------------------------------------------------------
const char *charging_source_name(charging_source_t src)
{
    switch (src) {
        case SRC_SOLAR:          return "SOLAR";
        case SRC_GRID:           return "GRID";
        case SRC_BATTERY_ACTIVE: return "BATT";
        default:                 return "NONE";
    }
}

// ---------------------------------------------------------------------------
esp_err_t control_logic_init(void)
{
    s_state.source        = SRC_NONE;
    s_state.relay_on      = false;
    s_state.last_change_ms = 0;
    s_pending             = SRC_NONE;
    s_pending_since_ms    = 0;
    relay_off();
    ESP_LOGI(TAG, "Control logic ready — relay OFF, waiting %d ms hysteresis",
             SOURCE_SWITCH_HYSTERESIS_MS);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
void control_logic_update(const sensor_data_t *s, control_state_t *out)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    // Priority order
    charging_source_t desired;
    if (s->solar_present && s->is_daylight) {
        desired = SRC_SOLAR;
    } else if (s->grid_present && s->bat_voltage < BATTERY_FULL_VOLTAGE) {
        desired = SRC_GRID;
    } else {
        desired = SRC_BATTERY_ACTIVE;
    }

    if (desired == s_state.source) {
        // Steady state — clear any in-flight pending transition
        s_pending          = s_state.source;
        s_pending_since_ms = 0;
    } else {
        if (desired != s_pending) {
            // New candidate — restart timer
            s_pending          = desired;
            s_pending_since_ms = now_ms;
        } else if ((now_ms - s_pending_since_ms) >= SOURCE_SWITCH_HYSTERESIS_MS) {
            // Hysteresis elapsed — commit
            charging_source_t old = s_state.source;

            if (desired == SRC_GRID) relay_on(); else relay_off();

            ESP_LOGI(TAG,
                "SOURCE: %s → %s  (bat=%.2fV solar=%.2fV grid=%.2fV day=%s)",
                charging_source_name(old), charging_source_name(desired),
                s->bat_voltage, s->solar_voltage, s->grid_voltage,
                s->is_daylight ? "YES" : "NO");

            portENTER_CRITICAL(&s_state_mux);
            s_state.source         = desired;
            s_state.relay_on       = (desired == SRC_GRID);
            s_state.last_change_ms = now_ms;
            portEXIT_CRITICAL(&s_state_mux);
        }
        // else: pending is accumulating — do nothing yet
    }

    portENTER_CRITICAL(&s_state_mux);
    *out = s_state;
    portEXIT_CRITICAL(&s_state_mux);

    // --- Alert determination (evaluated every call, highest priority wins) ---
    alert_type_t alert = ALERT_NONE;

    // Lowest priority: high temperature (only when DHT22 data is fresh)
    dht22_reading_t dht;
    dht22_get_last(&dht);
    bool dht_fresh = dht.valid && ((now_ms - dht.last_ok_ms) <= 10000);
    if (dht_fresh && dht.temperature_c > TEMP_HIGH_C) {
        alert = ALERT_HIGH_TEMP;
    }

    // Battery below low threshold — pick between LOW and NO_SOURCE
    if (s->bat_voltage <= BATTERY_LOW_VOLTAGE) {
        alert = (s_state.source == SRC_BATTERY_ACTIVE)
                    ? ALERT_NO_SOURCE
                    : ALERT_BATTERY_LOW;
    }

    // Critical voltage overrides everything
    if (s->bat_voltage <= BATTERY_CRITICAL_VOLTAGE) {
        alert = ALERT_BATTERY_CRITICAL;
    }

    buzzer_set_alert(alert);
}

// ---------------------------------------------------------------------------
void control_logic_get_state(control_state_t *out)
{
    portENTER_CRITICAL(&s_state_mux);
    *out = s_state;
    portEXIT_CRITICAL(&s_state_mux);
}
