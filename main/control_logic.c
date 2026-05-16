#include "control_logic.h"
#include "app_config.h"
#include "relay_control.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "CTRL";

// Internal state — only ever touched from the single sensor_task caller.
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

            s_state.source        = desired;
            s_state.relay_on      = (desired == SRC_GRID);
            s_state.last_change_ms = now_ms;

            if (desired == SRC_GRID) relay_on(); else relay_off();

            ESP_LOGI(TAG,
                "SOURCE: %s → %s  (bat=%.2fV solar=%.2fV grid=%.2fV day=%s)",
                charging_source_name(old), charging_source_name(desired),
                s->bat_voltage, s->solar_voltage, s->grid_voltage,
                s->is_daylight ? "YES" : "NO");
        }
        // else: pending is accumulating — do nothing yet
    }

    *out = s_state;
}
