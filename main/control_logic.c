#include "control_logic.h"
#include "app_config.h"
#include "relay_control.h"
#include "buzzer.h"
#include "dht22.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

static const char *TAG = "CTRL";

// Spinlock protects s_state reads (iot_task) vs writes (sensor_task).
static portMUX_TYPE      s_state_mux        = portMUX_INITIALIZER_UNLOCKED;
static control_state_t   s_state            = {.source = SRC_NONE, .sys_state = SYS_BOOT};
static charging_source_t s_pending          = SRC_NONE;
static uint32_t          s_pending_since_ms  = 0;
static system_state_t    s_last_sys_state   = SYS_BOOT;

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

const char *system_state_name(system_state_t s)
{
    switch (s) {
        case SYS_BOOT:             return "BOOT";
        case SYS_SOLAR_CHARGING:   return "SOLAR";
        case SYS_GRID_CHARGING:    return "GRID";
        case SYS_BATTERY_ACTIVE:   return "BATT";
        case SYS_BATTERY_LOW:      return "BAT_LOW";
        case SYS_BATTERY_CRITICAL: return "BAT_CRIT";
        case SYS_HIGH_TEMP:        return "HOT";
        case SYS_NO_SOURCE:        return "NO_SRC";
        case SYS_FAULT:            return "FAULT";
        default:                   return "UNKNOWN";
    }
}

// ---------------------------------------------------------------------------
static void nvs_save_sys_state(system_state_t state, uint32_t ts_ms)
{
    nvs_handle_t h;
    if (nvs_open("solar", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed (sys_state)");
        return;
    }
    nvs_set_u8(h,  "last_state",  (uint8_t)state);
    nvs_set_u32(h, "last_ts_ms",  ts_ms);
    nvs_commit(h);
    nvs_close(h);
}

// ---------------------------------------------------------------------------
esp_err_t control_logic_init(void)
{
    s_state.source         = SRC_NONE;
    s_state.relay_on       = false;
    s_state.last_change_ms = 0;
    s_state.sys_state      = SYS_BOOT;
    s_state.alert_flags    = 0;
    s_pending              = SRC_NONE;
    s_pending_since_ms     = 0;
    s_last_sys_state       = SYS_BOOT;
    relay_off();

    // Log last state before reboot (for post-mortem)
    nvs_handle_t h;
    if (nvs_open("solar", NVS_READONLY, &h) == ESP_OK) {
        uint8_t last = 0;
        uint32_t ts  = 0;
        if (nvs_get_u8(h,  "last_state", &last) == ESP_OK &&
            nvs_get_u32(h, "last_ts_ms", &ts)   == ESP_OK) {
            ESP_LOGI(TAG, "Last state before reboot: %s (%d) at uptime %"PRIu32" ms",
                     system_state_name((system_state_t)last), (int)last, ts);
        }
        nvs_close(h);
    }

    ESP_LOGI(TAG, "Control logic ready — relay OFF, hysteresis=%d ms",
             SOURCE_SWITCH_HYSTERESIS_MS);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
void control_logic_update(const sensor_data_t *s, control_state_t *out)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    // --- 1. Charging source state machine (unchanged from Stage 4) ---
    charging_source_t desired;
    if (s->solar_present && s->is_daylight) {
        desired = SRC_SOLAR;
    } else if (s->grid_present && s->bat_voltage < BATTERY_FULL_VOLTAGE) {
        desired = SRC_GRID;
    } else {
        desired = SRC_BATTERY_ACTIVE;
    }

    if (desired == s_state.source) {
        s_pending          = s_state.source;
        s_pending_since_ms = 0;
    } else {
        if (desired != s_pending) {
            s_pending          = desired;
            s_pending_since_ms = now_ms;
        } else if ((now_ms - s_pending_since_ms) >= SOURCE_SWITCH_HYSTERESIS_MS) {
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
    }

    // --- 2. Derive system_state + alert_flags ---
    system_state_t new_sys_state;
    uint32_t       new_alert_flags = 0;
    alert_type_t   buzzer_alert    = ALERT_NONE;

    dht22_reading_t dht;
    dht22_get_last(&dht);
    bool dht_fresh = dht.valid && ((now_ms - dht.last_ok_ms) <= 10000);

    if (s->is_fault) {
        new_sys_state      = SYS_FAULT;
        new_alert_flags    = ALERT_FLAG_FAULT;
        buzzer_alert       = ALERT_SENSOR_FAULT;
        relay_off();  // safety — force relay off on sensor fault
    } else {
        // Primary state from charging source
        switch (s_state.source) {
            case SRC_SOLAR:          new_sys_state = SYS_SOLAR_CHARGING; break;
            case SRC_GRID:           new_sys_state = SYS_GRID_CHARGING;  break;
            case SRC_BATTERY_ACTIVE: new_sys_state = SYS_BATTERY_ACTIVE; break;
            default:                 new_sys_state = SYS_BOOT;           break;
        }

        // Lowest priority: high temperature (flag + override if no other alert)
        if (dht_fresh && dht.temperature_c > TEMP_HIGH_C) {
            new_alert_flags |= ALERT_FLAG_HIGH_TEMP;
            buzzer_alert     = ALERT_HIGH_TEMP;
            if (new_sys_state != SYS_BATTERY_LOW &&
                new_sys_state != SYS_BATTERY_CRITICAL &&
                new_sys_state != SYS_NO_SOURCE) {
                new_sys_state = SYS_HIGH_TEMP;
            }
        }

        // Battery low — overrides high temp
        if (s->bat_voltage <= BATTERY_LOW_VOLTAGE) {
            new_alert_flags |= ALERT_FLAG_BATTERY_LOW;
            if (s_state.source == SRC_BATTERY_ACTIVE) {
                new_alert_flags |= ALERT_FLAG_NO_SOURCE;
                new_sys_state    = SYS_NO_SOURCE;
                buzzer_alert     = ALERT_NO_SOURCE;
            } else {
                new_sys_state = SYS_BATTERY_LOW;
                buzzer_alert  = ALERT_BATTERY_LOW;
            }
        }

        // Critical — highest priority, overrides everything
        if (s->bat_voltage <= BATTERY_CRITICAL_VOLTAGE) {
            new_alert_flags |= ALERT_FLAG_BATTERY_CRITICAL;
            new_sys_state    = SYS_BATTERY_CRITICAL;
            buzzer_alert     = ALERT_BATTERY_CRITICAL;
        }
    }

    // --- 3. Atomic update of sys_state + alert_flags, then snapshot ---
    portENTER_CRITICAL(&s_state_mux);
    s_state.sys_state   = new_sys_state;
    s_state.alert_flags = new_alert_flags;
    *out = s_state;
    portEXIT_CRITICAL(&s_state_mux);

    // --- 4. NVS persist on sys_state transition (outside spinlock) ---
    if (new_sys_state != s_last_sys_state) {
        ESP_LOGI(TAG, "SYS: %s → %s",
                 system_state_name(s_last_sys_state),
                 system_state_name(new_sys_state));
        s_last_sys_state = new_sys_state;
        nvs_save_sys_state(new_sys_state, now_ms);
    }

    buzzer_set_alert(buzzer_alert);
}

// ---------------------------------------------------------------------------
void control_logic_get_state(control_state_t *out)
{
    portENTER_CRITICAL(&s_state_mux);
    *out = s_state;
    portEXIT_CRITICAL(&s_state_mux);
}
