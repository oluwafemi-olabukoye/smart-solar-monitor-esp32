#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sensors.h"

// ---------------------------------------------------------------------------
// Charging source (internal state machine result)
// ---------------------------------------------------------------------------
typedef enum {
    SRC_NONE = 0,
    SRC_SOLAR,
    SRC_GRID,
    SRC_BATTERY_ACTIVE,
} charging_source_t;

// ---------------------------------------------------------------------------
// Unified system state — one dominant state at a time
// ---------------------------------------------------------------------------
typedef enum {
    SYS_BOOT = 0,
    SYS_SOLAR_CHARGING,
    SYS_GRID_CHARGING,
    SYS_BATTERY_ACTIVE,
    SYS_BATTERY_LOW,
    SYS_BATTERY_CRITICAL,
    SYS_HIGH_TEMP,
    SYS_NO_SOURCE,
    SYS_FAULT,          // sensor sanity failure — relay forced off
} system_state_t;

// ---------------------------------------------------------------------------
// Alert bitmask — multiple conditions can be active simultaneously
// ---------------------------------------------------------------------------
#define ALERT_FLAG_BATTERY_LOW      (1u << 0)
#define ALERT_FLAG_BATTERY_CRITICAL (1u << 1)
#define ALERT_FLAG_HIGH_TEMP        (1u << 2)
#define ALERT_FLAG_NO_SOURCE        (1u << 3)
#define ALERT_FLAG_FAULT            (1u << 4)

// ---------------------------------------------------------------------------
// Control state (readable by any task via control_logic_get_state)
// ---------------------------------------------------------------------------
typedef struct {
    charging_source_t source;
    bool              relay_on;
    uint32_t          last_change_ms;
    system_state_t    sys_state;    // dominant system state
    uint32_t          alert_flags;  // bitmask of ALERT_FLAG_xxx
} control_state_t;

esp_err_t   control_logic_init(void);
void        control_logic_update(const sensor_data_t *s, control_state_t *out);
void        control_logic_get_state(control_state_t *out);

const char *charging_source_name(charging_source_t src);
const char *system_state_name(system_state_t s);
