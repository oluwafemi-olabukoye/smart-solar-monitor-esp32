#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sensors.h"

typedef enum {
    SRC_NONE = 0,
    SRC_SOLAR,
    SRC_GRID,
    SRC_BATTERY_ACTIVE,
} charging_source_t;

typedef struct {
    charging_source_t source;
    bool              relay_on;
    uint32_t          last_change_ms;
} control_state_t;

esp_err_t control_logic_init(void);

// Evaluate sensor data, apply hysteresis, drive relay if needed.
// Fills *out with the current committed state.
void control_logic_update(const sensor_data_t *s, control_state_t *out);

// Short label for LCD / logs ("SOLAR", "GRID", "BATT", "NONE")
const char *charging_source_name(charging_source_t src);
