#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    int   bat_raw;   float bat_mv;   float bat_voltage;
    int   solar_raw; float solar_mv; float solar_voltage;
    int   grid_raw;  float grid_mv;  float grid_voltage;
    int   ldr_raw;
    bool  is_daylight;
    bool  solar_present;
    bool  grid_present;
} sensor_data_t;

esp_err_t sensors_init(void);
esp_err_t sensors_read_all(sensor_data_t *out);

// Thread-safe snapshot of the last successful sensors_read_all() result.
// Safe to call from any task; does not touch ADC hardware.
void sensors_get_last(sensor_data_t *out);
