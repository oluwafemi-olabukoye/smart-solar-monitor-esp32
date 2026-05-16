#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float    temperature_c;
    float    humidity_pct;
    bool     valid;         // true once at least one read has succeeded
    uint32_t last_ok_ms;    // esp_timer_get_time()/1000 at last good read
} dht22_reading_t;

esp_err_t dht22_init(void);
esp_err_t dht22_read(dht22_reading_t *out);

// Thread-safe snapshot of the last successful read.
// Returns zeroed struct with valid=false before any good read.
void dht22_get_last(dht22_reading_t *out);

void dht22_task(void *arg);
