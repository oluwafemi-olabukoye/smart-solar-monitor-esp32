#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float    voltage;      // V
    float    current;      // A
    float    power;        // W
    float    energy;       // Wh
    float    frequency;    // Hz
    float    pf;           // power factor 0.00–1.00
    bool     valid;
    uint32_t last_ok_ms;
} pzem_reading_t;

esp_err_t pzem_init(void);
esp_err_t pzem_read(pzem_reading_t *out);
esp_err_t pzem_reset_energy(void);

// Thread-safe snapshot of the last successful read
void pzem_get_last(pzem_reading_t *out);

void pzem_task(void *arg);
