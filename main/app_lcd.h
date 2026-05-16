#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "sensors.h"
#include "control_logic.h"
#include "dht22.h"
#include "pzem.h"

void      i2c_scan_bus(void);
esp_err_t app_lcd_init(void);
void      app_lcd_clear(void);
void      app_lcd_print(uint8_t row, uint8_t col, const char *text);
void      app_lcd_printf(uint8_t row, uint8_t col, const char *fmt, ...);

// Render one of four rotating pages onto the 20×4 LCD.
// page_idx is taken modulo 4.
void app_lcd_render_page(int page_idx,
                         const sensor_data_t   *s,
                         const control_state_t *c,
                         const dht22_reading_t *d,
                         const pzem_reading_t  *p);
