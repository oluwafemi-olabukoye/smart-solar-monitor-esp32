#pragma once
#include <stdint.h>
#include "esp_err.h"

void      i2c_scan_bus(void);
esp_err_t app_lcd_init(void);
void      app_lcd_clear(void);
void      app_lcd_print(uint8_t row, uint8_t col, const char *text);
void      app_lcd_printf(uint8_t row, uint8_t col, const char *fmt, ...);
