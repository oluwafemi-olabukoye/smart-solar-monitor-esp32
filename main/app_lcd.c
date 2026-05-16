#include <stdio.h>
#include <stdarg.h>
#include "app_lcd.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "i2c_lcd_pcf8574.h"

static const char *TAG = "LCD";

// Single bus handle shared between scanner and LCD device.
// Created in i2c_scan_bus(), reused in app_lcd_init() — never deleted.
static i2c_master_bus_handle_t  s_bus     = NULL;
static i2c_master_dev_handle_t  s_lcd_dev = NULL;
static i2c_lcd_pcf8574_handle_t s_lcd;

// ---------------------------------------------------------------------------
// I2C bus scanner — new i2c_master API
// Called from main.c BEFORE app_lcd_init().
// Creates and retains s_bus so app_lcd_init can add the LCD device to it.
// ---------------------------------------------------------------------------
void i2c_scan_bus(void)
{
    ESP_LOGI(TAG, "I2C scan: SDA=GPIO%d  SCL=GPIO%d  freq=%dHz",
             I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);

    i2c_master_bus_config_t bus_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = I2C_SCL_GPIO,
        .sda_io_num                   = I2C_SDA_GPIO,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    int found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_bus, addr, 10) == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "  No I2C devices found — check wiring and pull-ups");
    }
    // s_bus stays open; app_lcd_init() will add the LCD device to it
}

// ---------------------------------------------------------------------------
// LCD init — adds device to the existing bus, then starts the library
// ---------------------------------------------------------------------------
esp_err_t app_lcd_init(void)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = LCD_I2C_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(s_bus, &dev_cfg, &s_lcd_dev),
        TAG, "bus_add_device failed — is the I2C address correct?");

    lcd_init(&s_lcd, LCD_I2C_ADDR, s_lcd_dev);
    lcd_begin(&s_lcd, LCD_COLS, LCD_ROWS);
    lcd_set_backlight(&s_lcd, 1);

    ESP_LOGI(TAG, "LCD ready: addr=0x%02X  %dx%d", LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
    return ESP_OK;
}

void app_lcd_clear(void)
{
    lcd_clear(&s_lcd);
}

void app_lcd_print(uint8_t row, uint8_t col, const char *text)
{
    lcd_set_cursor(&s_lcd, col, row);
    lcd_print(&s_lcd, text);
}

void app_lcd_printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    char buf[LCD_COLS + 1];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    app_lcd_print(row, col, buf);
}
