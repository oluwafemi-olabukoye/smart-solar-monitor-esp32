#include <stdio.h>
#include <stdarg.h>
#include "app_lcd.h"
#include "app_config.h"
#include "buzzer.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
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

// ---------------------------------------------------------------------------
// Page rendering helpers
// ---------------------------------------------------------------------------

static const char *bat_status(float v)
{
    if (v >= BATTERY_FULL_VOLTAGE)     return "FULL";
    if (v <= BATTERY_CRITICAL_VOLTAGE) return "CRIT";
    if (v <= BATTERY_LOW_VOLTAGE)      return "LOW";
    return "OK";
}

static const char *alert_label(alert_type_t a)
{
    switch (a) {
        case ALERT_BATTERY_LOW:      return "BAT LOW";
        case ALERT_BATTERY_CRITICAL: return "CRITICAL";
        case ALERT_HIGH_TEMP:        return "HOT";
        case ALERT_NO_SOURCE:        return "NO SRC";
        default:                     return "NONE";
    }
}

// ---------------------------------------------------------------------------
// Page 0 — Power Overview
//   Row 0: "BAT  13.91V  OK     "
//   Row 1: "SOLAR  PRESENT      "
//   Row 2: "GRID   PRESENT      "
//   Row 3: "SRC SOLAR  RELAY OFF"
// ---------------------------------------------------------------------------
static void render_page0(const sensor_data_t *s, const control_state_t *c)
{
    app_lcd_printf(0, 0, "BAT %5.2fV  %-4s    ",
                   s->bat_voltage, bat_status(s->bat_voltage));
    app_lcd_printf(1, 0, "SOLAR  %-13s",
                   s->solar_present ? "PRESENT" : "ABSENT");
    app_lcd_printf(2, 0, "GRID   %-13s",
                   s->grid_present  ? "PRESENT" : "ABSENT");
    app_lcd_printf(3, 0, "SRC %-6s RELAY %-3s",
                   charging_source_name(c->source),
                   c->relay_on ? "ON" : "OFF");
}

// ---------------------------------------------------------------------------
// Page 1 — Sources Detail
//   Row 0: "Solar V:  13.49V    "
//   Row 1: "Grid  V:  13.95V    "
//   Row 2: "LDR raw: 2384 DAY   "
//   Row 3: "Relay: OFF  (BATT)  "
// ---------------------------------------------------------------------------
static void render_page1(const sensor_data_t *s, const control_state_t *c)
{
    app_lcd_printf(0, 0, "Solar V: %6.2fV     ", s->solar_voltage);
    app_lcd_printf(1, 0, "Grid  V: %6.2fV     ", s->grid_voltage);
    app_lcd_printf(2, 0, "LDR raw: %4d %-3s   ",
                   s->ldr_raw, s->is_daylight ? "DAY" : "NGT");
    app_lcd_printf(3, 0, "Relay: %-3s  (%s)           ",
                   c->relay_on ? "ON" : "OFF",
                   charging_source_name(c->source));
}

// ---------------------------------------------------------------------------
// Page 2 — Environment
//   Row 0: "Temp:  30.9 C   OK  "
//   Row 1: "Humid: 75.5 %       "
//   Row 2: "Alert: NONE         "
//   Row 3: "Uptime: 0d 00:28    "
// ---------------------------------------------------------------------------
static void render_page2(const dht22_reading_t *d)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    bool stale = !d->valid || ((now_ms - d->last_ok_ms) > 10000);

    app_lcd_printf(0, 0, "Temp: %5.1f C   %-3s  ",
                   d->temperature_c, stale ? "OLD" : "OK");
    app_lcd_printf(1, 0, "Humid: %4.1f %%       ", d->humidity_pct);
    app_lcd_printf(2, 0, "Alert: %-8s      ", alert_label(buzzer_get_alert()));

    uint64_t uptime_s = (uint64_t)(esp_timer_get_time() / 1000000LL);
    uint32_t days  = (uint32_t)(uptime_s / 86400);
    uint32_t hours = (uint32_t)((uptime_s % 86400) / 3600);
    uint32_t mins  = (uint32_t)((uptime_s % 3600) / 60);
    app_lcd_printf(3, 0, "Uptime:%2ud %02u:%02u    ", days, hours, mins);
}

// ---------------------------------------------------------------------------
// Page 3 — AC Load (PZEM)
//   Row 0: "AC V:  223.0 V      "
//   Row 1: "AC I:    0.089 A    "
//   Row 2: "Power:   19.2 W     "
//   Row 3: "Energy:  0.00 kWh   "
// ---------------------------------------------------------------------------
static void render_page3(const pzem_reading_t *p)
{
    app_lcd_printf(0, 0, "AC V: %6.1f V      ", p->voltage);
    app_lcd_printf(1, 0, "AC I: %8.3f A    ", p->current);
    app_lcd_printf(2, 0, "Power: %6.1f W     ", p->power);
    app_lcd_printf(3, 0, "Energy: %5.2f kWh  ", p->energy / 1000.0f);
}

// ---------------------------------------------------------------------------
void app_lcd_render_page(int page_idx,
                         const sensor_data_t   *s,
                         const control_state_t *c,
                         const dht22_reading_t *d,
                         const pzem_reading_t  *p)
{
    switch (page_idx % 4) {
        case 0: render_page0(s, c);  break;
        case 1: render_page1(s, c);  break;
        case 2: render_page2(d);     break;
        case 3: render_page3(p);     break;
    }
}
