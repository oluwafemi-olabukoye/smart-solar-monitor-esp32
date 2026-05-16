#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "app_lcd.h"
#include "relay_control.h"
#include "buzzer.h"
#include "sensors.h"
#include "dht22.h"
#include "esp_timer.h"

static const char *TAG = "BOOT";

// ---------------------------------------------------------------------------
// GPIO test task (Stage 1) — relay 3 s cycle, buzzer every 5 s
// ---------------------------------------------------------------------------
static void gpio_test_task(void *arg)
{
    TickType_t buzzer_last = xTaskGetTickCount();
    while (1) {
        relay_on();
        vTaskDelay(pdMS_TO_TICKS(1500));
        relay_off();
        if ((xTaskGetTickCount() - buzzer_last) >= pdMS_TO_TICKS(5000)) {
            buzzer_beep(100);
            buzzer_last = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// ---------------------------------------------------------------------------
// Sensor + LCD task (Stage 3) — 1 s update rate
//
// LCD layout (20 × 4):
//   Row 0: "BAT:  xx.xxV        "  (width 20)
//   Row 1: "SOL:xx.xx GRD:xx.xxV"  (width 20)
//   Row 2: "T xx.xC H xx% OK    "  DHT22, fresh or OLD if stale >10 s
//   Row 3: "Stage 3: DHT22      "  (static)
//
// Rows are written in-place each second — no lcd_clear() — to avoid flicker.
// ---------------------------------------------------------------------------
static void sensor_task(void *arg)
{
    sensor_data_t d = {0};
    app_lcd_print(3, 0, "Stage 3: DHT22      ");   // static row, write once

    while (1) {
        if (sensors_read_all(&d) == ESP_OK) {
            ESP_LOGI("SENSOR",
                "BAT raw=%d mv=%.0f V=%.2f  "
                "SOLAR raw=%d V=%.2f  "
                "GRID raw=%d V=%.2f  "
                "LDR raw=%d  DAY=%s",
                d.bat_raw,   d.bat_mv,   d.bat_voltage,
                d.solar_raw, d.solar_voltage,
                d.grid_raw,  d.grid_voltage,
                d.ldr_raw,
                d.is_daylight ? "YES" : "NO");

            // LCD row 0: "BAT:  13.91V        " (20 chars)
            app_lcd_printf(0, 0, "BAT:%7.2fV        ", d.bat_voltage);
            // LCD row 1: "SOL:13.54 GRD:13.96V" (20 chars)
            app_lcd_printf(1, 0, "SOL:%5.2f GRD:%5.2fV",
                           d.solar_voltage, d.grid_voltage);
        }

        // LCD row 2: "T 28.4C H 65% OK    " or "... OLD    " (20 chars)
        dht22_reading_t dht;
        dht22_get_last(&dht);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        bool stale = !dht.valid || ((now_ms - dht.last_ok_ms) > 10000);
        app_lcd_printf(2, 0, "T%5.1fC H%2.0f%% %-3s    ",
                       dht.temperature_c, dht.humidity_pct,
                       stale ? "OLD" : "OK");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---------------------------------------------------------------------------
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Solar Monitor v0.1 starting...");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size = 0;
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGW(TAG, "Could not read flash size");
    }
    ESP_LOGI(TAG, "Chip model %d | cores: %d | rev: %d | flash: %" PRIu32 " MB",
             (int)chip_info.model, (int)chip_info.cores,
             (int)chip_info.revision, flash_size / (1024 * 1024));

    // GPIO peripherals
    ESP_ERROR_CHECK(relay_control_init());
    ESP_ERROR_CHECK(buzzer_init());

    // I2C — scan then LCD
    i2c_scan_bus();
    ESP_ERROR_CHECK(app_lcd_init());

    // ADC sensors
    ESP_ERROR_CHECK(sensors_init());

    // DHT22
    ESP_ERROR_CHECK(dht22_init());

    // Start tasks
    xTaskCreate(gpio_test_task, "gpio_test",  2048, NULL, 4, NULL);
    xTaskCreate(sensor_task,    "sensor",     4096, NULL, 5, NULL);
    xTaskCreate(dht22_task,     "dht22",      4096, NULL, 5, NULL);

    // app_main idle loop (lowest priority consumer)
    int hb = 0;
    while (1) {
        ESP_LOGI(TAG, "Heartbeat #%d", ++hb);
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
