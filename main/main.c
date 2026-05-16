#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "app_lcd.h"
#include "app_config.h"
#include "relay_control.h"
#include "buzzer.h"
#include "sensors.h"
#include "dht22.h"
#include "control_logic.h"
#include "pzem.h"
#include "wifi_iot.h"
#include "esp_timer.h"

static const char *TAG = "BOOT";

// ---------------------------------------------------------------------------
// Sensor + LCD task (Stage 7) — 1 s update, 4-page LCD rotation
// ---------------------------------------------------------------------------
static void sensor_task(void *arg)
{
    esp_task_wdt_add(NULL);

    sensor_data_t   d    = {0};
    control_state_t ctrl = {0};
    int             page          = 0;
    uint32_t        page_start_ms = 0;

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

            control_logic_update(&d, &ctrl);
        }

        dht22_reading_t dht;
        dht22_get_last(&dht);

        pzem_reading_t pzem;
        pzem_get_last(&pzem);

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if ((now_ms - page_start_ms) >= LCD_PAGE_DURATION_MS) {
            page = (page + 1) % 4;
            page_start_ms = now_ms;
            app_lcd_clear();
        }

        app_lcd_render_page(page, &d, &ctrl, &dht, &pzem);

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---------------------------------------------------------------------------
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Solar Monitor v0.1 starting...");

    // Log reset reason — useful for diagnosing field crashes
    static const char *const reset_names[] = {
        "UNKNOWN", "POWER_ON", "EXT", "SW", "PANIC",
        "INT_WDT", "TASK_WDT", "WDT", "DEEPSLEEP", "BROWNOUT", "SDIO"
    };
    int rr = (int)esp_reset_reason();
    ESP_LOGI(TAG, "Reset reason: %s (%d)",
             (rr >= 0 && rr <= 10) ? reset_names[rr] : "?", rr);

    // NVS — required for Wi-Fi and energy persistence
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition corrupt — erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

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

    // Control logic (must come after relay_control_init)
    ESP_ERROR_CHECK(control_logic_init());

    // PZEM-004T
    ESP_ERROR_CHECK(pzem_init());

    // Wi-Fi STA + IoT upload + OTA (creates iot_task and ota_task internally)
    ESP_ERROR_CHECK(wifi_iot_init());

    // Start tasks
    xTaskCreate(sensor_task, "sensor", 4096, NULL, 5, NULL);
    xTaskCreate(dht22_task,  "dht22",  4096, NULL, 5, NULL);
    xTaskCreate(buzzer_task, "buzzer", 2048, NULL, 4, NULL);
    xTaskCreate(pzem_task,   "pzem",   3072, NULL, 4, NULL);

    // app_main idle loop — 5 s heartbeat (within 10 s WDT window)
    esp_task_wdt_add(NULL);
    int hb = 0;
    while (1) {
        ESP_LOGI(TAG, "Heartbeat #%d  free_heap=%" PRIu32 " B",
                 ++hb, (uint32_t)esp_get_free_heap_size());
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
