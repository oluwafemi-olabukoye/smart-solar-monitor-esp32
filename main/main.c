#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "app_lcd.h"
#include "relay_control.h"
#include "buzzer.h"

static const char *TAG = "BOOT";

// ---------------------------------------------------------------------------
// GPIO test task — relay 3 s cycle, buzzer every 5 s
// ---------------------------------------------------------------------------
static void gpio_test_task(void *arg)
{
    TickType_t buzzer_last = xTaskGetTickCount();

    while (1) {
        relay_on();
        ESP_LOGI("GPIO_TEST", "RELAY ON");
        vTaskDelay(pdMS_TO_TICKS(1500));

        relay_off();
        ESP_LOGI("GPIO_TEST", "RELAY OFF");

        if ((xTaskGetTickCount() - buzzer_last) >= pdMS_TO_TICKS(5000)) {
            buzzer_beep(100);
            ESP_LOGI("GPIO_TEST", "BUZZER BEEP");
            buzzer_last = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(1500));  // complete the 3 s relay cycle
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

    // --- GPIO peripherals (no I2C dependency) ---
    ESP_ERROR_CHECK(relay_control_init());
    ESP_ERROR_CHECK(buzzer_init());

    // --- I2C: scan with new API, then init LCD with legacy API ---
    i2c_scan_bus();
    ESP_ERROR_CHECK(app_lcd_init());

    // --- Step 4: LCD test pattern ---
    app_lcd_clear();
    app_lcd_print(0, 0, "Solar Monitor v0.1");
    app_lcd_print(1, 0, "I2C OK  LCD OK");
    app_lcd_print(2, 0, "Stage 1: GPIO Test");
    app_lcd_print(3, 0, "Relay/Buzz cycling");

    // --- Start relay/buzzer cycling task ---
    xTaskCreate(gpio_test_task, "gpio_test", 2048, NULL, 5, NULL);

    // --- Heartbeat (slow, background) ---
    int hb = 0;
    while (1) {
        ESP_LOGI(TAG, "Heartbeat #%d", ++hb);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
