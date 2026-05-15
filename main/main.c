#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

static const char *TAG = "BOOT";

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
             (int)chip_info.model,
             (int)chip_info.cores,
             (int)chip_info.revision,
             flash_size / (1024 * 1024));

    int heartbeat = 0;
    while (1) {
        ESP_LOGI(TAG, "Heartbeat #%d", ++heartbeat);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
