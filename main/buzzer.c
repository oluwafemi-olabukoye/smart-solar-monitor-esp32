#include "buzzer.h"
#include "esp_log.h"

static const char *TAG = "BUZZER";

esp_err_t buzzer_init(void)
{
    ESP_LOGI(TAG, "buzzer_init (stub)");
    return ESP_OK;
}
