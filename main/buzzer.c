#include "buzzer.h"
#include "app_config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "BUZZER";

esp_err_t buzzer_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << BUZZER_GPIO),
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio_config failed");

    gpio_set_level(BUZZER_GPIO, 0);  // silent at startup
    ESP_LOGI(TAG, "Buzzer init: GPIO%d", BUZZER_GPIO);
    return ESP_OK;
}

void buzzer_beep(uint32_t ms)
{
    gpio_set_level(BUZZER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(BUZZER_GPIO, 0);
}
