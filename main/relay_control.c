#include "relay_control.h"
#include "app_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "RELAY";

esp_err_t relay_control_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << RELAY_GPIO),
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio_config failed");

    // Safe default: relay OFF before any load is connected
    gpio_set_level(RELAY_GPIO, !RELAY_ACTIVE_LEVEL);

    ESP_LOGI(TAG, "Relay init: GPIO%d  active=%s  state=OFF",
             RELAY_GPIO, RELAY_ACTIVE_LEVEL ? "HIGH" : "LOW");
    return ESP_OK;
}

void relay_on(void)
{
    gpio_set_level(RELAY_GPIO, RELAY_ACTIVE_LEVEL);
}

void relay_off(void)
{
    gpio_set_level(RELAY_GPIO, !RELAY_ACTIVE_LEVEL);
}
