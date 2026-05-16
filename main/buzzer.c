#include "buzzer.h"
#include "app_config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "BUZZER";

static SemaphoreHandle_t s_mutex = NULL;
static alert_type_t      s_alert = ALERT_NONE;

// ---------------------------------------------------------------------------
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
    gpio_set_level(BUZZER_GPIO, 0);

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Buzzer init: GPIO%d", BUZZER_GPIO);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Drive the buzzer for ms milliseconds.  Yields to higher-priority tasks via
// vTaskDelay — the control loop (priority 5) preempts this freely.
// ---------------------------------------------------------------------------
void buzzer_beep(uint32_t ms)
{
    gpio_set_level(BUZZER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(BUZZER_GPIO, 0);
}

// ---------------------------------------------------------------------------
void buzzer_set_alert(alert_type_t a)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_alert != a) {
        ESP_LOGI(TAG, "Alert: %d → %d", (int)s_alert, (int)a);
        s_alert = a;
    }
    xSemaphoreGive(s_mutex);
}

alert_type_t buzzer_get_alert(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    alert_type_t a = s_alert;
    xSemaphoreGive(s_mutex);
    return a;
}

// ---------------------------------------------------------------------------
// Buzzer task — priority 4 (sensor_task at 5 preempts freely mid-beep).
// One full pattern cycle per loop iteration; re-reads alert each cycle so
// alert changes take effect within one cycle.
// ---------------------------------------------------------------------------
void buzzer_task(void *arg)
{
    while (1) {
        switch (buzzer_get_alert()) {

            case ALERT_BATTERY_LOW:
                // Single short beep every 5 s
                buzzer_beep(100);
                vTaskDelay(pdMS_TO_TICKS(4900));
                break;

            case ALERT_BATTERY_CRITICAL:
                // Three rapid beeps, then silence to fill 2 s
                buzzer_beep(80); vTaskDelay(pdMS_TO_TICKS(80));
                buzzer_beep(80); vTaskDelay(pdMS_TO_TICKS(80));
                buzzer_beep(80);
                vTaskDelay(pdMS_TO_TICKS(1600));  // total ≈ 2 s
                break;

            case ALERT_HIGH_TEMP:
                // One long beep every 10 s
                buzzer_beep(500);
                vTaskDelay(pdMS_TO_TICKS(9500));
                break;

            case ALERT_NO_SOURCE:
                // Continuous 200 ms on / 200 ms off
                buzzer_beep(200);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;

            case ALERT_SENSOR_FAULT:
                // Slow 200 ms beep every 3 s — distinct from all other patterns
                buzzer_beep(200);
                vTaskDelay(pdMS_TO_TICKS(2800));
                break;

            default:  // ALERT_NONE — idle
                vTaskDelay(pdMS_TO_TICKS(200));
                break;
        }
    }
}
