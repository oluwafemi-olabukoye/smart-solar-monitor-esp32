#include <string.h>
#include "dht22.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "DHT22";

static SemaphoreHandle_t  s_mutex   = NULL;
static portMUX_TYPE       s_spinlock = portMUX_INITIALIZER_UNLOCKED;

static dht22_reading_t g_last = {0};

// ---------------------------------------------------------------------------
// Bit-banged one-wire read
// Called with interrupts ENABLED; only the 4-5 ms data burst is in a
// critical section.
// ---------------------------------------------------------------------------

// Returns µs waited (0-based), or -1 on timeout.
// Waits until gpio_get_level(DHT22_GPIO) == target_level.
static inline int wait_for(int target_level, int timeout_us)
{
    for (int i = 0; i < timeout_us; i++) {
        if (gpio_get_level(DHT22_GPIO) == target_level) return i;
        esp_rom_delay_us(1);
    }
    return -1;
}

static esp_err_t read_raw(uint8_t data[5])
{
    // --- Host start pulse (20 ms LOW) — outside critical section ---
    gpio_set_direction(DHT22_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT22_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // --- Critical section: timing-sensitive bus turnaround + 40 data bits ---
    portENTER_CRITICAL(&s_spinlock);

    // Release line; external 10 kΩ pull-up brings it HIGH
    gpio_set_direction(DHT22_GPIO, GPIO_MODE_INPUT);
    esp_rom_delay_us(40);

    // DHT response: LOW ~80 µs then HIGH ~80 µs, then data starts (LOW)
    if (wait_for(0, 200) < 0) goto timeout;   // wait for response LOW
    if (wait_for(1, 200) < 0) goto timeout;   // wait for response HIGH
    if (wait_for(0, 200) < 0) goto timeout;   // wait for first bit preamble LOW

    // 40 data bits: each bit = 50 µs LOW preamble + HIGH (26-28 µs='0', ~70 µs='1')
    memset(data, 0, 5);
    for (int i = 0; i < 40; i++) {
        if (wait_for(1, 200) < 0) goto timeout;   // end of 50 µs LOW preamble
        int hi = 0;
        while (gpio_get_level(DHT22_GPIO) == 1) {
            if (++hi > 200) goto timeout;
            esp_rom_delay_us(1);
        }
        data[i / 8] <<= 1;
        if (hi > 40) data[i / 8] |= 1;           // >40 µs HIGH → bit '1'
    }

    portEXIT_CRITICAL(&s_spinlock);
    return ESP_OK;

timeout:
    portEXIT_CRITICAL(&s_spinlock);
    return ESP_ERR_TIMEOUT;
}

// ---------------------------------------------------------------------------
esp_err_t dht22_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask  = 1ULL << DHT22_GPIO,
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,    // rely on external 10 kΩ
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config");

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "DHT22 wiring: VCC=3.3V, DATA=GPIO%d with 10kΩ pull-up to 3.3V, GND=GND",
             DHT22_GPIO);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
esp_err_t dht22_read(dht22_reading_t *out)
{
    uint8_t raw[5];
    ESP_RETURN_ON_ERROR(read_raw(raw), TAG, "read_raw");

    uint8_t csum = raw[0] + raw[1] + raw[2] + raw[3];
    if (csum != raw[4]) {
        ESP_LOGW(TAG, "Checksum fail: calc=0x%02X recv=0x%02X", csum, raw[4]);
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t rh_raw   = ((uint16_t)raw[0] << 8) | raw[1];
    uint16_t temp_raw = ((uint16_t)raw[2] << 8) | raw[3];

    out->humidity_pct  = rh_raw / 10.0f;
    float temp_val     = (temp_raw & 0x7FFF) / 10.0f;
    out->temperature_c = (temp_raw & 0x8000) ? -temp_val : temp_val;
    out->valid         = true;

    return ESP_OK;
}

// ---------------------------------------------------------------------------
void dht22_get_last(dht22_reading_t *out)
{
    if (!s_mutex) { *out = (dht22_reading_t){0}; return; }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = g_last;
    xSemaphoreGive(s_mutex);
}

// ---------------------------------------------------------------------------
void dht22_task(void *arg)
{
    esp_task_wdt_add(NULL);
    while (1) {
        dht22_reading_t r = {0};
        esp_err_t err = dht22_read(&r);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (err == ESP_OK) {
            r.last_ok_ms = now_ms;
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            g_last = r;
            xSemaphoreGive(s_mutex);
            ESP_LOGI(TAG, "DHT22 T=%.1fC H=%.1f%% status=OK",
                     r.temperature_c, r.humidity_pct);
        } else {
            dht22_reading_t last;
            dht22_get_last(&last);
            uint32_t age_s = (now_ms - last.last_ok_ms) / 1000;
            ESP_LOGW(TAG, "DHT22 read FAIL, last good T=%.1fC H=%.1f%% age=%lus",
                     last.temperature_c, last.humidity_pct, (unsigned long)age_s);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(DHT22_READ_INTERVAL_MS));
    }
}
