#pragma once
#include <stdbool.h>
#include "esp_err.h"

// Returns true once Wi-Fi is associated and has an IP address.
bool wifi_is_connected(void);

// Initialize Wi-Fi STA, register event handlers, start connection.
// Internally creates iot_task and ota_task.
// Must be called after nvs_flash_init().
esp_err_t wifi_iot_init(void);

// FreeRTOS task bodies — created by wifi_iot_init(), not by the caller.
void iot_task(void *arg);
void ota_task(void *arg);
