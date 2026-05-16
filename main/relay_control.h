#pragma once
#include "esp_err.h"

esp_err_t relay_control_init(void);
void      relay_on(void);
void      relay_off(void);
