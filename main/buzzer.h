#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    ALERT_NONE = 0,
    ALERT_BATTERY_LOW,         // single 100 ms beep every 5 s
    ALERT_BATTERY_CRITICAL,    // three rapid 80 ms beeps every 2 s
    ALERT_HIGH_TEMP,           // long 500 ms beep every 10 s
    ALERT_NO_SOURCE,           // continuous 200 ms on / 200 ms off
    ALERT_SENSOR_FAULT,        // slow 200 ms beep every 3 s — highest priority
} alert_type_t;

esp_err_t    buzzer_init(void);
void         buzzer_beep(uint32_t ms);

void         buzzer_set_alert(alert_type_t a);   // thread-safe
alert_type_t buzzer_get_alert(void);             // thread-safe

void buzzer_task(void *arg);
