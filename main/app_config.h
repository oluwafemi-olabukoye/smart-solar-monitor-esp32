#pragma once

// ---- I2C / LCD ----
#define I2C_SDA_GPIO              21
#define I2C_SCL_GPIO              22
#define I2C_FREQ_HZ               100000
#define LCD_I2C_ADDR              0x27
#define LCD_COLS                  20
#define LCD_ROWS                  4

// ---- ADC1 channels ----
#define BAT_ADC_GPIO              34   // ADC1_CH6
#define SOLAR_ADC_GPIO            35   // ADC1_CH7
#define GRID_ADC_GPIO             33   // ADC1_CH5
#define LDR_ADC_GPIO              32   // ADC1_CH4

// ---- Voltage dividers ----
#define BAT_DIV_R1                47000.0f
#define BAT_DIV_R2                10000.0f
#define SOLAR_DIV_R1              91000.0f
#define SOLAR_DIV_R2              10000.0f
#define GRID_DIV_R1               56000.0f
#define GRID_DIV_R2               10000.0f

// ---- DHT22 ----
#define DHT22_GPIO                4
#define DHT22_READ_INTERVAL_MS    3000

// ---- Relay / Buzzer ----
#define RELAY_GPIO                18
#define RELAY_ACTIVE_LEVEL        0    // 0 = active LOW, 1 = active HIGH
#define BUZZER_GPIO               5

// ---- PZEM-004T ----
#define PZEM_UART_NUM             UART_NUM_2
#define PZEM_RX_GPIO              16
#define PZEM_TX_GPIO              17
#define PZEM_BAUD                 9600
#define PZEM_SLAVE_ADDR           0xF8

// ---- Thresholds ----
#define SOLAR_PRESENT_VOLTAGE     16.0f
#define GRID_PRESENT_VOLTAGE      10.0f
#define BATTERY_LOW_VOLTAGE       11.9f
#define BATTERY_CRITICAL_VOLTAGE  11.7f
#define BATTERY_FULL_VOLTAGE      13.8f
#define LDR_DAY_THRESHOLD_RAW     1500
#define TEMP_HIGH_C               45.0f

// ---- Timing ----
#define CONTROL_LOOP_MS           1000
#define SOURCE_SWITCH_HYSTERESIS_MS  5000
#define LCD_PAGE_DURATION_MS      4000
