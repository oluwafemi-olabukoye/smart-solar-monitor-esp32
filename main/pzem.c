#include "pzem.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "PZEM";

static SemaphoreHandle_t s_mutex          = NULL;
static pzem_reading_t    g_pzem_last_valid = {0};

// ---------------------------------------------------------------------------
// CRC16 Modbus: poly=0xA001 (reflected 0x8005), init=0xFFFF.
// Result is transmitted low byte first.
// ---------------------------------------------------------------------------
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

// Read a big-endian 16-bit register from the response buffer at byte index i.
#define REG16(buf, i) ((uint16_t)((buf)[(i)] << 8) | (buf)[(i) + 1])

// ---------------------------------------------------------------------------
esp_err_t pzem_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = PZEM_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(PZEM_UART_NUM, 256, 0, 0, NULL, 0),
                        TAG, "uart_driver_install");
    ESP_RETURN_ON_ERROR(uart_param_config(PZEM_UART_NUM, &cfg),
                        TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(PZEM_UART_NUM,
                                     PZEM_TX_GPIO, PZEM_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin");

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "PZEM-004T ready: UART%d TX=GPIO%d RX=GPIO%d %d baud addr=0x%02X",
             (int)PZEM_UART_NUM, PZEM_TX_GPIO, PZEM_RX_GPIO,
             PZEM_BAUD, PZEM_SLAVE_ADDR);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Send an 8-byte Modbus RTU request and receive the 25-byte response.
// Response layout: [addr][0x04][0x14][reg0_hi][reg0_lo]...[reg9_hi][reg9_lo][crc_lo][crc_hi]
// ---------------------------------------------------------------------------
esp_err_t pzem_read(pzem_reading_t *out)
{
    // Build request: FC=0x04, start reg=0x0000, count=0x000A (10 registers)
    uint8_t req[8] = {
        PZEM_SLAVE_ADDR, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00
    };
    uint16_t req_crc = crc16(req, 6);
    req[6] = req_crc & 0xFF;
    req[7] = (req_crc >> 8) & 0xFF;

    uart_flush_input(PZEM_UART_NUM);
    uart_write_bytes(PZEM_UART_NUM, req, sizeof(req));

    // 25 = 3-byte header + 20 data bytes + 2-byte CRC
    uint8_t resp[25];
    int n = uart_read_bytes(PZEM_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(200));
    if (n < (int)sizeof(resp)) {
        ESP_LOGW(TAG, "Timeout: got %d/25 bytes", n);
        return ESP_ERR_TIMEOUT;
    }

    // Validate function code and byte count.
    // Address check is skipped: a device responding to broadcast 0xF8 may
    // reply with its own address (0x01 etc.) rather than 0xF8.
    if (resp[1] != 0x04) {
        ESP_LOGW(TAG, "Wrong function code: 0x%02X (expected 0x04)", resp[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (resp[2] != 0x14) {   // 20 = 10 registers × 2 bytes
        ESP_LOGW(TAG, "Wrong byte count: %d (expected 20)", resp[2]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // CRC covers bytes 0..22 (everything before the two CRC bytes)
    uint16_t calc = crc16(resp, 23);
    uint16_t recv = (uint16_t)resp[23] | ((uint16_t)resp[24] << 8);
    if (calc != recv) {
        ESP_LOGW(TAG, "CRC fail: calc=0x%04X recv=0x%04X", calc, recv);
        return ESP_ERR_INVALID_CRC;
    }

    // Parse registers — data starts at byte 3, two bytes per register (big-endian)
    //   Reg 0x00 @ byte 3:  voltage raw → /10.0 V
    //   Reg 0x01 @ byte 5:  current LSB word
    //   Reg 0x02 @ byte 7:  current MSB word → combined/1000.0 A
    //   Reg 0x03 @ byte 9:  power LSB word
    //   Reg 0x04 @ byte 11: power MSB word   → combined/10.0 W
    //   Reg 0x05 @ byte 13: energy LSB word
    //   Reg 0x06 @ byte 15: energy MSB word  → combined Wh
    //   Reg 0x07 @ byte 17: frequency raw    → /10.0 Hz
    //   Reg 0x08 @ byte 19: power factor raw → /100.0
    //   Reg 0x09 @ byte 21: alarm status (ignored)
    uint16_t curr_lo = REG16(resp,  5);
    uint16_t curr_hi = REG16(resp,  7);
    uint16_t pwr_lo  = REG16(resp,  9);
    uint16_t pwr_hi  = REG16(resp, 11);
    uint16_t egy_lo  = REG16(resp, 13);
    uint16_t egy_hi  = REG16(resp, 15);

    out->voltage   = REG16(resp, 3) / 10.0f;
    out->current   = ((uint32_t)curr_hi << 16 | curr_lo) / 1000.0f;
    out->power     = ((uint32_t)pwr_hi  << 16 | pwr_lo)  / 10.0f;
    out->energy    = (float)((uint32_t)egy_hi << 16 | egy_lo);
    out->frequency = REG16(resp, 17) / 10.0f;    // raw is Hz×10 → /10 = Hz
    out->pf        = REG16(resp, 19) / 100.0f;
    out->valid     = true;

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Reset energy accumulator (FC=0x42).  Response is the 4-byte request echoed.
// ---------------------------------------------------------------------------
esp_err_t pzem_reset_energy(void)
{
    uint8_t req[4] = { PZEM_SLAVE_ADDR, 0x42, 0x00, 0x00 };
    uint16_t c = crc16(req, 2);
    req[2] = c & 0xFF;
    req[3] = (c >> 8) & 0xFF;

    uart_flush_input(PZEM_UART_NUM);
    uart_write_bytes(PZEM_UART_NUM, req, sizeof(req));

    uint8_t resp[4];
    int n = uart_read_bytes(PZEM_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(200));
    if (n < 4) return ESP_ERR_TIMEOUT;

    uint16_t rc = crc16(resp, 2);
    uint16_t rr = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    if (rc != rr) return ESP_ERR_INVALID_CRC;

    return (resp[1] == 0x42) ? ESP_OK : ESP_FAIL;
}

// ---------------------------------------------------------------------------
void pzem_get_last(pzem_reading_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = g_pzem_last_valid;
    xSemaphoreGive(s_mutex);
}

// ---------------------------------------------------------------------------
void pzem_task(void *arg)
{
    while (1) {
        pzem_reading_t r = {0};
        esp_err_t err = pzem_read(&r);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (err == ESP_OK) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            g_pzem_last_valid          = r;
            g_pzem_last_valid.last_ok_ms = now_ms;
            xSemaphoreGive(s_mutex);

            ESP_LOGI(TAG, "PZEM V=%.1f I=%.3f P=%.1fW E=%.0fWh F=%.1f PF=%.2f",
                     r.voltage, r.current, r.power, r.energy, r.frequency, r.pf);
        } else {
            ESP_LOGW(TAG, "Read fail: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
