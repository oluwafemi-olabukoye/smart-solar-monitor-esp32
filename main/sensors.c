// =============================================================================
// SAFETY WARNING — READ BEFORE CONNECTING ANY REAL SOURCE
// =============================================================================
// Before connecting battery, solar panel, or grid supply, measure the voltage
// at each ADC GPIO pin (GPIO34/35/36/32) with a multimeter while the source is
// at its MAXIMUM expected voltage. The pin voltage MUST stay below 3.1 V.
//
// Board divider ratios and max input voltage for 3.1 V at ADC node:
//   Battery (47k/10k,  ratio 5.7) : safe up to ~17.7 V
//   Solar   (91k/10k,  ratio 10.1): safe up to ~31.3 V
//   Grid    (56k/10k,  ratio 6.6) : safe up to ~20.5 V
//
// If the divider resistors are wrong, the ESP32 ADC input will exceed 3.3 V
// and the GPIO will be permanently damaged. Verify with the multimeter first.
// =============================================================================

#include "sensors.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "SENSORS";

static adc_oneshot_unit_handle_t s_adc  = NULL;
static adc_cali_handle_t         s_cali = NULL;
static bool                      s_cali_valid = false;

// ---------------------------------------------------------------------------
// Calibration — try curve fitting (ESP32-S2/S3/C3), fall back to line fitting
// (classic ESP32 reads eFuse calibration data stored at the factory).
// ---------------------------------------------------------------------------
static bool init_calibration(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cf = {
        .unit_id  = ADC_UNIT_1,
        .chan     = ADC_CHANNEL_0,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cf, &s_cali) == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration: curve fitting");
        return true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t lf = {
        .unit_id      = ADC_UNIT_1,
        .atten        = ADC_ATTEN_DB_12,
        .bitwidth     = ADC_BITWIDTH_DEFAULT,
        .default_vref = 1100,  // mV; used only if chip has no eFuse calibration data
    };
    if (adc_cali_create_scheme_line_fitting(&lf, &s_cali) == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration: line fitting");
        return true;
    }
#endif

    ESP_LOGW(TAG, "ADC calibration unavailable — mV values will be approximate");
    return false;
}

// ---------------------------------------------------------------------------
esp_err_t sensors_init(void)
{
    // Create ADC1 oneshot unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc),
                        TAG, "adc_oneshot_new_unit");

    // All four channels at 12 dB attenuation (0 – ~3.1 V input range)
    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_6, &ch_cfg),
                        TAG, "ch6 bat");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_7, &ch_cfg),
                        TAG, "ch7 solar");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_5, &ch_cfg),
                        TAG, "ch5 grid");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_4, &ch_cfg),
                        TAG, "ch4 ldr");

    s_cali_valid = init_calibration();

    ESP_LOGI(TAG, "ADC1 ready: 4 channels, atten=DB_12, cali=%s",
             s_cali_valid ? "OK" : "none");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Read one channel: 32 samples, discard single min and max, average 30.
// ---------------------------------------------------------------------------
static esp_err_t read_channel(adc_channel_t ch, int *raw_out, int *mv_out)
{
    int samples[32];
    for (int i = 0; i < 32; i++) {
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, ch, &samples[i]),
                            TAG, "adc_oneshot_read");
    }

    int mn = samples[0], mx = samples[0], sum = 0;
    for (int i = 0; i < 32; i++) {
        sum += samples[i];
        if (samples[i] < mn) mn = samples[i];
        if (samples[i] > mx) mx = samples[i];
    }
    *raw_out = (sum - mn - mx) / 30;

    if (s_cali_valid) {
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali, *raw_out, mv_out),
                            TAG, "adc_cali_raw_to_voltage");
    } else {
        // DB_12 full-scale ≈ 3100 mV at raw 4095
        *mv_out = (int)((int64_t)(*raw_out) * 3100 / 4095);
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
esp_err_t sensors_read_all(sensor_data_t *out)
{
    int raw, mv;

    // Battery — GPIO34, ADC1_CH6, divider 47k/10k (ratio 5.7)
    ESP_RETURN_ON_ERROR(read_channel(ADC_CHANNEL_6, &raw, &mv), TAG, "bat");
    out->bat_raw     = raw;
    out->bat_mv      = (float)mv;
    out->bat_voltage = (mv / 1000.0f) * (BAT_DIV_R1 + BAT_DIV_R2) / BAT_DIV_R2;

    // Solar — GPIO35, ADC1_CH7, divider 91k/10k (ratio 10.1)
    ESP_RETURN_ON_ERROR(read_channel(ADC_CHANNEL_7, &raw, &mv), TAG, "solar");
    out->solar_raw     = raw;
    out->solar_mv      = (float)mv;
    out->solar_voltage = (mv / 1000.0f) * (SOLAR_DIV_R1 + SOLAR_DIV_R2) / SOLAR_DIV_R2;

    // Grid — GPIO33, ADC1_CH5, divider 56k/10k (ratio 6.6)
    ESP_RETURN_ON_ERROR(read_channel(ADC_CHANNEL_5, &raw, &mv), TAG, "grid");
    out->grid_raw     = raw;
    out->grid_mv      = (float)mv;
    out->grid_voltage = (mv / 1000.0f) * (GRID_DIV_R1 + GRID_DIV_R2) / GRID_DIV_R2;

    // LDR — GPIO32, ADC1_CH4, raw only (no voltage divider on this channel)
    ESP_RETURN_ON_ERROR(read_channel(ADC_CHANNEL_4, &raw, &mv), TAG, "ldr");
    out->ldr_raw = raw;

    // --- Derived flags ---
    // LDR wiring assumption: pull-DOWN resistor to GND.
    //   Light → LDR resistance falls → ADC voltage rises → raw increases.
    //   If your LDR has a pull-UP to 3.3 V, flip this to (ldr_raw < LDR_DAY_THRESHOLD_RAW).
    out->is_daylight   = (out->ldr_raw      > LDR_DAY_THRESHOLD_RAW);
    out->solar_present = (out->solar_voltage >= SOLAR_PRESENT_VOLTAGE);
    out->grid_present  = (out->grid_voltage  >= GRID_PRESENT_VOLTAGE);

    return ESP_OK;
}
