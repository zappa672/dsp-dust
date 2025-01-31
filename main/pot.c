#include "common.h"
#include "pot.h"

#include <unistd.h>

#include "esp_log.h"

void configure_pot_adc() {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = POT_ADC_UNIT,
        .atten = POT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &pot_adc_cali_handle));

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = POT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &pot_adc_handle));

    adc_oneshot_chan_cfg_t oneshot_chan_config = {
        .bitwidth = POT_ADC_BITWIDTH,
        .atten = POT_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(pot_adc_handle, POT_GAIN_ADC_CHANNEL, &oneshot_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(pot_adc_handle, POT_BRIGHT_ADC_CHANNEL, &oneshot_chan_config));
}

void pot_read_task(void *arg) {
    int raw, voltage;

    ESP_LOGI(TAG, "pot_read_task started");

    for (;;) {
        ESP_ERROR_CHECK(adc_oneshot_read(pot_adc_handle, POT_GAIN_ADC_CHANNEL, &raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(pot_adc_cali_handle, raw, &voltage));

        gain_raw = 0.95*gain_raw + 0.05*raw;
        
        gain_voltage = 0.95*gain_voltage + 0.05*voltage;
        
        ESP_ERROR_CHECK(adc_oneshot_read(pot_adc_handle, POT_BRIGHT_ADC_CHANNEL, &raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(pot_adc_cali_handle, raw, &voltage));

        bright_raw = 0.5*bright_raw + 0.5*raw;
        bright_voltage = 0.5*bright_voltage + 0.5*voltage;

        // ESP_LOGI(TAG, "raw gain: %d, bright: %d, calibrated: %d, %d", gain_raw, bright_raw, gain_voltage, bright_voltage);

        usleep(200 * 1000);
    }

    ESP_LOGI(TAG, "delete %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(pot_adc_cali_handle));

    ESP_ERROR_CHECK(adc_oneshot_del_unit(pot_adc_handle));
}
