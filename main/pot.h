#pragma once

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#define POT_TASK_STACK_SIZE          2048
#define POT_ADC_ATTEN                ADC_ATTEN_DB_11
#define POT_ADC_UNIT                 ADC_UNIT_2
#define POT_GAIN_ADC_CHANNEL         ADC_CHANNEL_7 // GPIO 26
#define POT_BRIGHT_ADC_CHANNEL       ADC_CHANNEL_9 // GPIO 27
#define POT_ADC_CALI_SCHEME          ADC_CALI_SCHEME_VER_LINE_FITTING
#define POT_ADC_BITWIDTH             SOC_ADC_DIGI_MAX_BITWIDTH

static adc_cali_handle_t pot_adc_cali_handle;
static adc_oneshot_unit_handle_t pot_adc_handle;

static int gain_raw, gain_voltage;
static int bright_raw, bright_voltage;

void configure_pot_adc();

void pot_read_task(void *arg);
