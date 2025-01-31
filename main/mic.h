#pragma once

#include "led.h"

#include <pthread.h>

#include "esp_adc/adc_continuous.h"

#define MIC_TASK_STACK_SIZE          8096
#define MIC_ADC_READ_LEN             2048
#define MIC_ADC_ATTEN                ADC_ATTEN_DB_12
#define MIC_ADC_UNIT                 ADC_UNIT_1
#define MIC_ADC_CHANNEL              ADC_CHANNEL_7 // GPIO35
#define MIC_ADC_BITWIDTH             SOC_ADC_DIGI_MAX_BITWIDTH
#define MIC_ADC_CONV_MODE            ADC_CONV_SINGLE_UNIT_1
#define MIC_ADC_OUTPUT_TYPE          ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define MIC_ADC_SAMPLING_FREQUENCY   96000 // 48kHz

#define FFT_STACK_SIZE               8096

static pthread_mutex_t mic_data_mutex;

static float spectrum[LED_MATRIX_WIDTH];

void configure_mic_adc(void);

void mic_adc_task(void *arg);

void fft_task(void *arg);
