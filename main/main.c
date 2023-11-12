#include <math.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "led_strip.h"
#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"


// #define BUTTONS_UART_PORT_NUM       1
// #define BUTTONS_UART_RX_GPIO        4 //GPIO4
// #define BUTTONS_UART_BAUD_RATE      9600
// #define BUTTONS_TASK_STACK_SIZE     2048
// #define BUF_SIZE (1024)

#define LED_STRIP_GPIO              17

#define MIC_ADC_READ_LEN            256
#define MIC_ADC_ATTEN               ADC_ATTEN_DB_11
#define MIC_ADC_UNIT                ADC_UNIT_1
#define MIC_ADC_CHANNEL             ADC_CHANNEL_7 // GPIO35
#define MIC_ADC_BIT_WIDTH           SOC_ADC_DIGI_MAX_BITWIDTH
#define MIC_ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define MIC_ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define MIC_TASK_STACK_SIZE         8096
#define MIC_ADC_SAMPLING_FREQUENCY  40*1000 // 40kHz

#define FFT_STACK_SIZE              8096


static const char *TAG = "dsp-dust";

// static int current_mode = 0;

static led_strip_handle_t led_strip;

static adc_continuous_handle_t mic_adc_cont_handle;
static TaskHandle_t mic_adc_conv_task_handle;

static pthread_mutex_t mic_data_mutex;
static int mic_data[MIC_ADC_READ_LEN];
static int fft_data[MIC_ADC_READ_LEN];

__attribute__((aligned(16)))
float window[MIC_ADC_READ_LEN];
__attribute__((aligned(16)))
float y_cf[2*MIC_ADC_READ_LEN];

// void configure_buttons(void) {

// }

// static void buttons_read_task(void *arg)
// {
//     uart_config_t uart_config = {
//         .baud_rate = BUTTONS_UART_BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT,
//     };

//     ESP_ERROR_CHECK(uart_driver_install(BUTTONS_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 1));
//     ESP_ERROR_CHECK(uart_param_config(BUTTONS_UART_PORT_NUM, &uart_config));
//     ESP_ERROR_CHECK(uart_set_pin(BUTTONS_UART_PORT_NUM, -1, BUTTONS_UART_RX_GPIO, -1, -1));

//     char *data = (char *) malloc(BUF_SIZE);

//     while (1) {
//         // Read data from the UART
//         int len = uart_read_bytes(BUTTONS_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
//         if (len) {
//             for (int i = 0; i < len; i++) {
//                 if (data[i] >= '0' && data[i] < '6') {
//                     current_mode = (int)(data[i] - '0');
//                 }
//             }
//             ESP_LOGI(TAG, "Mode changed to: %d", current_mode);
//         }
//     }

//     free(data);
//     uart_driver_delete(BUTTONS_UART_PORT_NUM);
// }

void configure_led(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = 16*8,
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    led_strip_clear(led_strip);
}

static bool printed = false;

void draw_line(led_strip_handle_t *strip, uint16_t width, uint16_t height, uint16_t column_id, uint16_t column_height) {
    if (column_id >= width || column_height > height) {
        return;
    }

    for (int row_id = 0; row_id < height; row_id++) {
        int pixel_id = ((height - row_id - 1) * width) + (row_id % 2 ? (width - column_id - 1) : column_id);
        if (row_id < column_height) {
            if (row_id < 4) {
                led_strip_set_pixel(*strip, pixel_id, 1, 4, 1);
            } else if (row_id < 6) {
                led_strip_set_pixel(*strip, pixel_id, 3, 3, 1);
            } else {
                led_strip_set_pixel(*strip, pixel_id, 4, 1, 1);
            }
        } else {
            led_strip_set_pixel(*strip, pixel_id, 0, 0, 0);
        }
    }
    printed = true;
}

void configure_adc(adc_continuous_handle_t *handle) {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 2*MIC_ADC_READ_LEN*SOC_ADC_DIGI_RESULT_BYTES,
        .conv_frame_size = MIC_ADC_READ_LEN*SOC_ADC_DIGI_RESULT_BYTES,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, handle));

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = { 
        {
            .atten = MIC_ADC_ATTEN,
            .channel = MIC_ADC_CHANNEL,
            .unit = MIC_ADC_UNIT,
            .bit_width = MIC_ADC_BIT_WIDTH,
        }
    };

    adc_continuous_config_t adc_cfg = {
        .pattern_num = 1,
        .sample_freq_hz = MIC_ADC_SAMPLING_FREQUENCY,
        .adc_pattern = adc_pattern,
        .conv_mode = MIC_ADC_CONV_MODE,
        .format = MIC_ADC_OUTPUT_TYPE,
    };
    ESP_ERROR_CHECK(adc_continuous_config(*handle, &adc_cfg));
}

static bool IRAM_ATTR on_mic_adc_conv_done(
    adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(mic_adc_conv_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

static bool IRAM_ATTR on_mic_adc_pool_ovf(
    adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    return false;
}

void mic_adc_task(void *arg) {
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[MIC_ADC_READ_LEN*SOC_ADC_DIGI_RESULT_BYTES] = {0};
    memset(result, 0xcc, MIC_ADC_READ_LEN*SOC_ADC_DIGI_RESULT_BYTES);
    
    mic_adc_conv_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = on_mic_adc_conv_done,
        .on_pool_ovf = on_mic_adc_pool_ovf,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(mic_adc_cont_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(mic_adc_cont_handle));

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (1) {
            ret = adc_continuous_read(mic_adc_cont_handle, result, MIC_ADC_READ_LEN*SOC_ADC_DIGI_RESULT_BYTES, &ret_num, 0);
            if (ret == ESP_OK) {
                if (pthread_mutex_lock(&mic_data_mutex) == 0) {
                    for (int i = 0; i < ret_num/SOC_ADC_DIGI_RESULT_BYTES; i ++) {
                        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i*SOC_ADC_DIGI_RESULT_BYTES];
                        mic_data[i] = p->type1.data;
                    }
                    pthread_mutex_unlock(&mic_data_mutex);
                }
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(mic_adc_cont_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(mic_adc_cont_handle));
}

void fft_task(void *arg) {
    float spectrum[16];

    esp_err_t r = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", r);
        return;
    }

    // Generate hann window
    dsps_wind_hann_f32(window, MIC_ADC_READ_LEN);

    while (1) {
        if (pthread_mutex_lock(&mic_data_mutex) == 0) {
            memcpy(fft_data, mic_data, MIC_ADC_READ_LEN*sizeof(int));
            pthread_mutex_unlock(&mic_data_mutex);
        }

        for (int i = 0; i < MIC_ADC_READ_LEN; i++) {
            y_cf[i*2 + 0] = (fft_data[i] / 1000.0) * window[i];
            y_cf[i*2 + 1] = 0;
        }

        dsps_fft2r_fc32(y_cf, MIC_ADC_READ_LEN);

        for (int i = 0; i < 16; i++) {
            spectrum[i] = 0;
        }

        for (int i = 1 ; i < MIC_ADC_READ_LEN/2 ; i++) {
            int index = ((float)i / (MIC_ADC_READ_LEN/2 + 1)) * 16;
            float amp = y_cf[i * 2 + 0] * y_cf[i * 2 + 0] + y_cf[i * 2 + 1] * y_cf[i * 2 + 1];
            if (amp > spectrum[index]) {
                spectrum[index] = amp;
            }
        }

        // ESP_LOGI(TAG, "spectrum: { %f, %f, %f, %f, %f, %f, %f, %f }",
        //          spectrum[0], spectrum[1], spectrum[2], spectrum[3],
        //          spectrum[4], spectrum[5], spectrum[6], spectrum[7]);
    

        for (int i = 0; i < 16; i++) {
            int height = spectrum[i] / 128;
            draw_line(&led_strip, 16, 8, i, height);
        }
        led_strip_refresh(led_strip);
    }

    led_strip_clear(led_strip);
    led_strip_del(led_strip);
}

void app_main(void) {
    if (pthread_mutex_init(&mic_data_mutex, NULL) != 0) {
        ESP_LOGI(TAG, "Failed to initialize the spiffs mutex");
    }

    configure_led();
    // configure_buttons();
    configure_adc(&mic_adc_cont_handle);

    // xTaskCreate(buttons_read_task, "buttons_read_task", BUTTONS_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(mic_adc_task, "mic_adc_task", MIC_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(fft_task, "fft_task", FFT_STACK_SIZE, NULL, 10, NULL);
}
