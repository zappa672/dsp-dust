#include <math.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "soc/soc_caps.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "led_strip.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_dsp.h"


// #define BUTTONS_UART_PORT_NUM       1
// #define BUTTONS_UART_RX_GPIO        4 //GPIO4
// #define BUTTONS_UART_BAUD_RATE      9600
// #define BUTTONS_TASK_STACK_SIZE     2048
// #define BUF_SIZE (1024)

#define POT_TASK_STACK_SIZE          2048
#define POT_ADC_ATTEN                ADC_ATTEN_DB_11
#define POT_ADC_UNIT                 ADC_UNIT_2
#define POT_GAIN_ADC_CHANNEL         ADC_CHANNEL_5 // GPIO 12
#define POT_BRIGHT_ADC_CHANNEL       ADC_CHANNEL_4 // GPIO 13
#define POT_ADC_CALI_SCHEME          ADC_CALI_SCHEME_VER_LINE_FITTING
#define POT_ADC_BITWIDTH             SOC_ADC_DIGI_MAX_BITWIDTH

#define LED_MATRIX_GPIO              17
#define LED_MATRIX_WIDTH             16
#define LED_MATRIX_HEIGHT            16
#define LED_MATRIX_RMT_RESOLUTION_HZ 10 * 1000 * 1000 // 10MHz

#define MIC_TASK_STACK_SIZE          8096
#define MIC_ADC_READ_LEN             2048
#define MIC_ADC_ATTEN                ADC_ATTEN_DB_11
#define MIC_ADC_UNIT                 ADC_UNIT_1
#define MIC_ADC_CHANNEL              ADC_CHANNEL_7 // GPIO35
#define MIC_ADC_BITWIDTH             SOC_ADC_DIGI_MAX_BITWIDTH
#define MIC_ADC_CONV_MODE            ADC_CONV_SINGLE_UNIT_1
#define MIC_ADC_OUTPUT_TYPE          ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define MIC_ADC_SAMPLING_FREQUENCY   96000 // 48kHz

#define FFT_STACK_SIZE               8096

static const char *TAG = "dsp-dust";

typedef struct color {
    uint8_t red;
    uint8_t green;
    uint8_t blue; 
} color_t;

static color_t cur_pattern[LED_MATRIX_HEIGHT];
static led_strip_handle_t led_strip;

static adc_continuous_handle_t mic_adc_cont_handle;
static TaskHandle_t mic_adc_conv_task_handle;

static pthread_mutex_t mic_data_mutex;
static int mic_data[MIC_ADC_READ_LEN];
static int fft_data[MIC_ADC_READ_LEN];

__attribute__((aligned(16)))
float y_cf[2*MIC_ADC_READ_LEN];


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
//                     button_id = (int)(data[i] - '0');
//                     ESP_LOGI(TAG, "Recieved button %d" click, button_id);
//                 }
//             }
//         }
//     }

//     free(data);
//     uart_driver_delete(BUTTONS_UART_PORT_NUM);
// }

static adc_cali_handle_t pot_adc_cali_handle;
static adc_oneshot_unit_handle_t pot_adc_handle;

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

int gain_raw, gain_voltage;
int bright_raw, bright_voltage;

static void pot_read_task(void *arg) {
    int raw, voltage;

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

void configure_led(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_MATRIX_GPIO,
        .max_leds = LED_MATRIX_WIDTH*LED_MATRIX_HEIGHT,
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = LED_MATRIX_RMT_RESOLUTION_HZ, // 10MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip)
    );

    for (int row = 0; row < LED_MATRIX_HEIGHT; row++) {
        if (row < LED_MATRIX_HEIGHT / 2) {
            cur_pattern[row].red = 4;
            cur_pattern[row].green = 16;
            cur_pattern[row].blue = 4;
        } else if (row  < 3 * LED_MATRIX_HEIGHT / 4) {
            cur_pattern[row].red = 16;
            cur_pattern[row].green = 16;
            cur_pattern[row].blue = 4;
        } else {
            cur_pattern[row].red = 16;
            cur_pattern[row].green = 4;
            cur_pattern[row].blue = 4;
        }
    }

    led_strip_clear(led_strip);
}

void draw_line(
  led_strip_handle_t *strip,
  uint8_t col, uint8_t col_height,
  color_t pattern[LED_MATRIX_HEIGHT]) {
    if (col >= LED_MATRIX_WIDTH) {
        return;
    }

    float amp = bright_raw / 4095.0;
    
    for (int row = 0; row < LED_MATRIX_HEIGHT; row++) {
        // int pixel = ((LED_MATRIX_HEIGHT - row - 1) * LED_MATRIX_WIDTH) + 
        //              (row % 2 ? (LED_MATRIX_WIDTH - col - 1) : col);
        int pixel = (col * LED_MATRIX_HEIGHT) + (col % 2 ? row : LED_MATRIX_HEIGHT - row);
        if (row < col_height) {
            led_strip_set_pixel(*strip, pixel, (int)(amp*pattern[row].red), (int)(amp*pattern[row].green), (int)(amp*pattern[row].blue));
        } else {
            led_strip_set_pixel(*strip, pixel, 0, 0, 0);
        }
    }
}

void configure_mic_adc(adc_continuous_handle_t *handle) {
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
            .bit_width = MIC_ADC_BITWIDTH,
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

// void print_array(const char* prefix, const float *values, size_t len) {
//     char buffer[8*len + len];
//     char value_buf[9];
//     for (int i = 0; i < len; i++) {
//         sprintf(value_buf, "%8f", values[i]);
//         strcpy(&buffer[9*i], value_buf);
//         buffer[9*(i+1) - 1] = ',';
//     }
//     buffer[8*len + len - 1] = 0;
//     ESP_LOGI(TAG, "%s: [%s]", prefix, buffer);    
// }

static float spectrum[LED_MATRIX_WIDTH];

void fft_task(void *arg) {
    bzero(spectrum, LED_MATRIX_WIDTH*sizeof(float));

    esp_err_t r = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", r);
        return;
    }

    while (1) {
        if (pthread_mutex_lock(&mic_data_mutex) == 0) {            
            memcpy(fft_data, mic_data, MIC_ADC_READ_LEN*sizeof(int));
            pthread_mutex_unlock(&mic_data_mutex);

            for (int i = 0; i < MIC_ADC_READ_LEN; i++) {
                y_cf[i*2 + 0] = fft_data[i];
                y_cf[i*2 + 1] = 0;
            }

            // print_array("adc_data", y_cf, 2*MIC_ADC_READ_LEN);

            dsps_fft2r_fc32(y_cf, MIC_ADC_READ_LEN);
            dsps_bit_rev_fc32(y_cf, MIC_ADC_READ_LEN);

            // print_array("fft_data", y_cf, 2*MIC_ADC_READ_LEN);

            for (int i = 0; i < LED_MATRIX_WIDTH; i++) {
                spectrum[i] = 0;
            }

            int skip = 6;
            for (int i = skip; i < LED_MATRIX_WIDTH+skip; i++) {
                float amp = sqrt(y_cf[i * 2 + 0] * y_cf[i * 2 + 0] + y_cf[i * 2 + 1] * y_cf[i * 2 + 1]);
                spectrum[i-skip] = 0.95 * spectrum[i-skip] + 0.05* amp;
            }

            float mul = 10/4095.0 + (gain_raw/4095.0)*(20/4095.0);
                
            for (int col = 0; col < LED_MATRIX_WIDTH; col++) {
                int height = (int)(mul*spectrum[col]);
                draw_line(&led_strip, col, height, cur_pattern);
            }
            led_strip_refresh(led_strip);
        } 
        usleep(10*1000);
    }

    led_strip_clear(led_strip);
    led_strip_del(led_strip);
}

void app_main(void) {
    if (pthread_mutex_init(&mic_data_mutex, NULL) != 0) {
        ESP_LOGI(TAG, "Failed to initialize mutex");
    }

    // configure_buttons();
    configure_led();
    configure_pot_adc();
    configure_mic_adc(&mic_adc_cont_handle);
    
    // xTaskCreate(buttons_read_task, "buttons_read_task", BUTTONS_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(pot_read_task, "pot_read_task", POT_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(mic_adc_task, "mic_adc_task", MIC_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(fft_task, "fft_task", FFT_STACK_SIZE, NULL, 10, NULL);
}
