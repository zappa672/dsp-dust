#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "sdkconfig.h"
#include "esp_log.h"

#include "common.h"
#include "images.h"

#include "i2c0.h"
#include "led_mat.h"
#include "mic_adc.h"
#include "pot_adc.h"

void app_main(void) {
    // configure_buttons();
    configure_led();
    configure_pot_adc();
    configure_mic_adc();
    i2c_0_init();

    TaskHandle_t fftHandle;

    // xTaskCreate(buttons_read_task, "buttons_read_task", BUTTONS_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(pot_read_task, "pot_read_task", POT_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(mic_adc_task, "mic_adc_task", MIC_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(fft_task, "fft_task", FFT_TASK_STACK_SIZE, NULL, 10, &fftHandle);

    vTaskPrioritySet(fftHandle, tskIDLE_PRIORITY + 13);
}
