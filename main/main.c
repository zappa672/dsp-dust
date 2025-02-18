#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "common.h"
#include "ina.h"
#include "mic.h"
#include "led.h"
#include "pot.h"

#include "sdkconfig.h"
#include "esp_log.h"


void app_main(void) {
    // configure_buttons();
    configure_led();
    configure_pot_adc();
    configure_mic_adc();
    configure_ina_i2c();

    // xTaskCreate(buttons_read_task, "buttons_read_task", BUTTONS_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(pot_read_task, "pot_read_task", POT_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(mic_adc_task, "mic_adc_task", MIC_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(fft_task, "fft_task", FFT_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(ina_read_task, "ina_read_task", INA_TASK_STACK_SIZE, NULL, 10, NULL);
}
