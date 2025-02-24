#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INA_TASK_STACK_SIZE 1024

esp_err_t i2c_0_init(void);

void i2c_0_clean(void);

void i2c_0_task(void *arg);

#ifdef __cplusplus
}
#endif
