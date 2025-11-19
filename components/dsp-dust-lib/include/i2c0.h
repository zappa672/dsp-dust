#pragma once

#include "pthread.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t i2c_0_init();

void i2c_0_clean(void);

#ifdef __cplusplus
}
#endif
