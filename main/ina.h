#pragma once

#define INA_TASK_STACK_SIZE 2048

#ifdef __cplusplus
extern "C" {
#endif

void configure_ina_i2c();

void ina_read_task(void *arg);

#ifdef __cplusplus
}
#endif
