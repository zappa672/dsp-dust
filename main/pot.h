#pragma once

#define POT_TASK_STACK_SIZE          2048

void configure_pot_adc();

void pot_read_task(void *arg);

int gain(void);

int bright(void);
