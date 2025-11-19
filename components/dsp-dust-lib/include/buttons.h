#pragma once

#include "led_strip.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum button {
  BUTTON_UP = 0,
  BUTTON_LEFT = 1,
  BUTTON_RIGHT = 2,
  BUTTON_DOWN = 3,
  BUTTON_COUNT = 4,
} button_t;

typedef void (*button_cb_fn)(button_t but);

void but_init(void);

void but_check(button_cb_fn but_cb);

#ifdef __cplusplus
}
#endif
