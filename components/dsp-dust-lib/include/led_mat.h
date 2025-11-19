#pragma once

#include "led_strip.h"

#include "color.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED_MATRIX_GPIO              17
#define LED_MATRIX_WIDTH             16
#define LED_MATRIX_HEIGHT            16
#define LED_MATRIX_RMT_RESOLUTION_HZ 10 * 1000 * 1000 // 10MHz

static color_t cur_pattern[LED_MATRIX_HEIGHT];
static led_strip_handle_t led_strip;

void configure_led(void);

void draw_line(
  led_strip_handle_t *strip,
  uint8_t col, uint8_t col_height,
  color_t pattern[LED_MATRIX_HEIGHT]);

#ifdef __cplusplus
}
#endif
