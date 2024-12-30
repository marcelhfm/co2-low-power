#pragma once

#include <stddef.h>
#include <stdint.h>

// display resolution
#define EPD_3IN7_WIDTH 280
#define EPD_3IN7_HEIGHT 480

void epd_3in7_init(void);
void epd_3in7_init();
void epd_3in7_clear();
void epd_3in7_display(const uint8_t *image);
