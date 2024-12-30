#pragma once

#include <stddef.h>
#include <stdint.h>

void epd_3in7_init(void);
void epd_3in7_clear(void);
void epd_3in7_display(const uint8_t *image_data, size_t image_size);
