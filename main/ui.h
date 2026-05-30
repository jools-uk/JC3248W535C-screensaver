#pragma once
#include <stdint.h>
#include "display.h"

void text_init(void);
void text_draw(uint16_t *fb, int x, int y, const char *str,
               const uint8_t *font, uint16_t fg, uint16_t bg);

/* fb is borrowed by the caller (pass index_buf from main). */
void display_orientation_guide(uint16_t *fb);
