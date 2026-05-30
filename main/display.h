/*
 * display.h — AXS15231B QSPI display hardware layer.
 *
 * Owns: SPI bus, io_handle, strip_buf, dma_done_sem.
 * Provides: display_init, display_fill, flush_color_fb, lcd_set_window,
 *           lcd_ramwr (needed by fractal.c for flush_frame).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"

/* ── LCD pin map ──────────────────────────────────────────────────────────── */
#define LCD_CLK_IO    47
#define LCD_D0_IO     21
#define LCD_D1_IO     48
#define LCD_D2_IO     40
#define LCD_D3_IO     39
#define LCD_CS_IO     45
#define LCD_BL_IO      1

/* ── Physical panel dimensions — native portrait CASET/RASET addressing. ─── */
#define DISP_W         320
#define DISP_H         480
#define LCD_CLK_HZ     (40 * 1000 * 1000)

/* ── Software rotation ────────────────────────────────────────────────────── */
/* Defined in config.h — default is 0 (portrait) if not set there.
 * 0=portrait  90=landscape CW  180=portrait flipped  270=landscape CCW      */
#include "config.h"
#ifndef DISP_ROTATE
#  define DISP_ROTATE  0
#endif

/* Logical dimensions seen by the fractal, touch, and UI. */
#if (DISP_ROTATE == 90) || (DISP_ROTATE == 270)
#  define LCD_W  DISP_H   /* 480 */
#  define LCD_H  DISP_W   /* 320 */
#else
#  define LCD_W  DISP_W   /* 320 */
#  define LCD_H  DISP_H   /* 480 */
#endif

#define LCD_PIX        (LCD_W * LCD_H)

/* ── Strip-based DMA — sized for physical portrait strips. ───────────────── */
#define STRIP_H        80
#define STRIP_PIXELS   (DISP_W * STRIP_H)

/* ── Shared hardware handles ──────────────────────────────────────────────── */
extern SemaphoreHandle_t         dma_done_sem;
extern uint16_t                 *strip_buf;
extern esp_lcd_panel_io_handle_t io_handle;

/* ── Colour helper ────────────────────────────────────────────────────────── */
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t v = ((uint16_t)(r & 0xF8) << 8) |
                 ((uint16_t)(g & 0xFC) << 3) |
                 ((uint16_t)(b)        >> 3);
    return (v >> 8) | (v << 8);   /* byte-swap: display expects MSB first */
}

/* ── Public API ───────────────────────────────────────────────────────────── */
void display_init(void);
void display_fill(uint16_t colour);
void flush_color_fb(const uint16_t *fb);

/* Low-level pixel output — also used by fractal.c for flush_frame. */
void lcd_set_window(int x0, int y0, int x1, int y1);
void lcd_ramwr(const uint16_t *data, int n_pixels);
