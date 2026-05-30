#pragma once
#include <stdint.h>
#include "display.h"

/* ── Home view — shows the whole Mandelbrot set ───────────────────────────── */
#define HOME_CX   -0.5f
#define HOME_CY    0.0f
#define HOME_HW   10.0f

/* ── Fractal rendering ────────────────────────────────────────────────────── */
#define MAX_ITER        200
#define IN_SET_SENTINEL 0xFFFF
#define ITER_SCALE      200.0f

/* ── Palette ──────────────────────────────────────────────────────────────── */
#define PALETTE_BITS    12
#define PALETTE_SIZE    (1 << PALETTE_BITS)
#define PALETTE_MASK    (PALETTE_SIZE - 1)
#define PALETTE_SPEED   3

/* ── Animation / zoom ─────────────────────────────────────────────────────── */
#define FRAME_MS        33
#define ZOOM_FRAMES     90
#define ZOOM_STEP       0.90f
#define ZOOM_MIN        1e-6f

/* ── Shared buffers ───────────────────────────────────────────────────────── */
extern uint16_t  palette[PALETTE_SIZE];
extern uint16_t *index_buf;

/* ── Zoom state ───────────────────────────────────────────────────────────── */
extern float zoom_target_x;
extern float zoom_target_y;

/* ── API ──────────────────────────────────────────────────────────────────── */
void fractal_init(void);
void build_palette(void);
void render_fractal(float cx, float cy, float hw);
void flush_frame(int pal_offset);
void screen_to_fractal(int sx, int sy, float cx, float cy, float hw,
                       float *fx, float *fy);
