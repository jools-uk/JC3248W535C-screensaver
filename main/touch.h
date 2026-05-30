#pragma once
#include <stdbool.h>

/* ── Touch IC pin map ─────────────────────────────────────────────────────── */
#define TOUCH_SDA_IO   4
#define TOUCH_SCL_IO   8
#define TOUCH_RST_IO  12

/* ── Touch IC settings ────────────────────────────────────────────────────── */
#define TOUCH_I2C_ADDR    0x3B
/* IC reports: raw_x = portrait column (0–320), raw_y = portrait row (0–480).
 * Coordinate mapping to logical screen coords is derived from DISP_ROTATE. */
#define TOUCH_X_RAW_MAX   320
#define TOUCH_Y_RAW_MAX   480

void touch_init(void);
bool touch_read(int *out_x, int *out_y);
