/*
 * Mandelbrot fractal on JC3248W535C (ESP32-S3, AXS15231B 320×480 QSPI display).
 *
 * Display driver derived from ESPHome's working qspi_dbi/axs15231 component:
 *  – spi_mode=0  (CPOL=0/CPHA=0) — confirmed by ESPHome set_mode(MODE0)
 *  – lcd_cmd_bits=32: header is opcode(8b)+0x00(8b)+cmd(8b)+0x00(8b) in 1-bit SPI
 *  – pixel data in 4-bit QSPI mode (opcode 0x32 = write-color)
 *  – RASET sent before CASET (ESPHome order)
 *  – Init sequence matches ESPHome exactly; no ALL-PIXELS-OFF at the end
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "u8g2.h"

static const char *TAG = "fractal";

/* ── Pin map ──────────────────────────────────────────────────────────────── */
#define LCD_CLK_IO    47
#define LCD_D0_IO     21
#define LCD_D1_IO     48
#define LCD_D2_IO     40
#define LCD_D3_IO     39
#define LCD_CS_IO     45
#define LCD_BL_IO      1

#define TOUCH_SDA_IO   4
#define TOUCH_SCL_IO   8
#define TOUCH_RST_IO  12

/* ── Display ──────────────────────────────────────────────────────────────── */
/* Physical panel dimensions — native portrait CASET/RASET addressing. */
#define DISP_W         320
#define DISP_H         480
#define LCD_CLK_HZ     (40 * 1000 * 1000)

/* Software rotation applied in the flush functions.
 * 0 = portrait (no rotation)
 * 90 = landscape CW   (portrait right becomes top)
 * 180 = portrait upside-down
 * 270 = landscape CCW (portrait left becomes top)  */
#define DISP_ROTATE    270

/* Logical dimensions seen by the fractal, touch, and UI.
 * Swapped from physical for 90° / 270° rotations. */
#if (DISP_ROTATE == 90) || (DISP_ROTATE == 270)
#  define LCD_W        DISP_H   /* 480 */
#  define LCD_H        DISP_W   /* 320 */
#else
#  define LCD_W        DISP_W   /* 320 */
#  define LCD_H        DISP_H   /* 480 */
#endif

#define LCD_PIX        (LCD_W * LCD_H)

/* Strip-based DMA — always sized for physical portrait strips. */
#define STRIP_H        80
#define STRIP_PIXELS   (DISP_W * STRIP_H)

/* ── Touch ────────────────────────────────────────────────────────────────── */
#define TOUCH_I2C_ADDR    0x3B
/* IC always reports: raw_x = portrait column (0–320), raw_y = portrait row (0–480).
 * Coordinate mapping is derived from DISP_ROTATE in touch_read(). */
#define TOUCH_X_RAW_MAX   320
#define TOUCH_Y_RAW_MAX   480

/* ── Fractal home view ────────────────────────────────────────────────────── */
/* The "home" view shows the whole Mandelbrot set.  Tap re-anchors the centre
 * to the tapped point (mapped through this fixed frame) and resets the zoom. */
#define HOME_CX   -0.5f
#define HOME_CY    0.0f
#define HOME_HW   10.0f   /* x spans [-2, 1], y spans [-1, 1] at 480×320 */

/* ── Fractal / animation ──────────────────────────────────────────────────── */
#define MAX_ITER        200
#define PALETTE_BITS    12
#define PALETTE_SIZE    (1 << PALETTE_BITS)   /* 4096 */
#define PALETTE_MASK    (PALETTE_SIZE - 1)
#define ITER_SCALE      200.0f
#define PALETTE_SPEED   3     /* palette index steps per displayed frame   */
#define FRAME_MS        33    /* ~30 fps                                   */
#define ZOOM_FRAMES     90    /* frames of palette animation per zoom step */
#define ZOOM_STEP       0.90f /* multiply view_hw by this each zoom step   */
#define ZOOM_MIN        1e-6f /* float precision floor; resets view after  */
#define IN_SET_SENTINEL 0xFFFF

/* ── Globals ──────────────────────────────────────────────────────────────── */
static uint16_t palette[PALETTE_SIZE];
static uint16_t *index_buf;    /* PSRAM : full-frame iteration indices      */
static uint16_t *strip_buf;    /* SRAM  : one strip, fed to SPI DMA         */

static esp_lcd_panel_io_handle_t io_handle;
static i2c_master_bus_handle_t   touch_bus;
static i2c_master_dev_handle_t   touch_dev;

/* Binary semaphore: given by DMA-done ISR, taken before writing next strip. */
static SemaphoreHandle_t dma_done_sem;

/* Shared view state – written by touch task, read by render task. */
static volatile float view_cx  = HOME_CX;
static volatile float view_cy  = HOME_CY;
static volatile float view_hw  = HOME_HW;
static volatile bool  rerender =  false;

/* Fractal coords of the deepest boundary pixel in the last render.
 * Updated by render_fractal; used by the auto-zoom step. */
static float zoom_target_x = -0.5f;
static float zoom_target_y =  0.0f;

/* ═══════════════════════════════════════════════════════════════════════════
   Palette — three co-prime-frequency sine waves per channel.
   Amplitudes 80 + 47 = 127, so the sum always lands in [1, 255].
   ═══════════════════════════════════════════════════════════════════════════ */

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t v = ((uint16_t)(r & 0xF8) << 8) |
                 ((uint16_t)(g & 0xFC) << 3) |
                 ((uint16_t)(b)        >> 3);
    return (v >> 8) | (v << 8);  /* big-endian: display expects MSB first */
}

static void build_palette(void)
{
    for (int i = 0; i < PALETTE_SIZE; i++) {
        float a = (float)i / PALETTE_SIZE * 2.0f * (float)M_PI;
        uint8_t r = (uint8_t)(sinf(a *  3.0f)           * 80.0f +
                               sinf(a * 13.0f + 0.50f)   * 47.0f + 128.0f);
        uint8_t g = (uint8_t)(sinf(a *  5.0f + 2.094f)  * 80.0f +
                               sinf(a *  7.0f + 1.20f)   * 47.0f + 128.0f);
        uint8_t b = (uint8_t)(sinf(a *  7.0f + 4.189f)  * 80.0f +
                               sinf(a * 11.0f + 0.90f)   * 47.0f + 128.0f);
        palette[i] = rgb565(r, g, b);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   DMA callback — called from ISR when a color transfer completes.
   Gives dma_done_sem so flush_frame knows strip_buf is free again.
   ═══════════════════════════════════════════════════════════════════════════ */

static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                           esp_lcd_panel_io_event_data_t *edata,
                                           void *user_ctx)
{
    BaseType_t awoken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)user_ctx, &awoken);
    return awoken == pdTRUE;
}

/* ═══════════════════════════════════════════════════════════════════════════
   Fractal
   ═══════════════════════════════════════════════════════════════════════════ */

static void render_fractal(float cx, float cy, float hw)
{
    float aspect = (float)LCD_H / (float)LCD_W;
    float hh = hw * aspect;
    float dx = (2.0f * hw) / (float)LCD_W;
    float dy = (2.0f * hh) / (float)LCD_H;

    /* Track the escaped pixel with the highest iteration count — it sits
     * closest to the set boundary and is the best next zoom target. */
    int   best_iter = -1;
    float best_fx = cx, best_fy = cy;

    float py = cy - hh;
    for (int y = 0; y < LCD_H; y++, py += dy) {
        float px = cx - hw;
        for (int x = 0; x < LCD_W; x++, px += dx) {
            float cr = px, ci = py;

            /* Fast early-out for main cardioid and period-2 bulb */
            float cr_q = cr - 0.25f;
            float q = cr_q * cr_q + ci * ci;
            if (q * (q + cr_q) < 0.25f * ci * ci) {
                index_buf[y * LCD_W + x] = IN_SET_SENTINEL;
                continue;
            }
            float cr1 = cr + 1.0f;
            if (cr1 * cr1 + ci * ci < 0.0625f) {
                index_buf[y * LCD_W + x] = IN_SET_SENTINEL;
                continue;
            }

            float zr = 0.0f, zi = 0.0f;
            int   iter = 0;
            while (iter < MAX_ITER) {
                float zr2 = zr * zr, zi2 = zi * zi;
                if (zr2 + zi2 > 4.0f) break;
                zi = 2.0f * zr * zi + ci;
                zr = zr2 - zi2 + cr;
                iter++;
            }

            if (iter == MAX_ITER) {
                index_buf[y * LCD_W + x] = IN_SET_SENTINEL;
            } else {
                float mag2 = zr * zr + zi * zi;
                float nu   = logf(0.5f * logf(mag2)) / logf(2.0f);
                float t    = fmaxf(0.0f, (float)(iter + 1) - nu);
                index_buf[y * LCD_W + x] =
                    (uint16_t)((uint32_t)(t * ITER_SCALE) & PALETTE_MASK);
                if (iter > best_iter) {
                    best_iter = iter;
                    best_fx = px;
                    best_fy = py;
                }
            }
        }
    }

    zoom_target_x = best_fx;
    zoom_target_y = best_fy;
}

/* ═══════════════════════════════════════════════════════════════════════════
   QSPI display helpers (derived from ESPHome qspi_dbi component)

   Header format (lcd_cmd_bits=32, always sent in 1-bit SPI):
     byte 0: opcode  0x02 = write-command/param, 0x32 = write-color (QSPI data)
     byte 1: 0x00
     byte 2: command byte
     byte 3: 0x00
   ═══════════════════════════════════════════════════════════════════════════ */

static void lcd_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    int c = (int)(((uint32_t)0x02 << 24) | ((uint32_t)cmd << 8));
    esp_lcd_panel_io_tx_param(io_handle, c, data, len);
}

/* Portrait window: RASET=y (portrait rows 0–479), CASET=x (portrait cols 0–319). */
static void lcd_set_window(int x0, int y0, int x1, int y1)
{
    uint8_t rs[4] = { y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF };
    uint8_t cs[4] = { x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF };
    lcd_cmd(0x2B, rs, 4);
    lcd_cmd(0x2A, cs, 4);
}

/* RAMWR: pixel data in 4-bit QSPI mode */
static void lcd_ramwr(const uint16_t *data, int n_pixels)
{
    int c = (int)(((uint32_t)0x32 << 24) | ((uint32_t)0x2C << 8));
    esp_lcd_panel_io_tx_color(io_handle, c, data, n_pixels * sizeof(uint16_t));
}

/* ═══════════════════════════════════════════════════════════════════════════
   Display helpers
   ═══════════════════════════════════════════════════════════════════════════ */

/* ═══════════════════════════════════════════════════════════════════════════
   Text rendering — u8g2 as a null-display font engine.

   u8g2 renders into a 1bpp tile buffer (u8g2_buf).  We convert those bits
   to RGB565 and composite them onto any uint16_t framebuffer in PSRAM.
   flush_color_fb() then DMA-strips that buffer to the display.
   ═══════════════════════════════════════════════════════════════════════════ */

static u8g2_t  u8g2_ctx;
/* 1bpp tile buffer: LCD_W bytes × (LCD_H/8) tile rows */
static uint8_t u8g2_buf[LCD_W * (LCD_H / 8)];

static const u8x8_display_info_t u8g2_info = {
    .tile_width   = LCD_W / 8,
    .tile_height  = LCD_H / 8,
    .pixel_width  = LCD_W,
    .pixel_height = LCD_H,
};

static uint8_t u8x8_null_cb(u8x8_t *u8x8, uint8_t msg,
                              uint8_t arg_int, void *arg_ptr)
{
    if (msg == U8X8_MSG_DISPLAY_SETUP_MEMORY)
        u8x8->display_info = &u8g2_info;
    return 1;
}

static void text_init(void)
{
    /* display_info must be set before SetupBuffer — it calls update_dimension
     * immediately, which dereferences display_info to read pixel dimensions. */
    u8x8_SetupDefaults(&u8g2_ctx.u8x8);
    u8g2_ctx.u8x8.display_cb        = u8x8_null_cb;
    u8g2_ctx.u8x8.gpio_and_delay_cb = u8x8_null_cb;
    u8g2_ctx.u8x8.display_info      = &u8g2_info;

    u8g2_SetupBuffer(&u8g2_ctx, u8g2_buf, LCD_H / 8,
                     u8g2_ll_hvline_vertical_top_lsb, U8G2_R0);
    u8g2_InitDisplay(&u8g2_ctx);
    u8g2_SetPowerSave(&u8g2_ctx, 0);
}

/* Draw str at (x, y) — y is the top of the text bounding box.
 * Composites the 1bpp glyph onto fb.  Pass bg=0x0001 for transparent. */
static void text_draw(uint16_t *fb, int x, int y, const char *str,
                      const uint8_t *font, uint16_t fg, uint16_t bg)
{
    u8g2_ClearBuffer(&u8g2_ctx);
    u8g2_SetFont(&u8g2_ctx, font);
    u8g2_DrawStr(&u8g2_ctx, x, y + u8g2_GetAscent(&u8g2_ctx), str);
    bool transparent = (bg == 0x0001);
    for (int row = 0; row < LCD_H; row++) {
        for (int col = 0; col < LCD_W; col++) {
            bool lit = (u8g2_buf[(row / 8) * LCD_W + col] >> (row & 7)) & 1;
            if (lit)
                fb[row * LCD_W + col] = fg;
            else if (!transparent)
                fb[row * LCD_W + col] = bg;
        }
    }
}

/* DMA-strip a logical RGB565 framebuffer to the physical portrait display,
 * applying the software rotation selected by DISP_ROTATE. */
static void flush_color_fb(const uint16_t *fb)
{
    for (int y0 = 0; y0 < DISP_H; y0 += STRIP_H) {
        xSemaphoreTake(dma_done_sem, portMAX_DELAY);
#if DISP_ROTATE == 0
        /* No rotation: portrait logical == portrait physical. */
        for (int r = 0; r < STRIP_H; r++)
            for (int c = 0; c < DISP_W; c++)
                strip_buf[r * DISP_W + c] = fb[(y0 + r) * LCD_W + c];
#elif DISP_ROTATE == 90
        /* 90° CW: portrait(c, y0+r) ← logical(lx=DISP_H-1-y0-r, ly=c). */
        for (int c = 0; c < DISP_W; c++) {
            const uint16_t *src = &fb[c * LCD_W + (DISP_H - 1 - y0)];
            for (int r = 0; r < STRIP_H; r++)
                strip_buf[r * DISP_W + c] = src[-r];
        }
#elif DISP_ROTATE == 180
        /* 180°: portrait(c, y0+r) ← logical(DISP_W-1-c, DISP_H-1-y0-r). */
        for (int r = 0; r < STRIP_H; r++) {
            const uint16_t *src = &fb[(DISP_H - 1 - y0 - r) * LCD_W + (DISP_W - 1)];
            for (int c = 0; c < DISP_W; c++)
                strip_buf[r * DISP_W + c] = src[-c];
        }
#elif DISP_ROTATE == 270
        /* 270° CCW: portrait(c, y0+r) ← logical(lx=y0+r, ly=DISP_W-1-c). */
        for (int c = 0; c < DISP_W; c++) {
            const uint16_t *src = &fb[(DISP_W - 1 - c) * LCD_W + y0];
            for (int r = 0; r < STRIP_H; r++)
                strip_buf[r * DISP_W + c] = src[r];
        }
#endif
        lcd_set_window(0, y0, DISP_W - 1, y0 + STRIP_H - 1);
        lcd_ramwr(strip_buf, STRIP_PIXELS);
    }
}

/* Orientation guide: coloured corner blocks + white top bar + up-pointing triangle.
 * RED=top-left  GREEN=top-right  BLUE=bottom-left  YELLOW=bottom-right */
static void display_orientation_guide(void)
{
    uint16_t *fb = index_buf;   /* borrow PSRAM framebuffer */
    uint16_t tl  = rgb565(180,  30,  30);
    uint16_t tr  = rgb565( 30, 160,  30);
    uint16_t bl  = rgb565( 30,  30, 200);
    uint16_t br  = rgb565(180, 180,  30);
    uint16_t wh  = rgb565(255, 255, 255);

    ESP_LOGI(TAG, "orientation: RED=TL  GREEN=TR  BLUE=BL  YELLOW=BR  WHITE=top");

    /* Fill corner blocks and top bar into framebuffer */
    for (int row = 0; row < LCD_H; row++) {
        for (int col = 0; col < LCD_W; col++) {
            bool in_top = row <= 7;
            int  tri_hw = row - 14;
            bool in_tri = row >= 14 && row < 60
                          && col >= LCD_W/2 - tri_hw
                          && col <= LCD_W/2 + tri_hw;
            bool in_tl  = row <  LCD_H/4     && col < LCD_W/6;
            bool in_tr  = row <  LCD_H/4     && col >= LCD_W - LCD_W/6;
            bool in_bl  = row >= LCD_H*3/4   && col < LCD_W/6;
            bool in_br  = row >= LCD_H*3/4   && col >= LCD_W - LCD_W/6;
            uint16_t px = 0x0000;
            if      (in_top || in_tri) px = wh;
            else if (in_tl)            px = tl;
            else if (in_tr)            px = tr;
            else if (in_bl)            px = bl;
            else if (in_br)            px = br;
            fb[row * LCD_W + col] = px;
        }
    }

    /* Overlay text labels — transparent bg so corner colour shows through */
    const uint8_t *font = u8g2_font_ncenB14_tr;
    text_draw(fb,              8,               8, "TL", font, wh, 0x0001);
    text_draw(fb, LCD_W - 46,                 8, "TR", font, wh, 0x0001);
    text_draw(fb,              8,  LCD_H - 30,    "BL", font, wh, 0x0001);
    text_draw(fb, LCD_W - 46,  LCD_H - 30,        "BR", font, wh, 0x0001);

    flush_color_fb(fb);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

/* Fill the entire display with one solid colour — diagnostic helper. */
static void display_fill(uint16_t colour)
{
    for (int i = 0; i < STRIP_PIXELS; i++) strip_buf[i] = colour;
    for (int y0 = 0; y0 < DISP_H; y0 += STRIP_H) {
        xSemaphoreTake(dma_done_sem, portMAX_DELAY);
        lcd_set_window(0, y0, DISP_W - 1, y0 + STRIP_H - 1);
        lcd_ramwr(strip_buf, STRIP_PIXELS);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   flush_frame — apply palette (with rotation) and DMA each strip.
   ═══════════════════════════════════════════════════════════════════════════ */

/* flush_frame — apply palette and DMA each strip with DISP_ROTATE software rotation. */
static void flush_frame(int pal_offset)
{
#define MAP(idx) ((idx) == IN_SET_SENTINEL ? 0x0000 \
                  : palette[((idx) + pal_offset) & PALETTE_MASK])
    for (int y0 = 0; y0 < DISP_H; y0 += STRIP_H) {
        xSemaphoreTake(dma_done_sem, portMAX_DELAY);
#if DISP_ROTATE == 0
        for (int r = 0; r < STRIP_H; r++)
            for (int c = 0; c < DISP_W; c++)
                strip_buf[r * DISP_W + c] = MAP(index_buf[(y0 + r) * LCD_W + c]);
#elif DISP_ROTATE == 90
        for (int c = 0; c < DISP_W; c++) {
            const uint16_t *src = &index_buf[c * LCD_W + (DISP_H - 1 - y0)];
            for (int r = 0; r < STRIP_H; r++)
                strip_buf[r * DISP_W + c] = MAP(src[-r]);
        }
#elif DISP_ROTATE == 180
        for (int r = 0; r < STRIP_H; r++) {
            const uint16_t *src = &index_buf[(DISP_H - 1 - y0 - r) * LCD_W + (DISP_W - 1)];
            for (int c = 0; c < DISP_W; c++)
                strip_buf[r * DISP_W + c] = MAP(src[-c]);
        }
#elif DISP_ROTATE == 270
        for (int c = 0; c < DISP_W; c++) {
            const uint16_t *src = &index_buf[(DISP_W - 1 - c) * LCD_W + y0];
            for (int r = 0; r < STRIP_H; r++)
                strip_buf[r * DISP_W + c] = MAP(src[r]);
        }
#endif
        lcd_set_window(0, y0, DISP_W - 1, y0 + STRIP_H - 1);
        lcd_ramwr(strip_buf, STRIP_PIXELS);
    }
#undef MAP
}

/* ═══════════════════════════════════════════════════════════════════════════
   Coordinate helpers
   ═══════════════════════════════════════════════════════════════════════════ */

static void screen_to_fractal(int sx, int sy,
                               float cx, float cy, float hw,
                               float *fx, float *fy)
{
    float aspect = (float)LCD_H / (float)LCD_W;
    float hh = hw * aspect;
    *fx = cx - hw + (float)sx / (float)LCD_W * (2.0f * hw);
    *fy = cy - hh + (float)sy / (float)LCD_H * (2.0f * hh);
}

/* ═══════════════════════════════════════════════════════════════════════════
   Touch
   ═══════════════════════════════════════════════════════════════════════════ */

static void touch_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = I2C_NUM_0,
        .scl_io_num        = TOUCH_SCL_IO,
        .sda_io_num        = TOUCH_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &touch_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TOUCH_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(touch_bus, &dev_cfg, &touch_dev));

    gpio_set_direction(TOUCH_RST_IO, GPIO_MODE_OUTPUT);
    gpio_set_level(TOUCH_RST_IO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TOUCH_RST_IO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static bool touch_read(int *out_x, int *out_y)
{
    /* 8-byte write command, then read 8 bytes — from ESPHome axs15231 driver */
    static const uint8_t cmd[8] = {0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x08};
    uint8_t data[8] = {0};

    esp_err_t ret = i2c_master_transmit_receive(
        touch_dev, cmd, sizeof(cmd), data, sizeof(data), 50);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "touch i2c error: %s", esp_err_to_name(ret));
        return false;
    }
    /* data[0]==0 && data[1]!=0 means touch present */
    if (data[0] != 0 || data[1] == 0) return false;

    int x = ((data[2] & 0x0F) << 8) | data[3];
    int y = ((data[4] & 0x0F) << 8) | data[5];
    ESP_LOGI(TAG, "touch raw x=%d y=%d  data=[%02x %02x %02x %02x %02x %02x %02x %02x]",
             x, y, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    /* IC x-axis (0–LCD_W) runs along the display's y-axis, and vice-versa.
     * ESPHome normalises both axes to a common range before swapping, which
     * implicitly scales.  Replicate that: swap + scale in one step. */
    /* Map IC portrait coordinates to logical screen coordinates.
     * raw_x = portrait column (0–320), raw_y = portrait row (0–480). */
#if DISP_ROTATE == 0
    x = (LCD_W - 1) - x * LCD_W / TOUCH_X_RAW_MAX;
    y = (LCD_H - 1) - y * LCD_H / TOUCH_Y_RAW_MAX;
#elif DISP_ROTATE == 90
    /* CW: portrait row → lx, portrait col → ly (mirrored) */
    { int nx = y * LCD_W / TOUCH_Y_RAW_MAX;
      y = (LCD_H - 1) - x * LCD_H / TOUCH_X_RAW_MAX;
      x = nx; }
#elif DISP_ROTATE == 180
    /* IC reversals cancel the 180° flip — direct mapping */
    x = x * LCD_W / TOUCH_X_RAW_MAX;
    y = y * LCD_H / TOUCH_Y_RAW_MAX;
#elif DISP_ROTATE == 270
    /* CCW: portrait row → lx (mirrored), portrait col → ly */
    { int nx = (LCD_W - 1) - y * LCD_W / TOUCH_Y_RAW_MAX;
      y = x * LCD_H / TOUCH_X_RAW_MAX;
      x = nx; }
#endif
    *out_x = x;
    *out_y = y;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
   Tasks
   ═══════════════════════════════════════════════════════════════════════════ */

static void touch_task(void *pv)
{
    ESP_LOGI(TAG, "touch task running");
    bool was_down  = false;
    int  up_streak = 0;   /* consecutive no-touch polls before lifting */
    while (1) {
        int tx, ty;
        bool touched = touch_read(&tx, &ty);
        if (touched) {
            up_streak = 0;
            if (!was_down) {
                was_down = true;
                float fx, fy;
                /* Map tap through the fixed home frame so any screen position
                 * always selects the same fractal point regardless of zoom depth. */
                screen_to_fractal(tx, ty, HOME_CX, HOME_CY, HOME_HW, &fx, &fy);
                ESP_LOGI(TAG, "touch screen=(%d,%d) → fractal=(%.6f,%.6f), restart zoom",
                         tx, ty, (double)fx, (double)fy);
                view_cx  = fx;
                view_cy  = fy;
                view_hw  = HOME_HW;
                rerender = true;
            }
        } else {
            if (++up_streak >= 3)   /* 3 × 20 ms = 60 ms no-touch to confirm lift */
                was_down = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/*
 * Main loop:
 *  1. Render fractal at current view into index_buf.
 *  2. Show it for ZOOM_FRAMES frames while cycling the palette.
 *  3. Zoom in by ZOOM_STEP, go to 1.
 *  Touch resets the centre and zoom, restarting the sequence.
 */
static void render_task(void *pv)
{
    int pal_offset  = 0;
    int frame_count = 0;

    display_orientation_guide();

    ESP_LOGI(TAG, "initial render…");
    render_fractal(view_cx, view_cy, view_hw);

    /* Count sentinel vs coloured pixels so we can rule out an all-black frame */
    int in_set = 0;
    for (int i = 0; i < LCD_PIX; i++)
        if (index_buf[i] == IN_SET_SENTINEL) in_set++;
    ESP_LOGI(TAG, "done — in-set %d/%d (%.1f%% black), coloured %.1f%%",
             in_set, LCD_PIX,
             100.0 * in_set / LCD_PIX,
             100.0 * (LCD_PIX - in_set) / LCD_PIX);

    while (1) {
        /* Touch caused a reset — start fresh from new position */
        if (rerender) {
            rerender    = false;
            frame_count = 0;
            ESP_LOGI(TAG, "rendering (%.5f, %.5f) hw=%.3e…",
                     (double)view_cx, (double)view_cy, (double)view_hw);
            render_fractal(view_cx, view_cy, view_hw);
            ESP_LOGI(TAG, "done");
        }

        flush_frame(pal_offset);
        pal_offset = (pal_offset + PALETTE_SPEED) & PALETTE_MASK;
        frame_count++;

        /* Auto-zoom: re-centre on the deepest boundary pixel, then zoom in */
        if (frame_count >= ZOOM_FRAMES) {
            frame_count = 0;
            view_cx = zoom_target_x;
            view_cy = zoom_target_y;
            view_hw *= ZOOM_STEP;

            if (view_hw < ZOOM_MIN) {
                view_cx = HOME_CX;
                view_cy = HOME_CY;
                view_hw = HOME_HW;
                ESP_LOGI(TAG, "zoom reset (precision floor)");
            }

            ESP_LOGI(TAG, "zoom → (%.6f, %.6f) hw=%.3e, rendering…",
                     (double)view_cx, (double)view_cy, (double)view_hw);
            render_fractal(view_cx, view_cy, view_hw);
            ESP_LOGI(TAG, "done");
        }

        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   Display init — configuration derived from ESPHome qspi_dbi/axs15231
   ═══════════════════════════════════════════════════════════════════════════ */

static void display_init(void)
{
    gpio_set_direction(LCD_BL_IO, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_BL_IO, 1);

    spi_bus_config_t bus_cfg = {0};
    bus_cfg.sclk_io_num     = LCD_CLK_IO;
    bus_cfg.data0_io_num    = LCD_D0_IO;
    bus_cfg.data1_io_num    = LCD_D1_IO;
    bus_cfg.data2_io_num    = LCD_D2_IO;
    bus_cfg.data3_io_num    = LCD_D3_IO;
    bus_cfg.data4_io_num    = -1;
    bus_cfg.data5_io_num    = -1;
    bus_cfg.data6_io_num    = -1;
    bus_cfg.data7_io_num    = -1;
    bus_cfg.max_transfer_sz = STRIP_PIXELS * sizeof(uint16_t);
    bus_cfg.flags           = SPICOMMON_BUSFLAG_QUAD;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num       = LCD_CS_IO,
        .dc_gpio_num       = -1,
        .spi_mode          = 0,   /* MODE0 — confirmed by ESPHome qspi_dbi */
        .pclk_hz           = LCD_CLK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits      = 32,
        .lcd_param_bits    = 8,
        .flags.quad_mode   = true,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io_handle));

    /* Register DMA-done callback — gives dma_done_sem from ISR */
    esp_lcd_panel_io_callbacks_t cbs = { .on_color_trans_done = on_color_trans_done };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(
        io_handle, &cbs, dma_done_sem));

    /* ESPHome init sequence for axs15231 model:
     *   SLPOUT, vendor unlock, 0xC1{0x33}, vendor lock,
     *   INVOFF, MADCTL(RGB no-flip), COLMOD(RGB565), BRIGHTNESS, DISPON */
    lcd_cmd(0x11, NULL, 0);                         /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(120));

    static const uint8_t vendor_unlock[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0xA5 };
    static const uint8_t vendor_lock[8]   = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
    static const uint8_t c1_data[1]       = { 0x33 };
    static const uint8_t madctl[1]        = { 0x00 };  /* portrait native; software rotation handles landscape */
    static const uint8_t colmod[1]        = { 0x55 };  /* 16-bit RGB565 */
    static const uint8_t brightness[1]    = { 0xD0 };

    lcd_cmd(0xBB, vendor_unlock, 8);
    lcd_cmd(0xC1, c1_data,       1);
    lcd_cmd(0xBB, vendor_lock,   8);
    lcd_cmd(0x20, NULL,          0);  /* INVOFF */
    lcd_cmd(0x36, madctl,        1);  /* MADCTL */
    lcd_cmd(0x3A, colmod,        1);  /* COLMOD */
    lcd_cmd(0x51, brightness,    1);  /* BRIGHTNESS */
    lcd_cmd(0x29, NULL,          0);  /* DISPON */
    vTaskDelay(pdMS_TO_TICKS(50));
}

/* ═══════════════════════════════════════════════════════════════════════════
   Entry point
   ═══════════════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    index_buf = heap_caps_malloc(LCD_PIX * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    strip_buf = heap_caps_malloc(STRIP_PIXELS * sizeof(uint16_t),
                                 MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(index_buf && strip_buf);

    build_palette();

    /* Create semaphore with initial count 1 so the first strip starts immediately */
    dma_done_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(dma_done_sem);

    display_init();   /* registers DMA callback — must happen after sem is created */
    text_init();
    touch_init();

    xTaskCreatePinnedToCore(render_task, "render", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(touch_task,  "touch",  4096, NULL, 4, NULL, 1);
}
