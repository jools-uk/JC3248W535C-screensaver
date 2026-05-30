#include "fractal.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "fractal";

uint16_t  palette[PALETTE_SIZE];
uint16_t *index_buf;
float     zoom_target_x = HOME_CX;
float     zoom_target_y = HOME_CY;

void fractal_init(void)
{
    index_buf = heap_caps_malloc(LCD_PIX * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    assert(index_buf);
    build_palette();
    ESP_LOGI(TAG, "fractal init done — index_buf %u bytes in PSRAM",
             (unsigned)(LCD_PIX * sizeof(uint16_t)));
}

void build_palette(void)
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

void render_fractal(float cx, float cy, float hw)
{
    float aspect = (float)LCD_H / (float)LCD_W;
    float hh = hw * aspect;
    float dx = (2.0f * hw) / (float)LCD_W;
    float dy = (2.0f * hh) / (float)LCD_H;

    int   best_iter = -1;
    float best_fx = cx, best_fy = cy;

    float py = cy - hh;
    for (int y = 0; y < LCD_H; y++, py += dy) {
        float px = cx - hw;
        for (int x = 0; x < LCD_W; x++, px += dx) {
            float cr = px, ci = py;

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

void flush_frame(int pal_offset)
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

void screen_to_fractal(int sx, int sy,
                       float cx, float cy, float hw,
                       float *fx, float *fy)
{
    float aspect = (float)LCD_H / (float)LCD_W;
    float hh = hw * aspect;
    *fx = cx - hw + (float)sx / (float)LCD_W * (2.0f * hw);
    *fy = cy - hh + (float)sy / (float)LCD_H * (2.0f * hh);
}
