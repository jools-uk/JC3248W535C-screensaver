/*
 * Mandelbrot fractal screensaver — JC3248W535C (ESP32-S3, AXS15231B display).
 *
 * Modules:
 *   display.c — QSPI hardware, strip DMA, software rotation
 *   fractal.c — Mandelbrot render, palette, flush_frame
 *   touch.c   — AXS15231B I2C touch IC
 *   ui.c      — u8g2 text engine, orientation splash
 */

/* Set DISP_ROTATE in config.h (0=portrait, 90=landscape CW, 180, 270). */
#include "display.h"
#include "fractal.h"
#include "touch.h"
#include "ui.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "main";

/* ── Shared view state — written by touch_task, read by render_task ───────── */
volatile float view_cx  = HOME_CX;
volatile float view_cy  = HOME_CY;
volatile float view_hw  = HOME_HW;
volatile bool  rerender = false;
int            tap_anchor_steps = 0;

/* ═══════════════════════════════════════════════════════════════════════════
   Touch task
   ═══════════════════════════════════════════════════════════════════════════ */

static void touch_task(void *pv)
{
    ESP_LOGI(TAG, "touch task running");
    bool was_down  = false;
    int  up_streak = 0;
    while (1) {
        int tx, ty;
        bool touched = touch_read(&tx, &ty);
        if (touched) {
            up_streak = 0;
            if (!was_down) {
                was_down = true;
                float fx, fy;
                screen_to_fractal(tx, ty, HOME_CX, HOME_CY, HOME_HW, &fx, &fy);
                ESP_LOGI(TAG, "tap (%d,%d) → fractal (%.4f, %.4f)", tx, ty,
                         (double)fx, (double)fy);
                view_cx          = fx;
                view_cy          = fy;
                view_hw          = HOME_HW;
                zoom_target_x    = fx;
                zoom_target_y    = fy;
                tap_anchor_steps = 5;
                rerender         = true;
            }
        } else {
            if (++up_streak >= 3)
                was_down = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   Render task
   ═══════════════════════════════════════════════════════════════════════════ */

static void render_task(void *pv)
{
    int pal_offset  = 0;
    int frame_count = 0;

    display_orientation_guide(index_buf);

    ESP_LOGI(TAG, "initial render…");
    render_fractal(view_cx, view_cy, view_hw);

    int in_set = 0;
    for (int i = 0; i < LCD_PIX; i++)
        if (index_buf[i] == IN_SET_SENTINEL) in_set++;
    ESP_LOGI(TAG, "done — in-set %d/%d (%.1f%%)",
             in_set, LCD_PIX, 100.0 * in_set / LCD_PIX);

    while (1) {
        if (rerender) {
            rerender    = false;
            frame_count = 0;
            ESP_LOGI(TAG, "render (%.4f, %.4f) hw=%.3e",
                     (double)view_cx, (double)view_cy, (double)view_hw);
            render_fractal(view_cx, view_cy, view_hw);
        }

        flush_frame(pal_offset);
        pal_offset = (pal_offset + PALETTE_SPEED) & PALETTE_MASK;
        frame_count++;

        if (frame_count >= ZOOM_FRAMES) {
            frame_count = 0;
            if (tap_anchor_steps > 0) {
                tap_anchor_steps--;
            } else {
                view_cx = zoom_target_x;
                view_cy = zoom_target_y;
            }
            view_hw *= ZOOM_STEP;

            if (view_hw < ZOOM_MIN) {
                view_cx = HOME_CX;
                view_cy = HOME_CY;
                view_hw = HOME_HW;
                ESP_LOGI(TAG, "zoom reset (precision floor)");
            }

            ESP_LOGI(TAG, "zoom → (%.4f, %.4f) hw=%.3e",
                     (double)view_cx, (double)view_cy, (double)view_hw);
            render_fractal(view_cx, view_cy, view_hw);
        }

        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   Entry point
   ═══════════════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    fractal_init();   /* allocates index_buf in PSRAM, builds palette */

    dma_done_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(dma_done_sem);

    display_init();   /* SPI + QSPI IO + panel init sequence */
    text_init();      /* u8g2 null-display font engine */
    touch_init();     /* I2C touch IC */

    xTaskCreatePinnedToCore(render_task, "render", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(touch_task,  "touch",  4096, NULL, 4, NULL, 1);
}
