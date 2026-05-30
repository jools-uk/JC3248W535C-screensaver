#include "ui.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "u8g2.h"

static const char *TAG = "ui";

static u8g2_t  u8g2_ctx;
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

void text_init(void)
{
    u8x8_SetupDefaults(&u8g2_ctx.u8x8);
    u8g2_ctx.u8x8.display_cb        = u8x8_null_cb;
    u8g2_ctx.u8x8.gpio_and_delay_cb = u8x8_null_cb;
    u8g2_ctx.u8x8.display_info      = &u8g2_info;

    u8g2_SetupBuffer(&u8g2_ctx, u8g2_buf, LCD_H / 8,
                     u8g2_ll_hvline_vertical_top_lsb, U8G2_R0);
    u8g2_InitDisplay(&u8g2_ctx);
    u8g2_SetPowerSave(&u8g2_ctx, 0);
    ESP_LOGI(TAG, "u8g2 text engine ready");
}

void text_draw(uint16_t *fb, int x, int y, const char *str,
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

void display_orientation_guide(uint16_t *fb)
{
    uint16_t tl = rgb565(180,  30,  30);
    uint16_t tr = rgb565( 30, 160,  30);
    uint16_t bl = rgb565( 30,  30, 200);
    uint16_t br = rgb565(180, 180,  30);
    uint16_t wh = rgb565(255, 255, 255);

    ESP_LOGI(TAG, "orientation: RED=TL  GREEN=TR  BLUE=BL  YELLOW=BR  WHITE=top");

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

    const uint8_t *font = u8g2_font_ncenB14_tr;
    text_draw(fb,              8,            8, "TL", font, wh, 0x0001);
    text_draw(fb, LCD_W - 46,              8, "TR", font, wh, 0x0001);
    text_draw(fb,              8, LCD_H - 30, "BL", font, wh, 0x0001);
    text_draw(fb, LCD_W - 46, LCD_H - 30,    "BR", font, wh, 0x0001);

    flush_color_fb(fb);
    vTaskDelay(pdMS_TO_TICKS(2000));
}
