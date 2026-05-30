#include "display.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "display";

SemaphoreHandle_t         dma_done_sem;
uint16_t                 *strip_buf;
esp_lcd_panel_io_handle_t io_handle;

static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                           esp_lcd_panel_io_event_data_t *edata,
                                           void *user_ctx)
{
    BaseType_t awoken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)user_ctx, &awoken);
    return awoken == pdTRUE;
}

static void lcd_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    int c = (int)(((uint32_t)0x02 << 24) | ((uint32_t)cmd << 8));
    esp_lcd_panel_io_tx_param(io_handle, c, data, len);
}

void lcd_set_window(int x0, int y0, int x1, int y1)
{
    uint8_t rs[4] = { y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF };
    uint8_t cs[4] = { x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF };
    lcd_cmd(0x2B, rs, 4);
    lcd_cmd(0x2A, cs, 4);
}

void lcd_ramwr(const uint16_t *data, int n_pixels)
{
    int c = (int)(((uint32_t)0x32 << 24) | ((uint32_t)0x2C << 8));
    esp_lcd_panel_io_tx_color(io_handle, c, data, n_pixels * sizeof(uint16_t));
}

void display_fill(uint16_t colour)
{
    for (int i = 0; i < STRIP_PIXELS; i++) strip_buf[i] = colour;
    for (int y0 = 0; y0 < DISP_H; y0 += STRIP_H) {
        xSemaphoreTake(dma_done_sem, portMAX_DELAY);
        lcd_set_window(0, y0, DISP_W - 1, y0 + STRIP_H - 1);
        lcd_ramwr(strip_buf, STRIP_PIXELS);
    }
}

void flush_color_fb(const uint16_t *fb)
{
    for (int y0 = 0; y0 < DISP_H; y0 += STRIP_H) {
        xSemaphoreTake(dma_done_sem, portMAX_DELAY);
#if DISP_ROTATE == 0
        for (int r = 0; r < STRIP_H; r++)
            for (int c = 0; c < DISP_W; c++)
                strip_buf[r * DISP_W + c] = fb[(y0 + r) * LCD_W + c];
#elif DISP_ROTATE == 90
        for (int c = 0; c < DISP_W; c++) {
            const uint16_t *src = &fb[c * LCD_W + (DISP_H - 1 - y0)];
            for (int r = 0; r < STRIP_H; r++)
                strip_buf[r * DISP_W + c] = src[-r];
        }
#elif DISP_ROTATE == 180
        for (int r = 0; r < STRIP_H; r++) {
            const uint16_t *src = &fb[(DISP_H - 1 - y0 - r) * LCD_W + (DISP_W - 1)];
            for (int c = 0; c < DISP_W; c++)
                strip_buf[r * DISP_W + c] = src[-c];
        }
#elif DISP_ROTATE == 270
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

void display_init(void)
{
    gpio_set_direction(LCD_BL_IO, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_BL_IO, 1);

    strip_buf = heap_caps_malloc(STRIP_PIXELS * sizeof(uint16_t),
                                 MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(strip_buf);

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
        .spi_mode          = 0,
        .pclk_hz           = LCD_CLK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits      = 32,
        .lcd_param_bits    = 8,
        .flags.quad_mode   = true,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io_handle));

    esp_lcd_panel_io_callbacks_t cbs = { .on_color_trans_done = on_color_trans_done };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(
        io_handle, &cbs, dma_done_sem));

    lcd_cmd(0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));

    static const uint8_t vendor_unlock[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0xA5 };
    static const uint8_t vendor_lock[8]   = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
    static const uint8_t c1_data[1]       = { 0x33 };
    static const uint8_t madctl[1]        = { 0x00 };
    static const uint8_t colmod[1]        = { 0x55 };
    static const uint8_t brightness[1]    = { 0xD0 };

    lcd_cmd(0xBB, vendor_unlock, 8);
    lcd_cmd(0xC1, c1_data,       1);
    lcd_cmd(0xBB, vendor_lock,   8);
    lcd_cmd(0x20, NULL,          0);
    lcd_cmd(0x36, madctl,        1);
    lcd_cmd(0x3A, colmod,        1);
    lcd_cmd(0x51, brightness,    1);
    lcd_cmd(0x29, NULL,          0);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "display init done");
}
