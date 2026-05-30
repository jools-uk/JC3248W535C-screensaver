/*
 * display_test.c — standalone AXS15231B QSPI display driver validation.
 *
 * Initialises the display via the ESPHome-derived init sequence and fills the
 * screen with repeating R/G/B stripe blocks to verify:
 *   – SPI bus + QSPI IO setup
 *   – CASET / RASET window addressing
 *   – RAMWR / RAMWRC pixel data path
 *   – Byte-order and colour correctness
 *
 * Build with:  idf.py build -DDISPLAY_TEST=ON
 * No fractal, touch, or UI code required.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "display_test";

#define LCD_CLK_IO  47
#define LCD_D0_IO   21
#define LCD_D1_IO   48
#define LCD_D2_IO   40
#define LCD_D3_IO   39
#define LCD_CS_IO   45
#define LCD_BL_IO    1

/* Portrait panel dimensions */
#define W  320
#define H  480

/* Stripe block: 320×4 pixels per colour (4 portrait rows per band) */
#define BLOCK             (W * 4)
#define MAX_TRANSFER_SIZE (BLOCK * 3)   /* one full R+G+B cycle */

static esp_lcd_panel_io_handle_t io;
static SemaphoreHandle_t         done_sem;

/* ── RGB565 byte-swapped for little-endian DMA (display expects MSB first) ─ */
#define RED   0x00F8u   /* native 0xF800 */
#define GREEN 0xE007u   /* native 0x07E0 */
#define BLUE  0x1F00u   /* native 0x001F */

static void lcd_cmd(uint8_t c, const uint8_t *data, size_t len)
{
    int v = (int)(((uint32_t)0x02 << 24) | ((uint32_t)c << 8));
    esp_lcd_panel_io_tx_param(io, v, data, len);
}

static void lcd_color(const uint16_t *px, int n, bool cont)
{
    uint8_t cmd = cont ? 0x3C : 0x2C;   /* RAMWRC : RAMWR */
    int v = (int)(((uint32_t)0x32 << 24) | ((uint32_t)cmd << 8));
    esp_lcd_panel_io_tx_color(io, v, px, n * sizeof(uint16_t));
}

static bool IRAM_ATTR on_done(esp_lcd_panel_io_handle_t h,
                               esp_lcd_panel_io_event_data_t *e, void *ctx)
{
    BaseType_t w = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)ctx, &w);
    return w == pdTRUE;
}

static void fill_stripes(void)
{
    static uint16_t seg[MAX_TRANSFER_SIZE];
    for (int i =        0; i < BLOCK;   i++) seg[i] = RED;
    for (int i = BLOCK;   i < BLOCK*2;  i++) seg[i] = GREEN;
    for (int i = BLOCK*2; i < BLOCK*3;  i++) seg[i] = BLUE;

    /* Full portrait window: RASET 0-479, CASET 0-319 */
    static const uint8_t raset[] = {0x00, 0x00, 0x01, 0xDF}; /* rows 0-479 */
    static const uint8_t caset[] = {0x00, 0x00, 0x01, 0x3F}; /* cols 0-319 */
    lcd_cmd(0x2B, raset, 4);
    lcd_cmd(0x2A, caset, 4);

    /* W*H / MAX_TRANSFER_SIZE sends cover the full 320×480 display */
    for (int i = 0; i < W * H / MAX_TRANSFER_SIZE; i++) {
        xSemaphoreTake(done_sem, portMAX_DELAY);
        lcd_color(seg, MAX_TRANSFER_SIZE, i > 0);
    }
    xSemaphoreTake(done_sem, portMAX_DELAY);
    xSemaphoreGive(done_sem);
}

void app_main(void)
{
    gpio_set_direction(LCD_BL_IO, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_BL_IO, 1);

    spi_bus_config_t bus = {
        .sclk_io_num     = LCD_CLK_IO,
        .data0_io_num    = LCD_D0_IO,
        .data1_io_num    = LCD_D1_IO,
        .data2_io_num    = LCD_D2_IO,
        .data3_io_num    = LCD_D3_IO,
        .data4_io_num    = -1,
        .data5_io_num    = -1,
        .data6_io_num    = -1,
        .data7_io_num    = -1,
        .max_transfer_sz = MAX_TRANSFER_SIZE * sizeof(uint16_t),
        .flags           = SPICOMMON_BUSFLAG_QUAD,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num       = LCD_CS_IO,
        .dc_gpio_num       = -1,
        .spi_mode          = 0,
        .pclk_hz           = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits      = 32,
        .lcd_param_bits    = 8,
        .flags.quad_mode   = true,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));

    done_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(done_sem);

    esp_lcd_panel_io_callbacks_t cbs = { .on_color_trans_done = on_done };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io, &cbs, done_sem));

    /* ESPHome axs15231 init sequence — portrait, no MADCTL overrides */
    lcd_cmd(0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));

    static const uint8_t unlock[] = {0,0,0,0,0,0,0x5A,0xA5};
    static const uint8_t lock[]   = {0,0,0,0,0,0,0,0};
    static const uint8_t c1[]     = {0x33};
    static const uint8_t madctl[] = {0x00};   /* portrait, no rotation */
    static const uint8_t colmod[] = {0x55};   /* RGB565 */
    static const uint8_t bright[] = {0xD0};

    lcd_cmd(0xBB, unlock, 8);
    lcd_cmd(0xC1, c1,     1);
    lcd_cmd(0xBB, lock,   8);
    lcd_cmd(0x20, NULL,   0);
    lcd_cmd(0x36, madctl, 1);
    lcd_cmd(0x3A, colmod, 1);
    lcd_cmd(0x51, bright, 1);
    lcd_cmd(0x29, NULL,   0);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "filling stripes — R/G/B bands, %d rows each", BLOCK / W);
    fill_stripes();
    ESP_LOGI(TAG, "done");

    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
