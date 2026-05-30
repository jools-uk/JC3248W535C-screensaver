#include "touch.h"
#include "display.h"   /* for DISP_ROTATE, LCD_W, LCD_H */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "touch";

static i2c_master_bus_handle_t touch_bus;
static i2c_master_dev_handle_t touch_dev;

void touch_init(void)
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

bool touch_read(int *out_x, int *out_y)
{
    static const uint8_t cmd[8] = {0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x08};
    uint8_t data[8] = {0};

    esp_err_t ret = i2c_master_transmit_receive(
        touch_dev, cmd, sizeof(cmd), data, sizeof(data), 50);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c error: %s", esp_err_to_name(ret));
        return false;
    }
    if (data[0] != 0 || data[1] == 0) return false;

    int x = ((data[2] & 0x0F) << 8) | data[3];
    int y = ((data[4] & 0x0F) << 8) | data[5];
    ESP_LOGI(TAG, "raw x=%d y=%d", x, y);

    /* Map IC portrait coordinates to logical screen coordinates.
     * raw_x = portrait column (0–TOUCH_X_RAW_MAX)
     * raw_y = portrait row    (0–TOUCH_Y_RAW_MAX)
     * IC physical axes are both reversed from portrait-intuitive direction. */
#if DISP_ROTATE == 0
    x = (LCD_W - 1) - x * LCD_W / TOUCH_X_RAW_MAX;
    y = (LCD_H - 1) - y * LCD_H / TOUCH_Y_RAW_MAX;
#elif DISP_ROTATE == 90
    { int nx = y * LCD_W / TOUCH_Y_RAW_MAX;
      y = (LCD_H - 1) - x * LCD_H / TOUCH_X_RAW_MAX;
      x = nx; }
#elif DISP_ROTATE == 180
    x = x * LCD_W / TOUCH_X_RAW_MAX;
    y = y * LCD_H / TOUCH_Y_RAW_MAX;
#elif DISP_ROTATE == 270
    { int nx = (LCD_W - 1) - y * LCD_W / TOUCH_Y_RAW_MAX;
      y = x * LCD_H / TOUCH_X_RAW_MAX;
      x = nx; }
#endif

    *out_x = x;
    *out_y = y;
    return true;
}
