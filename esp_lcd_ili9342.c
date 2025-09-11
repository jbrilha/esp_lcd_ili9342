#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <sys/cdefs.h>

#include "esp_lcd_ili9342.h"

#define EXCT 0xC8
#define PWR_CTL1 0xC0
#define PWR_CTL2 0xC1
#define VCOM_CTL1 0xC5
#define RGB_IFACE 0xB0
#define IFACE_CTL 0xF6
#define PGAM_CTL 0xE0 // positive gamma control
#define NGAM_CTL 0xE1 // negative gamma control
#define DISP_CTL 0xB6

static const char *TAG = "ili9342";

static esp_err_t panel_ili9342_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ili9342_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ili9342_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ili9342_draw_bitmap(esp_lcd_panel_t *panel, int x_start,
                                           int y_start, int x_end, int y_end,
                                           const void *color_data);
static esp_err_t panel_ili9342_invert_color(esp_lcd_panel_t *panel,
                                            bool invert_color_data);
static esp_err_t panel_ili9342_mirror(esp_lcd_panel_t *panel, bool mirror_x,
                                      bool mirror_y);
static esp_err_t panel_ili9342_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ili9342_set_gap(esp_lcd_panel_t *panel, int x_gap,
                                       int y_gap);
static esp_err_t panel_ili9342_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save current value of LCD_CMD_COLMOD register
    const ili9342_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
} ili9342_panel_t;

esp_err_t
esp_lcd_new_panel_ili9342(const esp_lcd_panel_io_handle_t io,
                          const esp_lcd_panel_dev_config_t *panel_dev_config,
                          esp_lcd_panel_handle_t *ret_panel) {
    esp_err_t ret = ESP_OK;
    ili9342_panel_t *ili9342 = NULL;
    gpio_config_t io_conf = {0};

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG,
                      err, TAG, "invalid argument");
    ili9342 = (ili9342_panel_t *)calloc(1, sizeof(ili9342_panel_t));
    ESP_GOTO_ON_FALSE(ili9342, ESP_ERR_NO_MEM, err, TAG,
                      "no mem for ili9342 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num;
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG,
                          "configure GPIO for RST line failed");
    }

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        ili9342->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        ili9342->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG,
                          "unsupported color space");
        break;
    }
#else
    switch (panel_dev_config->rgb_endian) {
    case LCD_RGB_ENDIAN_RGB:
        ili9342->madctl_val = 0;
        break;
    case LCD_RGB_ENDIAN_BGR:
        ili9342->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG,
                          "unsupported rgb endian");
        break;
    }
#endif

    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        ili9342->colmod_val = 0x55;
        ili9342->fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        ili9342->colmod_val = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte,
        // which means 3 full bytes are required for a pixel
        ili9342->fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG,
                          "unsupported pixel width");
        break;
    }

    ili9342->io = io;
    ili9342->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ili9342->reset_level = panel_dev_config->flags.reset_active_high;
    if (panel_dev_config->vendor_config) {
        ili9342->init_cmds =
            ((ili9342_vendor_config_t *)panel_dev_config->vendor_config)
                ->init_cmds;
        ili9342->init_cmds_size =
            ((ili9342_vendor_config_t *)panel_dev_config->vendor_config)
                ->init_cmds_size;
    }
    ili9342->base.del = panel_ili9342_del;
    ili9342->base.reset = panel_ili9342_reset;
    ili9342->base.init = panel_ili9342_init;
    ili9342->base.draw_bitmap = panel_ili9342_draw_bitmap;
    ili9342->base.invert_color = panel_ili9342_invert_color;
    ili9342->base.set_gap = panel_ili9342_set_gap;
    ili9342->base.mirror = panel_ili9342_mirror;
    ili9342->base.swap_xy = panel_ili9342_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    ili9342->base.disp_off = panel_ili9342_disp_on_off;
#else
    ili9342->base.disp_on_off = panel_ili9342_disp_on_off;
#endif
    *ret_panel = &(ili9342->base);
    ESP_LOGD(TAG, "new ili9342 panel @%p", ili9342);

    ESP_LOGI(TAG, "LCD panel create success, version: %d.%d.%d", 0, 0, 1);

    return ESP_OK;

err:
    if (ili9342) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(ili9342);
    }
    return ret;
}

static esp_err_t panel_ili9342_del(esp_lcd_panel_t *panel) {
    ili9342_panel_t *ili9342 = __containerof(panel, ili9342_panel_t, base);

    if (ili9342->reset_gpio_num >= 0) {
        gpio_reset_pin(ili9342->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del ili9342 panel @%p", ili9342);
    free(ili9342);
    return ESP_OK;
}

static esp_err_t panel_ili9342_reset(esp_lcd_panel_t *panel) {
    ili9342_panel_t *ili9342 = __containerof(panel, ili9342_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9342->io;

    // perform hardware reset
    if (ili9342->reset_gpio_num >= 0) {
        gpio_set_level(ili9342->reset_gpio_num, ili9342->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(ili9342->reset_gpio_num, !ili9342->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(
            esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG,
            "send command failed");
        vTaskDelay(pdMS_TO_TICKS(
            20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t data_bytes;
} lcd_init_cmd_t;

static const ili9342_lcd_init_cmd_t vendor_specific_init_default[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {EXCT, (uint8_t[]){0xFF, 0x93, 0x42}, 3, 0},
    {PWR_CTL1, (uint8_t[]){0x12, 0x12}, 2, 0},
    {PWR_CTL2, (uint8_t[]){0x03}, 1, 0},
    {VCOM_CTL1, (uint8_t[]){0xF2}, 1, 0},
    {RGB_IFACE, (uint8_t[]){0xE0}, 1, 0},
    {IFACE_CTL, (uint8_t[]){0x01, 0x00, 0x00}, 3, 0},
    {PGAM_CTL, (uint8_t[]){0x00, 0x0C, 0x11, 0x04, 0x11, 0x08, 0x37, 0x89, 0x4C,
        0x06, 0x0C, 0x0A, 0x2E, 0x34, 0x0F}, 15, 0},
    {NGAM_CTL, (uint8_t[]){0x00, 0x0B, 0x11, 0x05, 0x13, 0x09, 0x33, 0x67, 0x48,
        0x07, 0x0E, 0x0B, 0x2E, 0x33, 0x0F}, 15, 0},
    {DISP_CTL, (uint8_t[]){0x08, 0x82, 0x1D, 0x04}, 4, 0},
};

static esp_err_t panel_ili9342_init(esp_lcd_panel_t *panel) {
    ili9342_panel_t *ili9342 = __containerof(panel, ili9342_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9342->io;

    // LCD goes into sleep mode and display will be turned off after power on
    // reset, exit sleep mode first
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0),
                        TAG, "send command failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL,
                                                  (uint8_t[]){
                                                      ili9342->madctl_val,
                                                  },
                                                  1),
                        TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD,
                                                  (uint8_t[]){
                                                      ili9342->colmod_val,
                                                  },
                                                  1),
                        TAG, "send command failed");

    const ili9342_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;
    if (ili9342->init_cmds) {
        init_cmds = ili9342->init_cmds;
        init_cmds_size = ili9342->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) /
                         sizeof(ili9342_lcd_init_cmd_t);
    }

    bool is_cmd_overwritten = false;
    for (int i = 0; i < init_cmds_size; i++) {
        // Check if the command has been used or conflicts with the internal
        switch (init_cmds[i].cmd) {
        case LCD_CMD_MADCTL:
            is_cmd_overwritten = true;
            ili9342->madctl_val = ((uint8_t *)init_cmds[i].data)[0];
            break;
        case LCD_CMD_COLMOD:
            is_cmd_overwritten = true;
            ili9342->colmod_val = ((uint8_t *)init_cmds[i].data)[0];
            break;
        default:
            is_cmd_overwritten = false;
            break;
        }

        if (is_cmd_overwritten) {
            ESP_LOGW(TAG,
                     "The %02Xh command has been used and will be overwritten "
                     "by external initialization sequence",
                     init_cmds[i].cmd);
        }

        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd,
                                                      init_cmds[i].data,
                                                      init_cmds[i].data_bytes),
                            TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
    }
    ESP_LOGD(TAG, "send init commands success");

    return ESP_OK;
}

static esp_err_t panel_ili9342_draw_bitmap(esp_lcd_panel_t *panel, int x_start,
                                           int y_start, int x_end, int y_end,
                                           const void *color_data) {
    ili9342_panel_t *ili9342 = __containerof(panel, ili9342_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) &&
           "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = ili9342->io;

    x_start += ili9342->x_gap;
    x_end += ili9342->x_gap;
    y_start += ili9342->y_gap;
    y_end += ili9342->y_gap;

    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET,
                                                  (uint8_t[]){
                                                      (x_start >> 8) & 0xFF,
                                                      x_start & 0xFF,
                                                      ((x_end - 1) >> 8) & 0xFF,
                                                      (x_end - 1) & 0xFF,
                                                  },
                                                  4),
                        TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET,
                                                  (uint8_t[]){
                                                      (y_start >> 8) & 0xFF,
                                                      y_start & 0xFF,
                                                      ((y_end - 1) >> 8) & 0xFF,
                                                      (y_end - 1) & 0xFF,
                                                  },
                                                  4),
                        TAG, "send command failed");
    // transfer frame buffer
    size_t len =
        (x_end - x_start) * (y_end - y_start) * ili9342->fb_bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);

    return ESP_OK;
}

static esp_err_t panel_ili9342_invert_color(esp_lcd_panel_t *panel,
                                            bool invert_color_data) {
    ili9342_panel_t *ili9342 = __containerof(panel, ili9342_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9342->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "send command failed");
    return ESP_OK;
}

static esp_err_t panel_ili9342_mirror(esp_lcd_panel_t *panel, bool mirror_x,
                                      bool mirror_y) {
    ili9342_panel_t *ili9342 = __containerof(panel, ili9342_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9342->io;
    if (mirror_x) {
        ili9342->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        ili9342->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        ili9342->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        ili9342->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL,
                                  (uint8_t[]){ili9342->madctl_val}, 1),
        TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_ili9342_swap_xy(esp_lcd_panel_t *panel, bool swap_axes) {
    ili9342_panel_t *ili9342 = __containerof(panel, ili9342_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9342->io;
    if (swap_axes) {
        ili9342->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        ili9342->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL,
                                  (uint8_t[]){ili9342->madctl_val}, 1),
        TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_ili9342_set_gap(esp_lcd_panel_t *panel, int x_gap,
                                       int y_gap) {
    ili9342_panel_t *ili9342 = __containerof(panel, ili9342_panel_t, base);
    ili9342->x_gap = x_gap;
    ili9342->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_ili9342_disp_on_off(esp_lcd_panel_t *panel,
                                           bool on_off) {
    ili9342_panel_t *ili9342 = __containerof(panel, ili9342_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9342->io;
    int command = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    on_off = !on_off;
#endif

    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "send command failed");
    return ESP_OK;
}
