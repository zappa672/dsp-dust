#include "i2c0.h"
#include "i2c0_ina219.h"

#include <string.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"
#include "soc/i2c_reg.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_vendor.h"

#include "lvgl.h"

#include "common.h"

/* Device descriptor */
static const uint8_t ina219_addrs[DSP_DUST_INA_CNT] = {
    INA219_ADDR_GND_GND,
    INA219_ADDR_GND_VS,
    INA219_ADDR_VS_GND,
    INA219_ADDR_VS_VS,
};

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t *i2c_devs[DSP_DUST_INA_CNT];

static uint8_t read_buffer[2];

#define SSD1306_PIXEL_CLOCK_HZ  (400 * 1000)
#define SSD1306_I2C_ADDR        0x3C
#define SSD1306_H_RES           128
#define SSD1306_V_RES           64
#define SSD1306_CMD_BITS        8
#define SSD1306_PARAM_BITS      8


void example_lvgl_demo_ui(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP); /* Circular scroll */
    lv_label_set_text(label, "Hello Espressif, Hello LVGL.");
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}



esp_err_t i2c_0_init()
{
    esp_err_t res;
    i2c_master_bus_config_t i2c_cfg;

    ESP_LOGI(TAG, "i2c_0_init");

    memset(&i2c_cfg, 0, sizeof(i2c_master_bus_config_t));
    i2c_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_cfg.i2c_port = I2C_NUM_0;
    i2c_cfg.scl_io_num = GPIO_NUM_22;
    i2c_cfg.sda_io_num = GPIO_NUM_21;
    i2c_cfg.glitch_ignore_cnt = 7;
    i2c_cfg.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    ESP_LOGI(TAG, "i2c_0_ina219_init");

    for (int i = 0; i < DSP_DUST_INA_CNT; i++) {
        res = i2c_master_probe(i2c_bus, ina219_addrs[i], I2C_TIME_OUT_REG_V);

        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Found device at addr 0x%02x", ina219_addrs[i]);
            
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = ina219_addrs[i],
                .scl_speed_hz = I2C_FREQ_HZ,
            };

            i2c_devs[i] = malloc(sizeof(i2c_master_dev_handle_t));
            ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, i2c_devs[i]));

            ESP_ERROR_CHECK(i2c_master_receive(*i2c_devs[i], read_buffer, 2, -1));

            uint8_t data_addr = INA219_I2C_REG_CONFIG;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(*i2c_devs[i], &data_addr, 1, read_buffer, 2, INA219_I2C_TIMEOUT_MS));

            ESP_LOGI(TAG, "0x%02x cfg 0x%02x 0x%02x", ina219_addrs[i], read_buffer[0], read_buffer[1]);

            // EPS_ERROR_CHECK();
        } else {
            i2c_devs[i] = NULL;
            ESP_LOGI(TAG, "Not found device at addr 0x%02x", ina219_addrs[i]);
        }
    }

    ESP_LOGI(TAG, "i2c_0_lcd_init");

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = SSD1306_I2C_ADDR,
        .scl_speed_hz = SSD1306_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = SSD1306_CMD_BITS,
        .lcd_param_bits = SSD1306_PARAM_BITS,
        .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = SSD1306_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = SSD1306_H_RES * SSD1306_V_RES,
        .double_buffer = true,
        .hres = SSD1306_H_RES,
        .vres = SSD1306_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_180);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (lvgl_port_lock(0)) {
        example_lvgl_demo_ui(disp);
        // Release the mutex
        lvgl_port_unlock();
    }

    return ESP_OK;
}

void i2c_0_clean(void) {
    // i2c_master_bus_rm_device() or i2c_del_master_bus()
}


    // for (int i = 0; i < 4; i++) {
    //     data_addr = INA219_I2C_REG_SHUNT_U;
    //     ESP_ERROR_CHECK(i2c_master_transmit_receive(*i2c_devs[i], &data_addr, 1, read_buffer, 2, INA219_I2C_TIMEOUT_MS));

    //     raw = (read_buffer[0] << 8) + read_buffer[1];
    //     raw = (raw >> 8) | (raw << 8);

    //     // ESP_LOGI(TAG, "0x%02x shunt U %f", addrs[i], raw / 100000.0);

    //     data_addr = INA219_I2C_REG_SHUNT_U;
    //     ESP_ERROR_CHECK(i2c_master_transmit_receive(*i2c_devs[i], &data_addr, 1, read_buffer, 2, INA219_I2C_TIMEOUT_MS));

    //     raw = (read_buffer[0] << 8) + read_buffer[1];
    //     raw = (raw >> 8) | (raw << 8);

    //     // ESP_LOGI(TAG, "0x%02x bus U %f", addrs[i], (raw >> 3) * 0.004);
    // }
    
