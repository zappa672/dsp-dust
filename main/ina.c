#include "ina.h"

#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include <driver/i2c_master.h>
#include <soc/i2c_reg.h>

#include "common.h"

#define INA219_ADDR_GND_GND 0x40 //!< I2C address, A1 pin - GND, A0 pin - GND
#define INA219_ADDR_GND_VS  0x41 //!< I2C address, A1 pin - GND, A0 pin - VS+
#define INA219_ADDR_GND_SDA 0x42 //!< I2C address, A1 pin - GND, A0 pin - SDA
#define INA219_ADDR_GND_SCL 0x43 //!< I2C address, A1 pin - GND, A0 pin - SCL
#define INA219_ADDR_VS_GND  0x44 //!< I2C address, A1 pin - VS+, A0 pin - GND
#define INA219_ADDR_VS_VS   0x45 //!< I2C address, A1 pin - VS+, A0 pin - VS+
#define INA219_ADDR_VS_SDA  0x46 //!< I2C address, A1 pin - VS+, A0 pin - SDA
#define INA219_ADDR_VS_SCL  0x47 //!< I2C address, A1 pin - VS+, A0 pin - SCL
#define INA219_ADDR_SDA_GND 0x48 //!< I2C address, A1 pin - SDA, A0 pin - GND
#define INA219_ADDR_SDA_VS  0x49 //!< I2C address, A1 pin - SDA, A0 pin - VS+
#define INA219_ADDR_SDA_SDA 0x4a //!< I2C address, A1 pin - SDA, A0 pin - SDA
#define INA219_ADDR_SDA_SCL 0x4b //!< I2C address, A1 pin - SDA, A0 pin - SCL
#define INA219_ADDR_SCL_GND 0x4c //!< I2C address, A1 pin - SCL, A0 pin - GND
#define INA219_ADDR_SCL_VS  0x4d //!< I2C address, A1 pin - SCL, A0 pin - VS+
#define INA219_ADDR_SCL_SDA 0x4e //!< I2C address, A1 pin - SCL, A0 pin - SDA
#define INA219_ADDR_SCL_SCL 0x4f //!< I2C address, A1 pin - SCL, A0 pin - SCL

/**
 * Bus voltage range
 */
typedef enum {
    INA219_BUS_RANGE_16V = 0, //!< 16V FSR
    INA219_BUS_RANGE_32V      //!< 32V FSR (default)
} ina219_bus_voltage_range_t;

/**
 * PGA gain for shunt voltage
 */
typedef enum {
    INA219_GAIN_1 = 0, //!< Gain: 1, Range: +-40 mV
    INA219_GAIN_0_5,   //!< Gain: 1/2, Range: +-80 mV
    INA219_GAIN_0_25,  //!< Gain: 1/4, Range: +-160 mV
    INA219_GAIN_0_125  //!< Gain: 1/8, Range: +-320 mV (default)
} ina219_gain_t;

/**
 * ADC resolution/averaging
 */
typedef enum {
    INA219_RES_9BIT_1S    = 0,  //!< 9 bit, 1 sample, conversion time 84 us
    INA219_RES_10BIT_1S   = 1,  //!< 10 bit, 1 sample, conversion time 148 us
    INA219_RES_11BIT_1S   = 2,  //!< 11 bit, 1 sample, conversion time 276 us
    INA219_RES_12BIT_1S   = 3,  //!< 12 bit, 1 sample, conversion time 532 us (default)
    INA219_RES_12BIT_2S   = 9,  //!< 12 bit, 2 samples, conversion time 1.06 ms
    INA219_RES_12BIT_4S   = 10, //!< 12 bit, 4 samples, conversion time 2.13 ms
    INA219_RES_12BIT_8S   = 11, //!< 12 bit, 8 samples, conversion time 4.26 ms
    INA219_RES_12BIT_16S  = 12, //!< 12 bit, 16 samples, conversion time 8.51 ms
    INA219_RES_12BIT_32S  = 13, //!< 12 bit, 32 samples, conversion time 17.02 ms
    INA219_RES_12BIT_64S  = 14, //!< 12 bit, 64 samples, conversion time 34.05 ms
    INA219_RES_12BIT_128S = 15, //!< 12 bit, 128 samples, conversion time 68.1 ms
} ina219_resolution_t;

/**
 * Operating mode
 */
typedef enum {
    INA219_MODE_POWER_DOWN = 0, //!< Power-done
    INA219_MODE_TRIG_SHUNT,     //!< Shunt voltage, triggered
    INA219_MODE_TRIG_BUS,       //!< Bus voltage, triggered
    INA219_MODE_TRIG_SHUNT_BUS, //!< Shunt and bus, triggered
    INA219_MODE_DISABLED,       //!< ADC off (disabled)
    INA219_MODE_CONT_SHUNT,     //!< Shunt voltage, continuous
    INA219_MODE_CONT_BUS,       //!< Bus voltage, continuous
    INA219_MODE_CONT_SHUNT_BUS  //!< Shunt and bus, continuous (default)
} ina219_mode_t;

#define I2C_FREQ_HZ 1000000 // Max 1 MHz for esp-idf, but supports up to 2.56 MHz

#define I2C_REG_CONFIG      0
#define I2C_REG_SHUNT_U     1
#define I2C_REG_BUS_U       2
#define I2C_REG_POWER       3
#define I2C_REG_CURRENT     4
#define I2C_REG_CALIBRATION 5


/* Device descriptor */
static const i2c_port_t port = 0;
static const uint8_t addrs[4] = {
    INA219_ADDR_GND_GND,
    INA219_ADDR_GND_VS,
    INA219_ADDR_VS_GND,
    INA219_ADDR_VS_VS,
};

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t *i2c_devs[4];

// static uint16_t ina219_cfg;

static esp_err_t i2c_setup_port()
{
    ESP_LOGI(TAG, "i2c_setup_port: %d", port);

    i2c_master_bus_config_t i2c_cfg;
    memset(&i2c_cfg, 0, sizeof(i2c_master_bus_config_t));
    i2c_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_cfg.i2c_port = port;
    i2c_cfg.scl_io_num = GPIO_NUM_22;
    i2c_cfg.sda_io_num = GPIO_NUM_21;
    i2c_cfg.glitch_ignore_cnt = 7;
    i2c_cfg.flags.enable_internal_pullup = true;
    
    ESP_LOGI(TAG, "i2c_setup_port: %d", port);

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    ESP_LOGI(TAG, "i2c_setup_port: %d", port);

    for (int i = 0; i < 4; i++) {
        esp_err_t res = i2c_master_probe(i2c_bus, addrs[i], I2C_TIME_OUT_REG_V);

        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Found device at addr 0x%02x", addrs[i]);
            
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addrs[i],
                .scl_speed_hz = I2C_FREQ_HZ,
            };

            i2c_devs[i] = malloc(sizeof(i2c_master_dev_handle_t));
            ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, i2c_devs[i]));
        } else {
            i2c_devs[i] = NULL;
            ESP_LOGI(TAG, "Not found device at addr 0x%02x", addrs[i]);
        }
    }

    return ESP_OK;
}

// esp_err_t i2c_dev_read(const void *out_data, size_t out_size, void *in_data, size_t in_size)
// {
//     if (!in_data || !in_size) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     esp_err_t res = i2c_setup_port();
//     if (res != ESP_OK) {
//         return res;
//     }

//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     if (out_data && out_size)
//     {
//         i2c_master_start(cmd);
//         i2c_master_write_byte(cmd, addrs[0] << 1, true);
//         i2c_master_write(cmd, (void *)out_data, out_size, true);
//     }
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (addrs[0] << 1) | 1, true);
//     i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
//     i2c_master_stop(cmd);

//     res = i2c_master_cmd_begin(port, cmd, I2C_TIME_OUT_REG_V);
//     if (res != ESP_OK)
//         ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", addrs[0], port, res);

//     i2c_cmd_link_delete(cmd);
//     return res;
// }

// inline esp_err_t i2c_dev_read_reg(uint8_t reg, void *in_data, size_t in_size)
// {
//     return i2c_dev_read(&reg, 1, in_data, in_size);
// }

// static esp_err_t read_reg_16(uint8_t reg, uint16_t *val)
// {
//     if (!val) return ESP_ERR_INVALID_ARG; 
    
//     esp_err_t r = i2c_dev_read_reg(reg, val, 2);
//     if (r != ESP_OK) {
//         return r;
//     }

//     *val = (*val >> 8) | (*val << 8);

//     return ESP_OK;
// }

void configure_ina_i2c() {
  esp_err_t res;

  res = i2c_setup_port();
  if (res != ESP_OK) { 
    return;
  }
  
//   ESP_LOGD(TAG, "Reading ina219 config %d", port);

//   res = read_reg_16(I2C_REG_CONFIG, &ina219_cfg);
//   if (res != ESP_OK) { 
//     return;
//   }
  
//   ESP_LOGD(TAG, "Ina219 config: 0x%04x", ina219_cfg);
}

void ina_read_task(void *arg) {
  while (true)
  {
    usleep(1000*1000);
  }
  
  // // Read register 0 to get the current value
  // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  
  // i2c_master_start(cmd);
  // i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
  // i2c_master_read_byte(cmd, &current, ACK_VAL);
  // i2c_master_stop(cmd);
  
  // i2c_cmd_link_perform(I2C_NUM_0, cmd);
  // // Process the register value to get the current
  // int16_t current = (current << 8);
  
  // // Update the current value
  // Serial.println(current);
}