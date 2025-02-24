#include "i2c0.h"

#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "soc/i2c_reg.h"

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

/* Bus voltage range */
typedef enum {
    INA219_BUS_RANGE_16V = 0, //!< 16V FSR
    INA219_BUS_RANGE_32V      //!< 32V FSR (default)
} ina219_bus_voltage_range_t;

/* PGA gain for shunt voltage */
typedef enum {
    INA219_GAIN_1 = 0, //!< Gain: 1, Range: +-40 mV
    INA219_GAIN_0_5,   //!< Gain: 1/2, Range: +-80 mV
    INA219_GAIN_0_25,  //!< Gain: 1/4, Range: +-160 mV
    INA219_GAIN_0_125  //!< Gain: 1/8, Range: +-320 mV (default)
} ina219_gain_t;

/* ADC resolution/averaging */
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

/* Operating mode */
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

/* I2C register addresses */
typedef enum {
    INA219_I2C_REG_CONFIG = 0,
    INA219_I2C_REG_SHUNT_U = 1,
    INA219_I2C_REG_BUS_U = 2,
    INA219_I2C_REG_POWER = 3,
    INA219_I2C_REG_CURRENT = 4,
    INA219_I2C_REG_CALIBRATION = 5,
} ina219_reg_t;

#define I2C_FREQ_HZ 1000000 // Max 1 MHz for esp-idf, but supports up to 2.56 MHz


typedef enum {
    DSP_DUST_INA_PWR = 0,
    DSP_DUST_INA_CHRG = 1,
    DSP_DUST_INA_DCHG = 2,
    DSP_DUST_INA_STB = 3,
    DSP_DUST_INA_CNT = 4
} dsp_dust_ina_t;

/* Device descriptor */
static const uint8_t addrs[DSP_DUST_INA_CNT] = {
    INA219_ADDR_GND_GND,
    INA219_ADDR_GND_VS,
    INA219_ADDR_VS_GND,
    INA219_ADDR_VS_VS,
};

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t *i2c_devs[DSP_DUST_INA_CNT];

static uint8_t read_buffer[2];

#define INA219_I2C_TIMEOUT_MS (50)


esp_err_t i2c_0_init(void)
{
    esp_err_t res;
    i2c_master_bus_config_t i2c_cfg;
    
    ESP_LOGI(TAG, "ina219_i2c_init");

    memset(&i2c_cfg, 0, sizeof(i2c_master_bus_config_t));
    i2c_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_cfg.i2c_port = I2C_NUM_0;
    i2c_cfg.scl_io_num = GPIO_NUM_22;
    i2c_cfg.sda_io_num = GPIO_NUM_21;
    i2c_cfg.glitch_ignore_cnt = 7;
    i2c_cfg.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    for (int i = 0; i < DSP_DUST_INA_CNT; i++) {
        res = i2c_master_probe(i2c_bus, addrs[i], I2C_TIME_OUT_REG_V);

        if (res == ESP_OK) {
            ESP_LOGI(TAG, "Found device at addr 0x%02x", addrs[i]);
            
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addrs[i],
                .scl_speed_hz = I2C_FREQ_HZ,
            };

            i2c_devs[i] = malloc(sizeof(i2c_master_dev_handle_t));
            ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, i2c_devs[i]));

            ESP_ERROR_CHECK(i2c_master_receive(*i2c_devs[i], read_buffer, 2, -1));

            uint8_t data_addr = INA219_I2C_REG_CONFIG;
            ESP_ERROR_CHECK(i2c_master_transmit_receive(*i2c_devs[i], &data_addr, 1, read_buffer, 2, INA219_I2C_TIMEOUT_MS));

            ESP_LOGI(TAG, "0x%02x cfg 0x%02x 0x%02x", addrs[i], read_buffer[0], read_buffer[1]);

            // EPS_ERROR_CHECK();
        } else {
            i2c_devs[i] = NULL;
            ESP_LOGI(TAG, "Not found device at addr 0x%02x", addrs[i]);
        }
    }

    return ESP_OK;
}

void i2c_0_clean(void) {
    // i2c_master_bus_rm_device() or i2c_del_master_bus()
}

void i2c_0_task(void *arg) {
  uint8_t data_addr;
  uint16_t raw;

  while (true)
  {
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
    
    usleep(1000*1000);
  }
}
