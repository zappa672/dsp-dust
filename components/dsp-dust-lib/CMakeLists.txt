set(component_srcs "src/buttons.c" "src/i2c0.c" "src/led_mat.c" "src/mic_adc.c" "src/pot_adc.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "esp_adc"
                       REQUIRES "")
