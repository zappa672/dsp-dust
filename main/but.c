#include "but.h"

// static void buttons_read_task(void *arg)
// {
//     uart_config_t uart_config = {
//         .baud_rate = BUTTONS_UART_BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT,
//     };

//     ESP_ERROR_CHECK(uart_driver_install(BUTTONS_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 1));
//     ESP_ERROR_CHECK(uart_param_config(BUTTONS_UART_PORT_NUM, &uart_config));
//     ESP_ERROR_CHECK(uart_set_pin(BUTTONS_UART_PORT_NUM, -1, BUTTONS_UART_RX_GPIO, -1, -1));

//     char *data = (char *) malloc(BUF_SIZE);

//     while (1) {
//         // Read data from the UART
//         int len = uart_read_bytes(BUTTONS_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
//         if (len) {
//             for (int i = 0; i < len; i++) {
//                 if (data[i] >= '0' && data[i] < '6') {
//                     button_id = (int)(data[i] - '0');
//                     ESP_LOGI(TAG, "Recieved button %d" click, button_id);
//                 }
//             }
//         }
//     }

//     free(data);
//     uart_driver_delete(BUTTONS_UART_PORT_NUM);
// }