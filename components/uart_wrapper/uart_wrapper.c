#include "uart_wrapper.h"
#include "esp_log.h"
#include "esp_vfs_dev.h" // Standard I/O Redirection: VFS allows us to redirect stdout and stdin to UART. This enables the use of printf for logging and debugging over the UART.

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0);

    // Redirect stdout and stderr to UART
    esp_vfs_dev_uart_use_driver(UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(UART_NUM, ESP_LINE_ENDINGS_CRLF);

    // Reassign the standard input/output/error streams to UART
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    esp_vfs_dev_uart_use_nonblocking(UART_NUM);
    
    // Initialize the UART for use with the esp_log library
    esp_log_set_vprintf(vprintf);
}


