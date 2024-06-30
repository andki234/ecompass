#ifndef UART_WRAPPER_H
#define UART_WRAPPER_H

#include <stdio.h>
#include "driver/uart.h"

// UART Configuration
#define UART_NUM                    UART_NUM_1
#define UART_TX_PIN                 17
#define UART_RX_PIN                 16
#define UART_BAUD_RATE              115200
#define UART_BUFFER_SIZE            (1024)

// Function to initialize UART
void uart_init(void);

#endif // UART_WRAPPER_H
