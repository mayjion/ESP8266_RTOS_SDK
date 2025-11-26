#ifndef UARTHANDLE_H
#define UARTHANDLE_H

#include "init_sys.h"

#define UART_BUF_SIZE 1024

void uart_init(uint32_t baud_rate, uint8_t stop_bits, uint8_t parity, uint8_t data_bits);
void forward_data_to_uart(const uint8_t *data, size_t len);

#endif // UARTHANDLE_H
