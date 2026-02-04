#ifndef UART_SERIAL_H
#define UART_SERIAL_H

#include <stdint.h>
#include "queue.h"

QUEUE_DECLARATION(uart_queue, char, 64);

extern uint8_t uart_txing;
extern struct uart_queue u_queue;

extern void uart_init();
extern void uart_printchar(char c);
extern void uart_printstr(const char *data);
extern void uart_println(const char *data);
extern void uart_printint(int32_t n, uint8_t newline);

#endif // UART_SERIAL_H