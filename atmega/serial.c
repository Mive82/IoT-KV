// Ako F_CPU nije definiran, build env nije setupan kako treba
// setbaud.h ce svejedno bacit error

// Postavljanje baudrate-a za uart
#ifndef BAUD
#define BAUD 115200
#endif


#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <stdlib.h>
#include "setbaud.h"
#include "serial.h"
#include "queue.h"

uint8_t uart_txing = 0;

QUEUE_DEFINITION(uart_queue, char);

struct uart_queue u_queue;

void uart_init() {
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	UCSR0C = (_BV(UCSZ01) | _BV(UCSZ00));
	UCSR0B = _BV(TXEN0);
	
	// Neke brzine zahtjevaju ovo, header setbaud.h ce postaviti makro
	// pri compile time-u
	// Vidi poglavlje 19.11 u datasheetu
	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif

	uart_queue_init(&u_queue);
	uart_txing = 0;
}

void uart_printchar(char c) {
	uart_queue_enqueue(&u_queue, &c);
}

void uart_printstr(const char *data) {
	while(*data)
		uart_printchar(*data++);
}

void uart_println(const char *data) {
	while(*data)
		uart_printchar(*data++);
	// Carriadge Return \r i Line Feed \n za pravilne new lineove, aka CRLF
	uart_printchar('\r');
	uart_printchar('\n');
}

void uart_printint(int32_t n, uint8_t newline)  {
	char str[11]; // Max int_32 ima 10 znamenki, +1 za \0
	ltoa(n, str, 10);
	if(newline)
	{
		uart_println(str);
	}
	else
	{
		uart_printstr(str);
	}
}