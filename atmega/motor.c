#include <avr/io.h>
#include "serial.h"

// PB1 (OC1A) - IN1
// PB2 (OC1B) - IN2
void motor_init()
{
	// Fast PWM 10bit
	// WGM10, WGM11, WGM12
	DDRB |= (1 << PB1) | (1 << PB2);

	TCCR1A |= (1 << WGM10) | (1 << WGM11) | (1 << COM1A1);
	TCCR1B |= (1 << WGM12) | (1 << CS10);

	OCR1A = 0;
	OCR1B = 0;

  uart_println(__func__);

}

void motor_stop(void)
{
#ifndef MIVE_DEBUG
  OCR1A = 0;
	OCR1B = 0;
#else
  uart_println(__func__);
#endif
}

void motor_start_opening(void)
{
#ifndef MIVE_DEBUG
	OCR1B = 0;
  OCR1A = 250;
#else
  uart_println(__func__);
#endif
}

void motor_start_closing(void)
{
#ifndef MIVE_DEBUG
	OCR1A = 0;
  OCR1B = 250;
#else
  uart_println(__func__);
#endif
}