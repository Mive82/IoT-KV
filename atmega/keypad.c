#include <avr/io.h>
#include <avr/portpins.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include "keypad.h"
#include "serial.h"


static const char* key_chars = "123A456B789C*0#D";

static void row_output_column_input()
{
	// PB0 - PB3 as output
	ROW_CTRL |= ROW_BM;
	// PD4 - PD7 as input
	COL_CTRL &= ~COL_BM;

	// Pullup enable
	OUT_COL |= COL_BM;

	OUT_ROW &= ~ROW_BM;
}

static void row_input_column_output()
{
	// PB0 - PB3 as input
	ROW_CTRL &= ~ROW_BM;
	// PD4 - PD7 as output
	COL_CTRL |= COL_BM;

	// Pullup enable
	OUT_ROW |= ROW_BM;

	// OUT_COL &= ~COL_BM;
}

char scan_keys()
{
	uint8_t key_pressed = 0;

	uint8_t output_bm = 0;

	if(!(IN_COL & ((uint8_t)1 << COL1)))
	{
		key_pressed = 0;
		output_bm = ~((uint8_t)1 << COL1);
	}
	else if(!(IN_COL & (1 << COL2)))
	{
		key_pressed = 1;
		output_bm = ~((uint8_t)1 << COL2);
	} else if(!(IN_COL & (1 << COL3)))
	{
		key_pressed = 2;
		output_bm = ~((uint8_t)1 << COL3);
	} else if(!(IN_COL & (1 << COL4)))
	{
		key_pressed = 3;
		output_bm = ~((uint8_t)1 << COL4);
	}

	row_input_column_output();

	output_bm &= COL_BM;

	OUT_COL = (OUT_COL & ~COL_BM ) | output_bm;

	_delay_loop_1(10);

	if(!(IN_ROW & (1 << ROW1)))
	{
		key_pressed += 0;
	} else if(!(IN_ROW & (1 << ROW2)))
	{
		key_pressed += 4;
	} else if(!(IN_ROW & (1 << ROW3)))
	{
		key_pressed += 8;
	} else if(!(IN_ROW & (1 << ROW4)))
	{
		key_pressed += 12;
	}

	row_output_column_input();

	return key_chars[key_pressed];
}

uint8_t is_button_pressed(void)
{
	return (IN_COL & COL_BM) != COL_BM;
}

char check_keys()
{
	char retval = 0;
	// State 0
	if((IN_COL & COL_BM) != COL_BM)
	{
		_delay_ms(10);
		// State 1 - initial debounce
		if((IN_COL & COL_BM) != COL_BM)
		{
			PORTB |= (1 << PB5);
			// State 2 - Key is pressed, check which one
			retval = scan_keys();
			// State 3 - last debounce, wait until it's let go
			while((IN_COL & COL_BM) != COL_BM);
			PORTB &= ~(1 << PB5);
		}
	}

	return retval;
}


/*
- All row pins will be high
- All column pins will be pulled-up
- The active row will be low

*/

void keypad_init()
{
	ROW_CTRL &= ~ROW_BM;
	COL_CTRL &= ~COL_BM;

	OUT_ROW |= ROW_BM;
	OUT_COL |= COL_BM;

	DDRB |= (1 << PB5);

	row_output_column_input();
}