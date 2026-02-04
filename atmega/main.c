#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/twi.h>

#include "queue.h"
#include "events.h"
#include "serial.h"
#include "keypad.h"
#include "motor.h"

#define KEYPAD_DEBOUNCE 4
#define LIMITSW_DEBOUNCE 15

enum garage_state_e
{
	// Invalid state, not supposed to appear
	GARAGE_INVALIC = 0,
	// Stopped states -> Next state is opening
	GARAGE_CLOSED = 1,
	GARAGE_CLOSING_STOPPED,
	// Stopped states -> Next state is closing
	GARAGE_OPEN,
	GARAGE_OPENING_STOPPED,

	// Moving states
	GARAGE_OPENING,
	GARAGE_CLOSING,
};

union garage_i2c_reg
{
	uint8_t reg;
	struct {
		uint8_t state : 7;
		uint8_t command : 1;
	} s;
};

// ====== Settings ======

// Valid code for the keypad
static const uint16_t valid_code = 1111;
// I2C address to use as slave
static const uint8_t i2c_address = 0x20;

// ====== Limit switch debouncing stuff ======
static uint8_t pd2_pending_state = (1 << PD2);
static uint8_t pd2_state = (1 << PD2);
static uint8_t pd2_poll_time = 0;
static uint8_t pd2_polling = 0;

static uint8_t pd3_pending_state = (1 << PD3);
static uint8_t pd3_state = (1 << PD3);
static uint8_t pd3_poll_time = 0;
static uint8_t pd3_polling = 0;

// ====== Millis, sort of ======
// Do NOT directly compare with ==, use <= or >=
// Overflows every 65.536 seconds
static uint16_t millis = 0;

// ====== Keypad state and button stuff ======
static uint8_t keypad_state = 0;
static uint8_t keypad_ms_delay = 0;
static char keypad_key = 0;

static uint16_t code_val;
static uint8_t code_chars;

// ====== Program events ======
struct event_queue e_queue;

// ====== Garage stuff ======
uint8_t garage_state = GARAGE_CLOSED;

uint8_t i2c_read_state = 0;

ISR(TWI_vect)
{
	struct garage_event event;
	union garage_i2c_reg reg_val;
	reg_val.reg = 0;
	switch (TW_STATUS)
	{
	// Got addressed for "write", reset state
	case TW_SR_SLA_ACK:
		i2c_read_state = 0;	
		TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
	break;
	// Data received from master
	case TW_SR_DATA_ACK:
		reg_val.reg = TWDR;
		// First is the "address" byte. We don't care about it
		if(i2c_read_state == 0)
		{
			i2c_read_state = 1;
		}
		// Next is actual data
		else{
			if(reg_val.s.command)
			{
				EVENT_SET(event, EVENT_ACTUATE_DOOR);
				event_queue_enqueue(&e_queue, &event);
			}
		}
		TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
		break;
	// Read request
	case TW_ST_SLA_ACK:
		reg_val.reg = 0;
		reg_val.s.state = garage_state;
		TWDR = reg_val.reg;
		TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
		break;
	// Additional read request, send NACK
	case TW_ST_DATA_ACK:
		TWCR = (1 << TWIE) | (1 << TWEN) | (1 << TWINT);
		break;
	default:
		i2c_read_state = 0;
		TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
		break;
	}
}

// 250Hz Timer
ISR(TIMER0_COMPA_vect)
{
	garage_event_t event;
	uint8_t pd2_new_state;
	uint8_t pd3_new_state;
	char uart_char;

	// Since the timer is 250 Hz, 4 milliseconds have actually passed
	millis += 4;

	// Read switch states
	pd2_new_state = PIND & (1 << PD2);
	pd3_new_state = PIND & (1 << PD3);

	// Closed limit switch
	if(pd2_new_state != pd2_pending_state)
	{
		pd2_poll_time = LIMITSW_DEBOUNCE;
		pd2_pending_state = pd2_new_state;
		pd2_polling = 1;
	} else if(pd2_poll_time > 0)
	{
		pd2_poll_time--;
	} else if (pd2_poll_time == 0 && pd2_polling)
	{
		pd2_state = pd2_pending_state;
		pd2_polling = 0;
		if(pd2_state == 0)
		{
			EVENT_SET(event, EVENT_CLOSED_LIMIT_SWITCH_PRESSED);
		}
		else{
			EVENT_SET(event, EVENT_CLOSED_LIMIT_SWITCH_RELEASED);
		}
		event_queue_enqueue(&e_queue, &event);
	}

	// Open limit switch
	if(pd3_new_state != pd3_pending_state)
	{
		pd3_poll_time = LIMITSW_DEBOUNCE;
		pd3_pending_state = pd3_new_state;
		pd3_polling = 1;
	} else if(pd3_poll_time > 0)
	{
		pd3_poll_time--;
	} else if (pd3_poll_time == 0 && pd3_polling)
	{
		pd3_state = pd3_pending_state;
		pd3_polling = 0;
		if(pd3_state == 0)
		{
			EVENT_SET(event, EVENT_OPEN_LIMIT_SWITCH_PRESSED);
		}
		else{
			EVENT_SET(event, EVENT_OPEN_LIMIT_SWITCH_RELEASED);
		}
		event_queue_enqueue(&e_queue, &event);
	}

	// Other stuff

	// Keypad
	switch (keypad_state)
	{
	case 0:
		{
			if(is_button_pressed())
			keypad_state = 1;
			keypad_ms_delay = KEYPAD_DEBOUNCE;
		}
		break;
	case 1:
		{
			if(keypad_ms_delay)
			{
				--keypad_ms_delay;
			}
			else
			{
				if(is_button_pressed())
				{
					keypad_state = 2;
				}
			}
		}
		break;
	case 2:
		{
			PORTB |= (1 << PB5);
			keypad_key = scan_keys();
			keypad_state = 3;
		}
		break;
	case 3:
		{
			if(!is_button_pressed())
			{
				PORTB &= ~(1 << PB5);
				EVENT_SET_DATA(event, EVENT_KEYPAD_NEW_KEY, keypad_key);
				event_queue_enqueue(&e_queue, &event);
				keypad_state = 0;
			}
		}
		break;
	default:
		// Invalid state
		break;
	}

	if(!uart_txing && !uart_queue_is_empty(&u_queue))
	{
		uart_queue_dequeue(&u_queue, &uart_char);
		uart_txing = 1;
		UDR0 = uart_char;
	}

}

static void timer_setup_250hz(void)
{
	// 16Mhz / 256 = 62.5kHz
	TCCR0B |= (1 << CS02);
	// Timer counts to 250 before triggering an interrupt
	// 62.5 kHz / 250 = 250Hz
	OCR0A = 250;

	// Setup timer modes and interrupts
	TCCR0A |= (1 << WGM01);
	TIMSK0 |= (1 << OCIE0A);
}

static void i2c_setup(void)
{
	PORTC |= (1 << PC4) | (1 << PC5);

	TWAR = i2c_address << 1;

	// Configure i2c as a slave device
	TWCR = (1 << TWIE) | (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
}

int main(void)
{
	uint16_t last_millis = 0;
	garage_event_t event;
	char key;

	event_queue_init(&e_queue);

	// Init limit switches
	DDRD &= ~((1 << PD2) | (1 << PD3));
	PORTD |= (1 << PD2) | (1 << PD3);

	i2c_setup();
	uart_init();
	keypad_init();
	motor_init();

	timer_setup_250hz();

	sei();

	while (1)
	{
		while(event_queue_dequeue(&e_queue, &event) == DEQUEUE_RESULT_SUCCESS)
		{
			if(IS_EVENT_SET(event, EVENT_KEYPAD_NEW_KEY))
			{
				key = EVENT_GET_DATA(event);
				// uart_printchar(key);
				// uart_println("");
				if(key >= '0' && key <= '9')
				{
					if(code_chars == 4)
					{
						code_val = 0;
						code_chars = 0;
					}
					++code_chars;
					code_val = (code_val * 10) + (key - '0');
					uart_printint(code_val, 1);
				} else if(key == '#')
				{
					if((code_chars == 4) && (code_val == valid_code))
					{
						uart_println("Valid code");
						if(garage_state >= GARAGE_OPENING)
						{
							EVENT_SET(event, EVENT_STOP_DOOR);
						}
						else {
							EVENT_SET(event, EVENT_START_DOOR);
						}
						event_queue_enqueue(&e_queue, &event);
					}
					uart_println("Reset code");
					code_chars = 0;
					code_val = 0;
				}
			}
			else if(IS_EVENT_SET(event, EVENT_ACTUATE_DOOR))
			{
				if(garage_state >= GARAGE_OPENING)
				{
					EVENT_SET(event, EVENT_STOP_DOOR);
				}
				else {
					EVENT_SET(event, EVENT_START_DOOR);
				}
				event_queue_enqueue(&e_queue, &event);
			}
			else if(IS_EVENT_SET(event, EVENT_CLOSED_LIMIT_SWITCH_PRESSED))
			{
				motor_stop();
				garage_state = GARAGE_CLOSED;
				uart_println("Closed limit switch pressed");
			}
			else if(IS_EVENT_SET(event, EVENT_CLOSED_LIMIT_SWITCH_RELEASED))
			{
				uart_println("Closed limit switch released");
			}
			else if(IS_EVENT_SET(event, EVENT_OPEN_LIMIT_SWITCH_PRESSED))
			{
				motor_stop();
				garage_state = GARAGE_OPEN;
				uart_println("Open limit switch pressed");
			}
			else if(IS_EVENT_SET(event, EVENT_OPEN_LIMIT_SWITCH_RELEASED))
			{
				uart_println("Open limit switch released");
			}
			else if(IS_EVENT_SET(event, EVENT_START_DOOR))
			{
				if(garage_state >= GARAGE_OPEN)
				{
					motor_start_closing();
					garage_state = GARAGE_CLOSING;
				}
				else
				{
					motor_start_opening();
					garage_state = GARAGE_OPENING;
				}
			}
			else if(IS_EVENT_SET(event, EVENT_STOP_DOOR))
			{
				motor_stop();
				if(garage_state == GARAGE_OPENING)
				{
					garage_state = GARAGE_OPENING_STOPPED;
				}
				else if (garage_state == GARAGE_CLOSING)
				{
					garage_state = GARAGE_CLOSING_STOPPED;
				}
				else {
					// Invalid state. Limit switches handle this
				}
			}
		}
		while(uart_queue_dequeue(&u_queue, &key) == DEQUEUE_RESULT_SUCCESS)
		{
			loop_until_bit_is_set(UCSR0A, UDRE0);
			UDR0 = key;
		}
		// Put the CPU to sleep until the next interrupt
		set_sleep_mode(0);
		sleep_mode();
	}
	

	return 0;
}
