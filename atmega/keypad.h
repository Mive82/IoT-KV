#ifndef _MIVE_KEYPAD_H
#define _MIVE_KEYPAD_H

#define COL_CTRL DDRD
#define OUT_COL PORTD
#define IN_COL PIND
#define COL1 PD7 // PCINT23
#define COL2 PD6 // PCINT22
#define COL3 PD5 // PCINT21
#define COL4 PD4 // PCINT20
#define COL_BM ((1 << COL1) | (1 << COL2) | (1 << COL3) | (1 << COL4))

#define ROW_CTRL DDRC
#define OUT_ROW PORTC
#define IN_ROW PINC
#define ROW1 PC0
#define ROW2 PC1
#define ROW3 PC2
#define ROW4 PC3
#define ROW_BM ((1 << ROW1) | (1 << ROW2) | (1 << ROW3) | (1 << ROW4))

void keypad_init(void);
char check_keys(void);

uint8_t is_button_pressed(void);
char scan_keys(void);

#endif