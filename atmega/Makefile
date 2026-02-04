FILES ?= $(wildcard *.c)
OBJS = $(patsubst %c, %o, $(FILES))

DEVICE     ?= atmega328p
CLOCK      ?= 16000000
PROGRAMMER ?= arduino
PORT       ?= /dev/ttyACM0
BAUD       ?= 115200
FILENAME   ?= main

CC          = avr-gcc 
OBJCOPY     = avr-objcopy
SIZE        = avr-size
CFLAGS      = -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -DMIVE_DEBUG=1
AVRDUDE     = avrdude

## -------------------------------------------- ##
## ------ TARGETI/TASKOVI POČINJU ODAVDE ------ ##
## -------------------------------------------- ##

.PHONY: all clean build upload

all: build upload clean

build: $(OBJS)
	$(CC) $(CFLAGS) $^ -o $(FILENAME).elf
	$(OBJCOPY) -j .text -j .data -O ihex $(FILENAME).elf $(FILENAME).hex
	$(SIZE) --format=avr --mcu=$(DEVICE) $(FILENAME).elf

%.o: %.c
	$(info Building object $@)
	$(CC) $(CFLAGS) $< -c -o $@

upload:
	$(AVRDUDE) -v -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(FILENAME).hex 

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex
