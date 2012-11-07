#!/usr/bin/make

CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy

PROJECT = tracker

OBJECTS = main.o dac.o feedback.o adc.o event.o scan.o uart.o

CFLAGS = -mthumb -mcpu=cortex-m4 -Ilibopencm3/include -DSTM32F4 -std=gnu99 -g3 -O0 -Wall -Werror
LDFLAGS = -lm

all : tracker.bin

tracker.elf : ${OBJECTS}
	$(CC) $(LDFLAGS) -o $@ $+

tracker.bin : tracker.elf
	$(OBJCOPY) $+ $@

.PHONY : clean
clean :
	rm ${OBJECTS}
