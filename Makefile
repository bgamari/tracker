#!/usr/bin/make

PREFIX ?= arm-none-eabi

PROJECT = tracker

OBJECTS = main.o dac.o beagle_spi.o feedback.o adc.o event.o scan.o uart.o

CFLAGS = -Ilibopencm3/include -DSTM32F4 -std=gnu99 -g3 -O0 -Wall
LDFLAGS = -lm

all : tracker.bin

tracker.elf : ${OBJECTS}
tracker.bin : tracker.elf
	$(PREFIX)-objcopy $+ $@

.PHONY : clean
clean :
	rm ${OBJECTS}
