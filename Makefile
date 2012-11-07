#!/usr/bin/make

CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy

PROJECT = tracker

OBJECTS = main.o clock.o dac.o feedback.o adc.o event.o scan.o uart.o

CFLAGS = -mthumb -mcpu=cortex-m4 -Ilibopencm3/include -DSTM32F4 -std=gnu99 -g3 -Os -Wall -Werror -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-common -MD -nostartfiles
LDFLAGS = -lm -Tudaq.ld -Llibopencm3/lib

opencm3_a = libopencm3/lib/libopencm3_stm32f4.a

all : tracker.bin

.PHONY : $(opencm3_a)
$(opencm3_a) :
	make -Clibopencm3 lib

tracker.elf : ${OBJECTS} $(opencm3_a)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $+

tracker.bin : tracker.elf
	$(OBJCOPY) $+ $@

.PHONY : clean
clean :
	rm ${OBJECTS} tracker.elf tracker.bin
