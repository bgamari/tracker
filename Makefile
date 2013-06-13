#!/usr/bin/make

CC      = arm-none-eabi-g++
CXX     = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy

PROJECT = tracker

OBJECTS = main.o clock.o dac.o feedback.o adc.o event.o scan.o uart.o syscalls.o ringbuffer.o timer.o commands.o

FLAGS = -mthumb -mcpu=cortex-m4 -Ilibopencm3/include -g3 -O0 -Wall -Werror -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-common -MD -nostartfiles -DLPC43XX
CFLAGS = $(FLAGS) -std=gnu++0x
CXXFLAGS = $(FLAGS) -Ieigen -fno-exceptions -fno-rtti
LDSCRIPT = libopencm3/lib/lpc43xx/m4/libopencm3_lpc43xx.ld
LDFLAGS = -lm -Tmdaq.ld -Llibopencm3/lib

opencm3_a = libopencm3/lib/libopencm3_lpc43xx.a

all : tracker.bin

.PHONY : $(opencm3_a)
$(opencm3_a) :
	make -Clibopencm3 lib

tracker.elf : ${OBJECTS} $(opencm3_a)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $+

tracker.bin : tracker.elf
	$(OBJCOPY) -Obinary -R.dma_data $+ $@

.PHONY : clean
clean :
	rm -f ${OBJECTS} tracker.elf tracker.bin

include $(wildcard *.d)
