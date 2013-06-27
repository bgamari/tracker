#!/usr/bin/make

CC      = arm-none-eabi-gcc
CXX     = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

PROJECT = tracker

USB_OBJECTS = tracker_usb.o usb.o usb_descriptor.o usb_request.o usb_standard_request.o
OBJECTS = main.o clock.o dac.o feedback.o adc.o event.o scan.o uart.o syscalls.o timer.o commands.o $(USB_OBJECTS)

FLAGS = -mthumb -mcpu=cortex-m4 -Ilibopencm3/include -g3 -O0 -Wall -Werror -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-common -MD -nostartfiles -DLPC43XX
CFLAGS = $(FLAGS) -std=gnu99
CXXFLAGS = $(FLAGS) -Ieigen -fno-exceptions -fno-rtti
LDSCRIPT = libopencm3/lib/lpc43xx/m4/libopencm3_lpc43xx.ld
LDFLAGS = -lm -Tmdaq.ld -Llibopencm3/lib

opencm3_a = libopencm3/lib/libopencm3_lpc43xx.a

all : tracker.bin

.PHONY : $(opencm3_a)
$(opencm3_a) :
	make -Clibopencm3 lib

tracker.elf : ${OBJECTS} $(opencm3_a)
	$(CC) $(CFLAGS) $+ $(LDFLAGS) -o $@

tracker.bin : tracker.elf
	$(OBJCOPY) -Obinary -R.dma_data $+ $@

tracker.srec : tracker.elf
	$(OBJCOPY) -Osrec -R.dma_data $+ $@

.PHONY : clean
clean :
	rm -f ${OBJECTS} tracker.elf tracker.bin

include $(wildcard *.d)

%.dfu: %.bin
	$(Q)rm -f _tmp.dfu _header.bin
	$(Q)cp $(*).bin _tmp.dfu
	$(Q)dfu-suffix --vid=0x1fc9 --pid=0x000c --did=0x0 -s 0 -a _tmp.dfu
	$(Q)python -c "import os.path; import struct; print('0000000: da ff ' + ' '.join(map(lambda s: '%02x' % ord(s), struct.pack('<H', os.path.getsize('$(*).bin') / 512 + 1))) + ' ff ff ff ff')" | xxd -g1 -r > _header.bin
	$(Q)cat _header.bin _tmp.dfu >$(*).dfu
	$(Q)rm -f _tmp.dfu _header.bin
