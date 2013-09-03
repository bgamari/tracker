#!/usr/bin/make

CC      = arm-none-eabi-gcc
CXX     = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

PROJECT = tracker

USB_OBJECTS = usb.o usb_request.o usb_standard_request.o usb_queue.o
OBJECTS = main.o clock.o dac.o feedback.o adc.o event.o uart.o syscalls.o timer.o commands.o $(addprefix hackrf_usb/,$(USB_OBJECTS)) tracker_usb.o usb_descriptor.o buffer.o path.o

FLAGS = -mthumb -mcpu=cortex-m4 -Ilibopencm3/include -Ihackrf_usb -g3 -O3 -Wall -Werror -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-common -MD -nostartfiles -DLPC43XX -DLPC43XX_M4
CFLAGS = $(FLAGS) -std=gnu99
CXXFLAGS = $(FLAGS) -Ieigen -fno-exceptions -fno-rtti
LDFLAGS = -Tmdaq.ld -Llibopencm3/lib

opencm3_a = libopencm3/lib/libopencm3_lpc43xx.a

all : tracker.bin tracker.dfu

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
	rm -f ${OBJECTS} tracker.elf tracker.bin *.d

include $(wildcard *.d)

%.dfu: %.bin
	$(Q)rm -f _tmp.dfu _header.bin
	$(Q)cp $(*).bin _tmp.dfu
	$(Q)dfu-suffix --vid=0x1fc9 --pid=0x000c --did=0x0 -s 0 -a _tmp.dfu
	$(Q)python -c "import os.path; import struct; print('0000000: da ff ' + ' '.join(map(lambda s: '%02x' % ord(s), struct.pack('<H', os.path.getsize('$(*).bin') / 512 + 1))) + ' ff ff ff ff')" | xxd -g1 -r > _header.bin
	$(Q)cat _header.bin _tmp.dfu >$(*).dfu
	$(Q)rm -f _tmp.dfu _header.bin

program : tracker.dfu
	sudo dfu-util -D $< -d 1fc9:000c
