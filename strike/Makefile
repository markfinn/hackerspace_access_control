#*************************************************************************
#Title:    MRB-DOOR Electric Lock Strike Controller
#Authors:  Michael Petersen <railfan@drgw.net>
#          Nathan Holmes <maverick@drgw.net>
#File:     Makefile
#License:  GNU General Public License v3
#
#LICENSE:
#    Copyright (C) 2015 Nathan Holmes and Michael Petersen
#
#    Derived from example WinAVR project Makefile
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 3 of the License, or
#    any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#    
#    You should have received a copy of the GNU General Public License along 
#    with this program. If not, see http://www.gnu.org/licenses/
#    
#*************************************************************************

BASE_NAME = mrb-door

# ATmega328P 20MHz @ 5.0V
DEVICE  = atmega328p
F_CPU   = 20000000  # Hz
FUSE_L  = 0xD7
FUSE_H  = 0xD1
FUSE_E  = 0x04
BOOTSTART = 0x7000  #0x3800 in words

GITREV := 'GIT_REV=0x$(shell git rev-parse --short=6 HEAD)L'

MRBOOTPATH = ../../MRBus-Bootloader
BUSLOAD = python $(MRBOOTPATH)/load.py -p /dev/ttyUSB0 -a 0xfe -d 0x10

MRBUS_DIRECTORY=../mrbus2/src
I2CLIB_DIRECTORY=./avr-i2c
PROGRAMMER_TYPE=avrispmkii
#PROGRAMMER_TYPE=usbtiny
PROGRAMMER_PORT=usb

# MRBus
DEFINES = -DMRBUS -DENABLE_WATCHDOG -DBOOTSTART=$(BOOTSTART) -D$(GITREV)
SRCS = $(BASE_NAME).c $(MRBUS_DIRECTORY)/mrbus-avr.c $(MRBUS_DIRECTORY)/mrbus-crc.c $(MRBUS_DIRECTORY)/mrbus-queue.c 
INCS = $(MRBUS_DIRECTORY)/mrbus.h $(MRBUS_DIRECTORY)/mrbus-avr.h

# PROGRAMMER_TYPE defines the programmer to be used
# Common options are usbtiny and avrispmkii

AVRDUDE = avrdude -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(DEVICE) -B1 -F
AVRDUDE_SLOW = avrdude -P $(PROGRAMMER_PORT) -c $(PROGRAMMER_TYPE) -p $(DEVICE) -B32 -F

OBJS = ${SRCS:.c=.o}
INCLUDES = -I. -I$(MRBUS_DIRECTORY) -I$(I2CLIB_DIRECTORY)
CFLAGS  = $(INCLUDES) -Wall -O2 -std=gnu99 -ffunction-sections -fdata-sections
LDFLAGS = -Wl,-gc-sections

COMPILE = avr-gcc $(DEFINES) -DF_CPU=$(F_CPU) $(CFLAGS) $(LDFLAGS) -mmcu=$(DEVICE)

# -E -dM options preprocess only and output results of #define
#COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE) -E -dM

help:
	@echo "make setup...... init/update git submodules"
	@echo "make hex ....... build $(BASE_NAME).hex"
	@echo "make flash ..... flash the firmware"
	@echo "make fuse ...... flash the fuses"
	@echo "make program ... flash fuses and firmware"
	@echo "make busload ... flash the firmware via mrbus bootloader"
	@echo "make read ...... read the fuses"
	@echo "make size ...... memory usage"
	@echo "make clean ..... delete objects and hex file"
	@echo "make terminal... open up avrdude terminal"

hex: $(BASE_NAME).hex

setup:
	git submodule init
	git submodule update

program: fuse flash

terminal:
	$(AVRDUDE_SLOW) -t

# rule for programming fuse bits:
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" -a "$(FUSE_E)" != "" ] || \
		{ echo "*** Invalid Fuse values."; exit 1; }
	$(AVRDUDE_SLOW) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m -U efuse:w:$(FUSE_E):m

read:
	$(AVRDUDE) -v

size:
	avr-size -C --mcu=$(DEVICE) $(BASE_NAME).elf

# rule for uploading firmware:
flash: $(BASE_NAME).hex
	$(AVRDUDE) -U flash:w:$(BASE_NAME).hex:i

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f $(BASE_NAME).hex $(BASE_NAME).lst $(BASE_NAME).obj $(BASE_NAME).cof $(BASE_NAME).list $(BASE_NAME).map $(BASE_NAME).eep.hex $(BASE_NAME).elf $(BASE_NAME).s $(OBJS) *.o *.tgz *~

# Generic rule for compiling C files:
.c.o: $(INCS)
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

# Generic rule for compiling C to assembler, used for debugging only.
.c.s:
	$(COMPILE) -S $< -o $@

# file targets:

$(BASE_NAME).elf: $(OBJS)
	$(COMPILE) -o $(BASE_NAME).elf $(OBJS)

$(BASE_NAME).hex: $(BASE_NAME).elf
	rm -f $(BASE_NAME).hex $(BASE_NAME).eep.hex
	avr-objcopy -j .text -j .data -O ihex $(BASE_NAME).elf $(BASE_NAME).hex
	avr-size $(BASE_NAME).hex

# debugging targets:

disasm:	$(BASE_NAME).elf
	avr-objdump -d $(BASE_NAME).elf

PWD := $(shell pwd)

# rule for uploading firmware:
busload: $(BASE_NAME).hex
	$(BUSLOAD) -x -r -c -s -- $(BASE_NAME).hex

busreset:
	$(BUSLOAD) -x -r


