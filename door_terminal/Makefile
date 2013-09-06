#*************************************************************************
#Title:    MRB-door_terminal Makefile
#Authors:  Mark Finn <mark@mfinn.net>
#License:  GNU General Public License v3
#
#    Derived from MRB-AVRTEMPLATE

BASE_NAME = door_terminal
MRBUSLIB = ../mrbus-lib

#DEVICE  = atmega48
#F_CPU   = 20000000  # Hz
#FUSE_L  = 0xD7
#FUSE_H  = 0xD4
#FUSE_E  = 0x00

# ATmega164P 20MHz @ 5.0V
#DEVICE  = atmega164p
#F_CPU   = 20000000  # Hz
#FUSE_L  = 0xD7
#FUSE_H  = 0xD1
#FUSE_E  = 0x04

# ATmega328P 20MHz @ 5.0V
DEVICE  = atmega328p
F_CPU   = 20000000  # Hz
FUSE_L  = 0xD7
FUSE_H  = 0xD1
FUSE_E  = 0x04

# ATmega328P 12MHz @ 3.3V
#DEVICE  = atmega328p
#F_CPU   = 12000000  # Hz
#FUSE_L  = 0xD7
#FUSE_H  = 0xD1
#FUSE_E  = 0x05

# ATmega328P 3.579545MHz @ 3.3V
#DEVICE  = atmega328p
#F_CPU   = 3579545   # Hz
#FUSE_L  = 0x97      # Clock output on PB0 (Debug)
#FUSE_H  = 0xD1
#FUSE_E  = 0x05

# ATmega48 20MHz @ 5.0V
#DEVICE  = atmega48
#F_CPU   = 20000000  # Hz
#FUSE_L  = 0xD7
#FUSE_H  = 0xD4
#FUSE_E  = 0x00

# ATmega48 11.0592MHz @ 3.3V
#DEVICE  = atmega48
#F_CPU   = 11059200  # Hz
#FUSE_L  = 0xD7
#FUSE_H  = 0xD5
#FUSE_E  = 0x00



# MRBus
DEFINES = -DMRBUS
SRCS = $(BASE_NAME).c $(MRBUSLIB)/mrbus-avr.c $(MRBUSLIB)/mrbus-crc.c
INCS = $(MRBUSLIB)/mrbus.h $(MRBUSLIB)/mrbus-avr.h

# MRBee
#DEFINES = -DMRBEE -DMRBEE_BAUD=115200
#SRCS = $(BASE_NAME).c ../../mrbus/src/mrbus-crc.c ../../mrbee/src/mrbee.c ../../mrbee/src/xbee.c
#INCS = ../../mrbus/src/mrbus.h ../../mrbee/src/mrbee.h ../../mrbee/src/xbee.h

AVRDUDE = avrdude -c usbtiny -p $(DEVICE) 

OBJS = ${SRCS:.c=.o}
INCLUDES = -I. -I$(MRBUSLIB)
CFLAGS  = $(INCLUDES) -Wall -Os

COMPILE = avr-gcc $(DEFINES) -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)

# -E -dM options preprocess only and output results of #define
#COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE) -E -dM

help:
	@echo "make hex ....... build $(BASE_NAME).hex"
	@echo "make flash ..... flash the firmware"
	@echo "make fuse ...... flash the fuses"
	@echo "make program ... flash fuses and firmware"
	@echo "make read ...... read the fuses"
	@echo "make clean ..... delete objects and hex file"
	@echo "make release.... produce release tarball"

hex: $(BASE_NAME).hex

program: fuse flash

# rule for programming fuse bits:
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" -a "$(FUSE_E)" != "" ] || \
		{ echo "*** Invalid Fuse values."; exit 1; }
	$(AVRDUDE) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m -U efuse:w:$(FUSE_E):m

read:
	$(AVRDUDE) -v

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

release: hex
	@echo -n "Creating temporary build directories..."
	@$(eval BTMPDIR := $(shell mktemp -d))
	@$(eval TMPDIR := $(BTMPDIR)/$(BASE_NAME))
	@$(eval BOILERPLATE_FILES := $(shell find ../../docs/release-boilerplate -type f -name *.txt -print))
	@$(eval RELEASE_TIME := $(shell date +"%d%b%Y-%H%Mh"))
	@mkdir -p $(TMPDIR)/mrbus/src
	@mkdir -p $(TMPDIR)/$(BASE_NAME)/src
	@echo "  [done]"

	@echo -n "Copying boilerplate files..."
	@cp $(BOILERPLATE_FILES) $(TMPDIR)
	@echo "  [done]"

	@echo -n "Copying Makefile..."
	@cp Makefile $(TMPDIR)/$(BASE_NAME)/src/Makefile

	@echo -n "Copying object..."
	@cp $(BASE_NAME).hex $(TMPDIR)/$(BASE_NAME)/src/$(BASE_NAME).hex
	@echo "  [done]"

	@echo -n "Copying source files..."
	@tar cPf - $(INCS) | tar xPf - -C $(TMPDIR)/$(BASE_NAME)/src/
	@echo "  [done]"

	@echo -n "Copying include files..."
	@tar cPf - $(SRCS) | tar xPf - -C $(TMPDIR)/$(BASE_NAME)/src/
	@echo "  [done]"

	@echo -n "Writing file SVN statuses..."
	@echo "### Archive built at $(RELEASE_TIME)" > $(TMPDIR)/$(BASE_NAME)/src/FILE_SVN_VERSIONS
	@svn status -v $(BOILERPLATE_FILES) >> $(TMPDIR)/$(BASE_NAME)/src/FILE_SVN_VERSIONS
	@svn status -v Makefile >> $(TMPDIR)/$(BASE_NAME)/src/FILE_SVN_VERSIONS
	@svn status -v $(INCS) >> $(TMPDIR)/$(BASE_NAME)/src/FILE_SVN_VERSIONS
	@svn status -v $(SRCS) >> $(TMPDIR)/$(BASE_NAME)/src/FILE_SVN_VERSIONS
	@echo "  [done]"
	

	@echo -n "Creating tarball..."
	@tar zcf $(BASE_NAME)-$(RELEASE_TIME).tgz -C $(BTMPDIR) $(BASE_NAME)
	@echo "  [done]"

	@echo "Release in $(BASE_NAME)-$(RELEASE_TIME).tgz"
