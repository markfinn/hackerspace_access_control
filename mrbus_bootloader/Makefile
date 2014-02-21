#*************************************************************************
#Title:    MRB-bootloader Makefile
#Authors:  Mark Finn <mark@mfinn.net>
#License:  GNU General Public License v3
#
#    Derived from MRB-AVRTEMPLATE

BASE_NAME = bootloader
MRBUSLIB = ../mrbus2/src

# ATmega328P 20MHz @ 5.0V
DEVICE  = atmega328p
F_CPU   = 20000000  # Hz
FUSE_L  = 0xD7
#CKDIV8=1:
#CKOUT=1
#SUT1,0=0,1
#CKSEL3-0=7  full swing crystal, BOD enable, 16k clk delay, 14k clk +4.1ms reset delay
FUSE_H  = 0xD0
#RSTDISBL=1: reset enabled
#DWEN=1: debug wire disabled
#SPIEN=0: spi programming enabled
#WDTON=1: Watchdog under software control
#EESAVE=0: eeprom not preserved through chip erase
#BOOTSZ1,0=0,0: #boot area is 2048 words, 32 pages of 64 words and 128 bytes.  application flash from 0x0000 - 0x37FF, boot flash from 0x3800 - 0x3FFF
#BOOTRST=0: #boot into the boot loader
FUSE_E  = 0x04
# BODLEVEL=4: 4.1-4.3-4.5 v brown out
LOCK=0x0F
#BLB12, 11 -> 0,0  #bootloader can't write to itself, application can't read bootloader
#BLB02, 01 -> 1,1  #bootloader can read and write application flash
#LB2,1 -> 1,1 unlocked. Maybe lock these later once I know what I'm doing, but do it as a second command after fuses and BLBs are set

BOOTSTART = 0x7000#0x3800#in words


# MRBus
DEFINES = -DMRBUS -DF_CPU=$(F_CPU) -DBOOTSTART=$(BOOTSTART)
SRCS = $(BASE_NAME).c $(MRBUSLIB)/mrbus-avr.c $(MRBUSLIB)/mrbus-crc.c $(MRBUSLIB)/mrbus-queue.c
ASRCS = aes_enc-asm.S  aes_keyschedule-asm.S  aes_sbox-asm.S blvects.S
INCS = $(MRBUSLIB)/mrbus.h $(MRBUSLIB)/mrbus-avr.h aes128_enc.h  aes_keyschedule.h  aes_types.h

AVRDUDE = avrdude -c usbtiny -p $(DEVICE) 

OBJS = ${SRCS:.c=.o} ${ASRCS:.S=.o}
INCLUDES = -I. -I$(MRBUSLIB)
CFLAGS  = $(INCLUDES) -Wall -Os -std=gnu99 -ffunction-sections -fdata-sections
LDFLAGS += -Wl,-gc-sections,--section-start=.text=$(BOOTSTART),-Map,$(BASE_NAME).map,--gc-sections,--relax,-Tbootloader.x

COMPILE = avr-gcc $(DEFINES) $(CFLAGS) -mmcu=$(DEVICE)

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
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" -a "$(FUSE_E)" != "" -a "$(LOCK)" != "" ] || \
		{ echo "*** Invalid Fuse values."; exit 1; }
	$(AVRDUDE) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m -U efuse:w:$(FUSE_E):m -U lock:w:$(LOCK):m

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

$(BASE_NAME).elf: $(OBJS) bootloader.x
	$(COMPILE) -o $(BASE_NAME).elf $(OBJS) $(LDFLAGS)



$(BASE_NAME).hex: $(BASE_NAME).elf
	rm -f $(BASE_NAME).hex $(BASE_NAME).eep.hex
	avr-objcopy -j .text -j .data -j .pageload -j .sbox -O ihex $(BASE_NAME).elf $(BASE_NAME).hex
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
