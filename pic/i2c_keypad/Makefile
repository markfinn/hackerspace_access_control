

.PHONY: upload on

%.hex: %.asm
	gpasm -I.. $<

all: i2c_keypad.hex

upload: i2c_keypad.hex
	pk2cmd -M -Ppic16f54 -F$<

on: upload
	pk2cmd  -Ppic16f54 -R -T ON

off:
	pk2cmd  -Ppic16f54 -T OFF

