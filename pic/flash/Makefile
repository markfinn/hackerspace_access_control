

.PHONY: upload on

%.hex: %.asm
	gpasm -I.. $<

all: flash.hex

upload: flash.hex
	pk2cmd -M -Ppic16f54 -F$<

on: upload
	pk2cmd  -Ppic16f54 -R -T ON

off:
	pk2cmd  -Ppic16f54 -T OFF

