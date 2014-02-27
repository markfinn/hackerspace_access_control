avr crypto from svn co http://das-labor.org/svn/microcontroller-2/crypto-lib  5906
sudo pip install intelhex
sudo pip install pycrypto


todo: 
use load plan to speed up
fuses


cmds:

python load.py -p /dev/ttyUSB0 -a 0xfe -d 0x5c ../door_terminal2/doorterm.hex -x -c -s
avrdude -c usbtiny -p atmega328p -U flash:r:readback.hex:i
scp bootloader.c
laptop:hackerspace_access_control/mrbus_bootloader/bootloader.c && ssh laptop "cd hackerspace_access_control/mrbus_bootloader; make hex" && scp laptop:hackerspace_access_control/mrbus_bootloader/bootloader.hex . && make -t hex


good, small:
ubunutu 13.4:
avr-libc package version: 1.8.0-3
gcc-avr package version: 4.7.2-2


bad:
lm12:
avr-libc package version: 1.7.1-2
gcc-avr package version: 4.5.3-2

