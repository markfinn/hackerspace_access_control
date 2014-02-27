mrbus_bootloader
==========================

Provides a fast way to bootload code to AVRs over [MRBus](http://mrbus.org).

Features:
---------
* Supports an AES-CBC-MAC (length prepended) for signing code so that the bootloader won't run arbitrary code from an open bus.
* AES and CBC-MAC routines are available to user code through service vectors.
* Bootloader and host end collaborate to allow minimal data transfer and minimal erasing when possible.  This makes many repeated code-burn cycle very fast
* Bus node management options from the host end, such as reboot-to-bootloader and node-find



Installing:
--------
###To use the host end script
Get these python packages:
* intelhex
* pycrypto
* mrbus (Not a package yet, curently lives in this repo)


    sudo pip install intelhex
    sudo pip install pycrypto

###To use the bootloder
Build from source of use the included bootloader.hex for ATmega328p.  To build, you will need the MRBus libs.

Other parts will need some changes, feel free to submit those changes.

When building from source, older GCCs are not as efficient, and will not make a bootloder that is small enough.  Currently there are 154 bytes (out of 4096 available) free in the bootloder when built with gcc-avr 4.7.2-2 and avr-libc 1.8.0-3 on ubunutu 13.4, but gcc-avr 4.5.3-2 and avr-libc 1.7.1-2 on Linux Mint 12 is out of space by a lot. 

Third party code used
----------------------
* AES code from [AVR-Crypto-Lib](http://www.das-labor.org/wiki/AVR-Crypto-Lib) from svn co http://das-labor.org/svn/microcontroller-2/crypto-lib, rev 5906.  GPL
* MRBUS libs (version 2) from mrbus.org, svn rev 977

todo: 
-----
* use load plan to speed up rewriting more
* settle fuses
* make code / Makefile easier to set up for other parts
* make signing less hard-coded in host script
* update this doc and the doc in the code
* make mrbus.py a package and submit to pypi

Using:
-----
I hit the bus pretty hard through the MRB-CI2 computer interface.  You will need the latest version of firmware (Feb 25th 2014) to get flow control or you will have problems.

###Examples
    ###
    #load a file with the node already in bootloader mode, don't know it's address, nothing else in bootloader mode on bus.

    python load.py ../door_terminal2/doorterm.hex


    ###
    #use /dev/ttyUSB0 as MRB-CI2 interface port
    #use 0xfe as host address
    #load to node 0x5c
    #reboot node, then wait for it to enter the bootloader and grab it there
    #reboot node to application when done loading
    #use caching system to keep track of files loaded and use them to speed up burning in code-and-load development loop

    python load.py -p /dev/ttyUSB0 -a 0xfe -d 0x5c ../door_terminal2/doorterm.hex -x -r -c -s


    ###
    #get help

    python load.py -h



Signing / crypto notes
----------------------
1. If you don't want it, leave the key as the default, it functions as a big CRC.
1. A bad signature will keep the part in the bootloader instead of running the application
2. nothing stops a rouge on you network from overwriting your application or corrupting it. They can't get it to run though without your key
1. once fuses are set right, your code will be somewhat read-safe. (The AVR is not very protected according to "the net", but the bootloader won't make it worse
1. you are not protected from someone with an ICSP programmer, but if you use a key and the fuses, you should be safe from an unsigned application and all reads from the bus.
1. the only thing I know of that comes close to a read form the bus is the ability (through an attack) to tell is an arbitrary byte in the application is 0xff or not.  no other reading should be possible.  I plan to store keys there.



License 
-------
Copyright Mark A Finn, 2014.

GPL version 3 or later.




