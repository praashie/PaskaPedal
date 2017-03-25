# Adapted from http://codeandlife.com/data/Makefile
# Options set up for ATTiny84


# WinAVR cross-compiler toolchain is used here
CC = avr-gcc
OBJCOPY = avr-objcopy
DUDE = avrdude
F_CPU = 16000000

TINY_CORE_PATH = $(HOME)/.arduino15/packages/ATTinyCore/hardware/avr/1.1.0/
TINY_VARIANT = tinyX4_reverse

# If you are not using ATtiny2313 and the USBtiny programmer, 
# update the lines below to match your configuration
CFLAGS = -DF_CPU=$(F_CPU) -Wall -Os -Iusbdrv -I$(TINY_CORE_PATH)/cores/tiny -I$(TINY_CORE_PATH)/variants/$(TINY_VARIANT) -mmcu=attiny84
OBJFLAGS = -j .text -j .data -O ihex
DUDEFLAGS = -v -e -C /opt/arduino-1.6.7/hardware/tools/avr/etc/avrdude.conf -c stk500v1 -b 19200 -p attiny84 -P /dev/ttyACM0

# Object files for the firmware (usbdrv/oddebug.o not strictly needed I think)
OBJECTS = usbdrv/usbdrv.o usbdrv/oddebug.o usbdrv/usbdrvasm.o main.o

# Command-line client
CMDLINE = usbtest.exe

# By default, build the firmware and command-line client, but do not flash
all: main.hex

# With this, you can flash the firmware by just typing "make flash" on command-line
flash: main.hex
	$(DUDE) $(DUDEFLAGS) -U flash:w:$<

# One-liner to compile the command-line client from usbtest.c
$(CMDLINE): usbtest.c
	gcc -I ./libusb/include -L ./libusb/lib/gcc -O -Wall usbtest.c -o usbtest.exe -lusb

# Housekeeping if you want it
clean:
	$(RM) *.o *.hex *.elf usbdrv/*.o

# From .elf file to .hex
%.hex: %.elf
	$(OBJCOPY) $(OBJFLAGS) $< $@

# Main.elf requires additional objects to the firmware, not just main.o
main.elf: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $@

# Without this dependance, .o files will not be recompiled if you change 
# the config! I spent a few hours debugging because of this...
$(OBJECTS): usbdrv/usbconfig.h

# From C source to .o object file
%.o: %.c	
	$(CC) $(CFLAGS) -c $< -o $@

# From assembler source to .o object file
%.o: %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@
# From assembler source to .o object file
%.o: %.asm
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@
