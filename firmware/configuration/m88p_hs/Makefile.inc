# Name: Makefile
# Project: Micronucleus
# License: GNU GPL v2 (see License.txt)

# Controller type: ATmega88p
# Configuration:   Uses 12 MHz V-USB implementation
# Last Change:     Aug 26, 2020

# Enable unsafe optimizions. This will disable several safety features in microncleus to save around 40 more bytes
# 
# Disabled features:
#   * Stack pointer and SREG initialization in CRT
#   * Client side reset vector patching
#   * USB collision detection. Micronucleus will not work reliability with hubs if this is disabled.

#CFLAGS += -DENABLE_UNSAFE_OPTIMIZATIONS

# handystick uses 12Mhz external crystal
F_CPU = 12000000
DEVICE = atmega88p

# hexadecimal address for bootloader section to begin. To calculate the best value:
# - make clean; make main.hex; ### output will list data: 2124 (or something like that)
# - for the size of your device (8kb = 1024 * 8 = 8192) subtract above value 2124... = 6068
# - How many pages in is that? 6068 / 64 (tiny85 page size in bytes) = 94.8125
# - round that down to 94 - our new bootloader address is 94 * 64 = 6016, in hex = 1780
BOOTLOADER_ADDRESS = 1800

# Note: the bootloader must reside in the space that is marked as bootloader flash space
# in the fuse bits. (The ATmega328 prevents flash programming from code that is not in the
# bootloader space.) But the application may also be in part of that space. 

#FUSEOPT = -U lfuse:w:0xdf:m -U hfuse:w:0xdd:m -U efuse:w:0xf8:m
#FUSEOPT from usbasploader
#FUSEOPT = -U lfuse:w:0xd7:m -U hfuse:w:0xd4:m -U efuse:w:0xf8:m
# mod: FS xtal, BOD 4.3->2.7V,   
FUSEOPT = -U lfuse:w:0xd7:m -U hfuse:w:0xd5:m -U efuse:w:0xf8:m
#FUSEOPT_DISABLERESET = -U lfuse:w:0xff:m -U hfuse:w:0x5d:m -U efuse:w:0xf9:m

#---------------------------------------------------------------------
# ATMega88p
#---------------------------------------------------------------------
# Fuse low byte:
# 0xef = 1 1 0 1   1 1 1 1
#        ^ ^ \+/   \--+--/
#        | |  |       +------- CKSEL 3..0 (clock -> LP xtal > 8MHz)
#        | |  +--------------- SUT 1..0 (Start up time -> fast BOD)
#        | +------------------ CKOUT (clock output on CKOUT pin -> disabled)
#        +-------------------- CKDIV8 (divide clock by 8 -> don't divide)
#
# Fuse high byte:
# 0xdd = 1 1 0 1   1 1 0 1
#        ^ ^ ^ ^   ^ \-+-/ 
#        | | | |   |   +------ BODLEVEL 2..0 (2.7 V)
#        | | | |   +---------- EESAVE (preserve EEPROM on Chip Erase -> not preserved)
#        | | | +-------------- WDTON (watchdog timer always on -> disable)
#        | | +---------------- SPIEN (enable serial programming -> enabled)
#        | +------------------ DWEN (debug wire enable)
#        +-------------------- RSTDISBL (disable external reset -> reset is enabled)
#
# Fuse extended byte:
# 0xf8 = - - - -   - 0 0 0
#        ^ ^ ^ ^   ^ \-/ ^
#        \---------/  |  +---- BOOTRST (reset vector -> bootloader)
#             |       +------- BOOTSZ 1..0 (Bootloader flash size -> 2048 Bytes)
#             +--------------- Unused
###############################################################################
