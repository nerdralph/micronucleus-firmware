# Name: Makefile
# Project: Micronucleus
# License: GNU GPL v2 (see License.txt)

# Controller type: ATtiny 88 - 12 Mhz internal RC oscillator
# Configuration:   Default  + LED
# Last Change:     9/2020

#CFLAGS += -DENABLE_SAFE_OPTIMIZATIONS  # This saves 10 bytes and disables reliable jump to 0x0000, which one should avoid anyway.

F_CPU = 12000000
DEVICE = attiny88

# hexadecimal address for bootloader section to begin. To calculate the best value:
# - make clean; make main.hex; ### output will list data: 1452
# - for the size of your device (8kb = 1024 * 8 = 8192) subtract above value = 6740
# - How many pages in is that? 6740 / 64 (tiny88 page size in bytes) = 105.xxx
# - round that down to 105 - our new bootloader address is 105 * 64 = 6720, in hex = 1A40
# - For data size up to 1408 the address is 1A80 for size from 1409 to 1471 the address is 1A40 from 1472 1A00
BOOTLOADER_ADDRESS = 1A80

FUSEOPT = -U lfuse:w:0xC2:m -U hfuse:w:0xDD:m -U efuse:w:0xFE:m
FUSEOPT_DISABLERESET = -U lfuse:w:0xe1:m -U efuse:w:0xfe:m -U hfuse:w:0x5d:m

#---------------------------------------------------------------------
# ATtiny85
#---------------------------------------------------------------------
# Fuse extended byte:
# 0xFE = - - - -   - 1 1 0
#                        ^
#                        |
#                        +---- SELFPRGEN (enable self programming flash)
#
# Fuse high byte:
# 0xdd = 1 1 0 1   1 1 0 1
#        ^ ^ ^ ^   ^ \-+-/
#        | | | |   |   +------ BODLEVEL 2..0 (brownout trigger level -> 2.7V)
#        | | | |   +---------- EESAVE (preserve EEPROM on Chip Erase -> not preserved)
#        | | | +-------------- WDTON (watchdog timer always on -> disable)
#        | | +---------------- SPIEN (enable serial programming -> enabled)
#        | +------------------ DWEN (debug wire enable)
#        +-------------------- RSTDISBL (disable external reset -> enabled)
#
# Fuse high byte ("no reset": external reset disabled, can't program through SPI anymore)
# 0x5d = 0 1 0 1   1 1 0 1
#        ^ ^ ^ ^   ^ \-+-/
#        | | | |   |   +------ BODLEVEL 2..0 (brownout trigger level -> 2.7V)
#        | | | |   +---------- EESAVE (preserve EEPROM on Chip Erase -> not preserved)
#        | | | +-------------- WDTON (watchdog timer always on -> disable)
#        | | +---------------- SPIEN (enable serial programming -> enabled)
#        | +------------------ DWEN (debug wire enable)
#        +-------------------- RSTDISBL (disable external reset -> disabled!)
#
# Fuse low byte:
# 0xC2 = 1 1 0 0   - - 1 0
#        ^ ^ \+/   \--+--/
#        | |  |       +------- CKSEL 1..0 (clock selection -> RC OSC)
#        | |  +--------------- SUT 1..0 (BOD enabled) 6CK/14CK
#        | +------------------ CKOUT (clock output on CKOUT pin -> disabled)
#        +-------------------- CKDIV8 (divide clock by 8 -> don't divide)

###############################################################################
