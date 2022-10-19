# Hey Emacs, this is a -*- makefile -*-

# This varible defines which program will be compiled and downloaded to iRobot
TARGET = shape

# Makefile, version 0.2
#
# iRobot requires a much more complex compile command than your typical
# program, so we put all the instructions into this makefile instead.
# 
#----------------------------------------------------------------------------
# iRobot compile instructions written by kwalsh@cs.holycross.edu, based on
# WinAVR Makefile Template written by Eric B. Weddington, Jörg Wunsch, et al.
#
# Released to the Public Domain
#
# Additional material for this makefile was written by:
# Peter Fleury
# Tim Henigan
# Colin O'Flynn
# Reiner Patommel
# Markus Pfaff
# Sander Pool
# Frederik Rouleau
#
#----------------------------------------------------------------------------
#
# On command line:
#
# make
# make all
#   Compile the program listed in the TARGET variable, below.
#
# make download
#   Download the compiled hex file to the device using avrdude,
#   and using the avrdude settings below.
#
# make clean
#   Delete all compiled and temporary files.
#
# To rebuild project do "make clean" then "make all".
#
#----------------------------------------------------------------------------

# ---------------- iRobot Settings ----------------

# iRobot has an Atmel ATMega 168 processor, running at 18.432 MHz
MCU = atmega168
F_CPU = 18432000

# The downloader expects programs to be in ihex file format.
EXE_FORMAT = ihex

# Compiler optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. 1-3 = optimize for speed. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s

# The TARGET source code is compiled along with iRobot.c to create the program.
EXTRA_FILES = iRobot.c

# There are two header files that are used.
EXTRA_HEADERS = iRobotCommands.h iRobot.h

# ---------------- Compiler Settings ----------------

# Compile for Atmel processor.
CFLAGS = -mmcu=$(MCU)
# Search in local directory for include files.
CFLAGS += -I.
# Turn on warnings.
CFLAGS += -Werror -Wall # -Wstrict-prototypes
# Configure processor frequency.
CFLAGS += -DF_CPU=$(F_CPU)UL
# Configure optimization level.
CFLAGS += -O$(OPT)
# Configure miscelaneous C/C++ options.
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
# Strip dead code.
CFLAGS += -fdata-sections -ffunction-sections -Wl,--gc-sections

# Link with standard math library.
LIBRARIES = -lm


# ---------------- Downloading Settings (avrdude) ----------------

# The iRobot contains an stk500v1 serial port downloader chip.
AVRDUDE_PROGRAMMER = stk500v1

# Uncomment the correct USB serial port to connect to your iRobot.
# Typical Linux Serial Port
AVRDUDE_PORT = /dev/ttyUSB0
# Typical Mac OSX Serial Port - /dev/tty.usbserial***
# AVRDUDE_PORT = /dev/tty.usbserial-A3000DZh
# Typical Windows Serial Port
# AVRDUDE_PORT = COM9 

# Set avrdude to download the program to the iRobot flash memory.
AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

# Combine all of the avrdude flags into one variable.
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)


#---------------- Compiler Rules ----------------

# Define programs and commands.
CC = avr-g++
OBJCOPY = avr-objcopy
OBJSIZE = avr-size
AVRDUDE = avrdude
REMOVE = rm -f

# Default is to make all.
all: $(TARGET).hex

always:

# Compile and link: create compiled ELF output file from source files.
%.hex: %.c $(EXTRA_FILES) always
	@echo
	@echo "-------- begin --------"
	@echo
	@echo "Compiling program: " $*.elf
	$(CC) $(CFLAGS) $*.c $(EXTRA_FILES) -o $*.elf $(LIBRARIES)
	# Create final output file (.hex) from ELF output file.
	@echo
	@echo "Creating flash memory file for download: " $*.hex
	$(OBJCOPY) -O $(EXE_FORMAT) -R .eeprom $*.elf $*.hex
	# End with a nice message and some statistics.
	@if test -f $*.elf; then \
	  echo; \
	  echo "Size of compiled iRobot program:"; \
	  $(OBJSIZE) -A $*.elf; \
	  textsize=`$(OBJSIZE) -A $*.elf | grep "^.text" | ( read t s a; echo $$s; )`; \
	  if test $$textsize -gt 7168; then \
	    echo "ERROR: Program is too large to fit in iRobot flash memory."; \
	    echo "ERROR: The Command Module has only 7168 bytes of memory,"; \
	    echo "ERROR: but $*.elf contains $$textsize bytes of code."; \
	    $(REMOVE) $*.elf; \
	    exit 1; \
	  fi; \
	fi
	@echo "--------  end  --------"


# Program the device using local USB connection. No recompilation.
download_local:
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)

# Program using local downloader. Recompiles if needed.
download: $(TARGET).hex
	~csci131/bin/downloadRobot

# Program using remote downloader. Recompiles if needed.
download_remote: $(TARGET).hex
	~csci131/bin/remoteDownloadRobot

# Clean up the project.
clean: 
	@echo
	@echo "Removing compiled and temporary files:"
	$(REMOVE) *.hex *.eep *.cof *.elf *.map *.sym *.lss *.o *.lst *.s *.d .dep/*
	@echo


.PHONY: all begin end clean program download always
