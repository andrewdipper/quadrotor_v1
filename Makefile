# The name of your project (used to name the compiled .hex file)
TARGET = main

# configurable options
OPTIONS = -DF_CPU=96000000 -Wall -DUSB_SERIAL -DLAYOUT_US_ENGLISH -D__MK20DX256__ -U__MK20DX128 -DSQUAD2S -DPLUSMODE

VPATH = $(SRCPATH) $(LINKFILEPATH)

# path location for Teensy Loader, teensy_post_compile and teensy_reboot
TOOLSPATH = ../../../tools   # on Linux

# path location for the arm-none-eabi compiler
COMPILERPATH = ../../../tools/arm-none-eabi/bin

LINKFILEPATH = link

#path to hardware source files
SRCPATH = hw

QUADPATH = quadcore

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -g -Os -mcpu=cortex-m4 -mthumb -nostdlib -MMD $(OPTIONS) -I.

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS =

SFLAGS = -S

# linker options
LDFLAGS = -Os -Wl,--gc-sections -mcpu=cortex-m4 -mthumb -T $(LINKFILEPATH)/mk20dx256.ld

# additional libraries to link
LIBS = -lm


# names for the compiler programs
CC = $(abspath $(COMPILERPATH))/arm-none-eabi-gcc
CXX = $(abspath $(COMPILERPATH))/arm-none-eabi-g++
OBJCOPY = $(abspath $(COMPILERPATH))/arm-none-eabi-objcopy
SIZE = $(abspath $(COMPILERPATH))/arm-none-eabi-size

# automatically create lists of the sources and objects
C_FILES := $(wildcard *.c) $(wildcard $(SRCPATH)/*.c) $(wildcard $(QUADPATH)/*.c) 
#CPP_FILES := $(wildcard *.cpp) $(wildcard $(SRCPATH)/*.cpp)
OBJS := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) 
ASM_FILES := $(C_FILES:.c=.s)


# the actual makefile rules (all .o files built by GNU make's default implicit rules)

all: $(TARGET).hex


$(TARGET).elf: $(OBJS) $(LINKFILEPATH)/mk20dx256.ld
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@
	$(abspath $(TOOLSPATH))/teensy_post_compile -file=$(basename $@) -path=$(shell pwd) -tools=$(abspath $(TOOLSPATH))
	-$(abspath $(TOOLSPATH))/teensy_reboot


# compiler generated dependency info
-include $(OBJS:.o=.d)

%.s: %.c
	$(CC) -S $(CPPFLAGS) $<

asm: $(ASM_FILES)

asm_clean: 
	rm -f *.s

clean:
	rm -f *.o *.d $(TARGET).elf $(TARGET).hex
	rm -f $(SRCPATH)/*.o $(SRCPATH)/*.d
	rm -f $(QUADPATH)/*.o $(QUADPATH)/*.d

nuke: clean
	rm  -f *~ 
	rm  -f $(SRCPATH)/*~ 	
	rm  -f $(QUADPATH)/*~ 	


