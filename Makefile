# Makefile for bare-metal STM32F103 driving I2C OLED display

# We'll pick up the GCC toolchain from the Arduino installation
ifeq ($(OS),Windows_NT)
ARDUINO=/Users/John.Honniball/AppData/Local/Arduino15
else
ARDUINO=/home/john/.arduino15
endif

# We need CMSIS from the ST repo STM32CubeF1, which is installed locally
CMSISDIR=../STM32CubeF1/Drivers/CMSIS

# ST-LINK is the Open Source tool used to program the STM32
ifeq ($(OS),Windows_NT)
STLINK_TOOLS=/Users/John.Honniball/Documents/bin/stlink-1.7.0-x86_64-w64-mingw32/bin
else
STLINK_TOOLS=/home/john/Arduino/hardware/Arduino_STM32/tools/linux/stlink
endif

MCU=cortex-m3
STM32MCU=STM32F103xB

ARM_TOOLS=$(ARDUINO)/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin

CC=$(ARM_TOOLS)/arm-none-eabi-gcc
LD=$(ARM_TOOLS)/arm-none-eabi-gcc
OC=$(ARM_TOOLS)/arm-none-eabi-objcopy
SZ=$(ARM_TOOLS)/arm-none-eabi-size

STINFO=$(STLINK_TOOLS)/st-info
STFLASH=$(STLINK_TOOLS)/st-flash

LDSCRIPT=$(CMSISDIR)/Device/ST/STM32F1xx/Source/Templates/gcc/linker/STM32F103XB_FLASH.ld
STARTUP=$(CMSISDIR)/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s
SYSTEM=$(CMSISDIR)/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c

CFLAGS=-Wall -mthumb -c -o $@ -Os -D$(STM32MCU) -I$(CMSISDIR)/Device/ST/STM32F1xx/Include -I$(CMSISDIR)/Include
LDFLAGS=-mthumb --specs=nosys.specs -o $@ -T$(LDSCRIPT)
OCFLAGS=-R .stack -O binary
SZFLAGS=-B -d
INFOFLAGS=--descr

OBJS=i2c_oled.o
ELFS=$(OBJS:.o=.elf)
BINS=$(OBJS:.o=.bin)

# Default target will compile and link all C sources, but not program anything
all: $(BINS)
.PHONY: all

i2c_oled.bin: i2c_oled.elf
	$(OC) $(OCFLAGS) i2c_oled.elf i2c_oled.bin

i2c_oled.elf: i2c_oled.o startup_stm32f103xb.o system_stm32f1xx.o
	$(LD) -mcpu=$(MCU) $(LDFLAGS) startup_stm32f103xb.o system_stm32f1xx.o i2c_oled.o
	$(SZ) $(SZFLAGS) i2c_oled.elf
	
i2c_oled.o: i2c_oled.c image.h
	$(CC) -mcpu=$(MCU) $(CFLAGS) i2c_oled.c

system_stm32f1xx.o: $(SYSTEM)
	$(CC) -mcpu=$(MCU) $(CFLAGS) $(SYSTEM)

startup_stm32f103xb.o: $(STARTUP)
	$(CC) -mcpu=$(MCU) $(CFLAGS) $(STARTUP)

image.h: image.pbm pbm2oled
	./pbm2oled image.pbm OLEDImage >image.h

pbm2oled: pbm2oled.c
	gcc -o pbm2oled pbm2oled.c

# Target to invoke the programmer and program the flash memory of the MCU
prog: i2c_oled.bin
	$(STFLASH) write i2c_oled.bin 0x8000000

.PHONY: prog

# Target 'teststlink' will connect to the programmer and read the
# device ID, but not program it
teststlink:
	$(STINFO) $(INFOFLAGS)

.PHONY: teststlink

# Target 'clean' will delete all object files, ELF files, and BIN files
clean:
	-rm -f $(OBJS) $(ELFS) $(BINS) startup_stm32f103xb.o system_stm32f1xx.o pbm2oled

.PHONY: clean


