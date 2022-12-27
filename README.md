# STM32_I2C_OLED #

An STM32F103 "Blue Pill" program to display stuff on a 128x32 pixel
OLED display connected to the I2C interface.
There's a 9600 baud serial interface on USART1 (pins PA9 and PA10) which
is used to control the display.
The command '0' will clear it, while 'i' will show a pre-generated
image.

The program is in C and may be compiled with GCC on Linux
(Windows may also work if you have a copy of GNU 'make' installed).

## Chips Supported ##

At present, there's only support for the STM32F103 on the "Blue Pill" development board.

## ARM Toolchain ##

A recent version of GCC for ARM, such as that installed by the Arduino "stm32duino" toolchain.

You'll also need some files from the ST "STM32CubeF1" repo on GitHub, such as the linker script.

