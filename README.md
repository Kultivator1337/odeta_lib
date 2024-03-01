# Introduction
Odeta-lib is a library for the Blackpill board aimed at beginners wanting to learn embedded development. This readme file will contain a theoretical background and example projects for each peripheral.

## Blackpill board
The Blackpill is a low-cost minimal development board for the STM32F411 microcontroller. The board features an USB-C Connector for communication and programming, a programmable LED and a pushbutton, with other pins of the STM32F411 accessible via breadboard-compatible 2.54mm pin headers.

![Blackpill pinout diagram](blackpill.png)

## Required software
I recommend using the [Visual Studio Code](https://code.visualstudio.com/docs/setup/linux) editor for development, although any text editor can be used.  
Compiling code for the STM32 microcontroller, which uses a 32-bit ARM core, an [ARM cross-compiler toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) is required.  
Finally, flashing the board over USB requires the dfu-util package, which can be installed by running the command `sudo apt install dfu-util`.

## Building and flashing
First, clone this repository and enter the resulting folder. The code can then be compiled using the `make` command.  
To flash the microcontroller, it must first enter DFU mode, which can be accomplished by pressing both the BOOT and NRST buttons, releasing the NRST button and then releasing the BOOT button. Finally, the microcontroller can be flashed using the `make flash` command.
If flashing was successful, the on-board blue LED should start flashing.
```
$ git clone https://github.com/Kultivator1337/odeta_lib.git
$ cd odeta_lib
$ make
$ make flash
```

## Development
User code should be written in the `app.c` and `app.h` files. The `app.c` file contains the function ` void app()`, which is called after the board is initialized, and this is where user code should start.
```c
    void app(){
        // Initialize used peripherals, variables ...
        for(;;){
            // Main loop which runs forever
            // Program logic should be placed here
        }
    }
```
All of the examples in this document only require modifying the `app.c` file, unless explicitly stated otherwise.

# Part 1: GPIO
GPIO stands for _"General Purpose Input/Output"_ and it is one of the most basic peripherals which enable the microcontroller to interact with the outside world.
GPIO pins of a microcontroller can be set as either Output or Input.
Output pins allow you to set the voltage of that pin to a high or low level, and Input pins allow you to measure whether the voltage connected to that pin is high or low. For the Blackpill board, the high voltage level is 3,3V and the low voltage level is 0V.

The GPIO peripheral of the STM32 looks like this:

![STM32 GPIO](stm32_gpio.png)

