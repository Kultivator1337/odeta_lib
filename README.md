# Introduction
Odeta-lib is a library for the Blackpill board aimed at beginners wanting to learn embedded development. This readme file will contain a theoretical background and example projects for each peripheral.

## Blackpill board
The Blackpill is a low-cost minimal development board for the STM32F411 microcontroller. The board features an USB-C Connector for communication and programming, a programmable LED and a pushbutton, with other pins of the STM32F411 accessible via breadboard-compatible 2.54mm pin headers.

![Blackpill pinout diagram](/.doc_img/blackpill_pinout.jpg)

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

## GPIO configuration
![STM32 GPIO](/.doc_img/gpio_block_diag.png)

In hardware, the GPIO pins of the STM32 are implemented as shown in the block diagram above, allowing for various configurations depending on the needs of a specific project.

### Pull-Up and Pull-Down
The Pull-Up and Pull-Down resistors of each GPIO pin can be enabled to connect the pin to **Vdd** or **GND** internally through a ~40kOhm resistor.  
If neither of them is enabled, a GPIO pin that is not connected to **Vdd** or **GND** externally will be in a high-impedance (_"Floating"_, _"Hi-Z"_) state, and its level will be undefined.  
A GPIO pin cannot have both the Pull-Up and Pull-Down resistors enabled at the same time.

### Push-Pull and Open-Drain
An output pin can be either in Push-Pull or Open-Drain mode, changing how the pin behaves when set to logic high.  
In Push-Pull mode, the pin is connected to **Vdd** through the P-MOS transistor shown in the above block diagram.  
In Open-Drain mode, the P-MOS transistor is disabled, leaving the pin _"floating"_ when set to logic high.
In either case, an output pin set to logic low will be connected to **GND** through the N-MOS transistor.

## Debouncing
When a mechanical switch such as a push-button changes state, the vibrations can cause the voltage across it to rapidly change, making reading their state at that moment very unreliable. For example, if we were trying to increment a value every time a button is pressed, a single button press might increment the value multiple times. 
![Debouncing](/.doc_img/debounce.jpg)

To solve this issue, we can "debounce" such inputs by continuously reading the value until a certain number of reads result in the same value, indicating that the voltage across the switch has stabilized.

## Interrupts
Interrupts are a way for the microcontroller to service events which happen independently from from the currently executing instructions, eg. a button being pressed. As their name suggests they interrupt normal program flow, perform certain tasks, and finally return to where the program was interrupted.


![Interrupt program flow](/.doc_img/interrupt_flow_diagram.jpg)

The code executed when an interrupt occurs (**ISR** - _Interrupt service routine_) should be kept short to make their impact on normal program flow minimal.

On the STM32, GPIO interrupts can be triggered on falling and/or rising edges of input pins.

## API reference

### Defines
```c
#define LED PIN(C, 13)
#define KEY PIN(A, 0)
```

### Macros
```c
// Get the uint32_t value corresponding to pin y of port x
// x can be A, B or C
PIN(x, y)

// Get the uint32_t value corresponding to the pins signified by the bits of y of port x
// Ex. PINS(A, 0b0101) == PINS(A, 5) == PIN(A, 0) | PIN(A, 2)
PINS(x, y)
```


### Functions
```c
void gpioSetPinsAsOutput(uint32_t pins, OutputMode_t mode);
```
+ Set pins as output
+ Mode can be a combination of OUTPUT_OPENDRAIN, OUTPUT_PUSHPULL, OUTPUT_PULLUP, OUTPUT_PULLDOWN and OUTPUT_NOPULL
+ Ex. `gpioSetPinsAsOutput(LED|_PIN(B, 8), OUTPUT_PUSHPULL|OUTPUT_NOPULL);`

```c
void gpioSetPinsAsInput(uint32_t pins, InputMode_t mode);
```
+ Set pins as input
+ Mode can be INPUT_PULLUP, INPUT_PULLDOWN and INPUT_NOPULL
+ Ex. `gpioSetPinsAsInput(KEY|_PINS(B,5), INPUT_PULLUP);`

```c
void gpioSetOutput(uint32_t pins, uint32_t state);
```
+ Set the level of output pins
+ Pins will be set to high level if the corresponding bit in state is 1, and low if it is 0
+ Ex. `gpioSetOutput(LED|_PIN(B, 8), LED)` -> Sets LED pin high and B8 low
+ To set all pins high or low, state can be set to HIGH or LOW

```c
uint32_t gpioGetInput(uint32_t pins);
```
+ Get the level of input pins
+ The return value will have the bits at positions corresponding to the "pins" parameter set to 1 if the voltage at that pin is high, and 0 otherwise

```c
int32_t gpioGetInputDebounced(uint32_t pins, uint32_t debounce_cnt);
```
+ Get the level of input pins after deboubncing for "debounce_cnt" miliseconds

```c
uint32_t gpioGetOutput(uint32_t pins);
```
+ Get the level of output pins

```c
void gpioRegisterInterruptHandler(uint32_t pin, GpioIntHandler_f handler);
```
+ Register a function that will be called when an interrupt caused by "pin" occurs
+ The handler function should be of type `void` and accept one `uint32_t` parameter, signifying the pin which caused the interrupt
+ Ex. `void keyPressedHandler (uint32_t pin);`

```c
void gpioEnableInterrupt(uint32_t pin, GpioIntMode_t mode, uint32_t debounce);
```
+ Enable an interrupt caused by the change of state on "pin"
+ Mode can be INT_FALLING and/or INT_RISING
+ The registered interrupt handler function will be called after debouncing the pin for "debounce" miliseconds



## Examples

### 1: Turn LED on while button is pressed
In this example, we will modify `app.c` to turn on the LED only while the KEY button is pressed. As we are continuously checking the KEY pin level, there is no need for debouncing.
```c
void app(){
    // Set LED pin as push-pull output with no pull-up or pull-down resistors
    gpioSetPinsAsOutput(LED, OUTPUT_PUSHPULL|OUTPUT_NOPULL);
    // Set KEY pin as input with a pull-up resistor, as pressing the key connects the pin to GND
    gpioSetPinsAsInput(KEY, INPUT_PULLUP);

    while (1)
    {
        // If KEY is high (button is not pressed), set LED pin high, otherwise set it low
        gpioSetOutput(LED, gpioGetInput(KEY)?HIGH:LOW);
    }
}
```

### 2: Interrupts: Toggle LED when button is pressed
In this example, we will toggle the LED pin every time the button is pressed. Since the button should be toggled once for every time the button is pressed, we should debounce the input. Otherwise, a single key press could toggle the LED pin multiple times, leaving it in the same state it was in before the button press.
```c
// Define an interrupt handler function that toggles the LED pin
void keyHandler(uint32_t l){
    gpioSetOutput(LED, ~gpioGetOutput(LED));
}

void app(){
    gpioSetPinsAsOutput(LED, OUTPUT_PUSHPULL|OUTPUT_NOPULL);
    gpioSetPinsAsInput(KEY, INPUT_PULLUP);

    // Register the keyHandler function to be called whenever an interrupt is caused by the KEY pin
    gpioRegisterInterruptHandler(KEY, keyHandler);

    // Enable the KEY pin interrupt on falling edge (when button is pressed down), debounce for 100 ms
    gpioEnableInterrupt(KEY, INT_FALLING, 100);

    while (1)
    {
        // As all the program logic happens in the interrupt handler, the loop can be left empty
    }
}
```

### 3: Coming soon


# Part 2: Timers coming soon!