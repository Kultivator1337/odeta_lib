#ifndef __ODETA_GPIO__
#define __ODETA_GPIO__

#include "odeta_lib.h"

#define _PA_OFFS 0
#define _PA_N_PINS 14
#define _PA_BREAK 13
#define _PA_SHIFT 2


#define _PB_OFFS (_PA_OFFS + _PA_N_PINS)
#define _PB_N_PINS 15
#define _PB_BREAK 11
#define _PB_SHIFT 1

#define _PC_OFFS (_PB_OFFS + _PB_N_PINS)
#define _PC_N_PINS 3
#define _PC_BREAK 0
#define _PC_SHIFT 13

#define __PRE(P) ((0x01UL<<_P##P##_BREAK)-1)
#define __POST(P) (((0x01UL<<_P##P##_N_PINS)-1)&(~__PRE(P)))

#define __P_SH(P, x) ((x)>=_P##P##_BREAK?_P##P##_SHIFT:0)

#define _P(P, x) (0x01UL<<((x)-__P_SH(P, x)+_P##P##_OFFS))

#define __PORT(P, x) ((((x)&__POST(P))<<_P##P##_SHIFT)|((x)&__PRE(P)))
#define _PORT(P, x) __PORT(P, (x>>_P##P##_OFFS))

#define _EXTI_IRQ_LINE(X) 


#define _PINS(P, x) ((x)<<_P##P##_OFFS)
#define __PORTID(x) ((x) == PORTA)?A:((x) == PORTB)?B:C
#define _PORTID(x) __PORTID(x)

#define PIN(x, y) _P(x, y)
#define PINS(x, y) _PINS(x, y)


typedef union{
    uint32_t pins;
    struct {
        uint32_t pa_pins : _PA_N_PINS;
        uint32_t pb_pins : _PB_N_PINS;
        uint32_t pc_pins : _PC_N_PINS;
    };
}GpioPins_t;

typedef enum {
    OUTPUT_PUSHPULL = 0x00,
    OUTPUT_NOPULL   = 0x00,
    OUTPUT_PULLUP   = 0x01<<0,
    OUTPUT_PULLDOWN = 0x01<<1,
    OUTPUT_OD       = 0x01<<2
}OutputMode_t;


typedef enum {
    INPUT_NOPULL   = 0x00,
    INPUT_PULLUP   = 0x01<<0,
    INPUT_PULLDOWN = 0x01<<1
} InputMode_t;

typedef enum {
    INT_RISING  = 0x01,
    INT_FALLING = 0x02
} GpioIntMode_t;

void gpioSetPinsAsOutput(uint32_t pins, OutputMode_t mode);
void gpioSetPinsAsInput(uint32_t pins, InputMode_t mode);

void gpioSetOutput(uint32_t pins, uint8_t state);
uint32_t gpioGetInput(uint32_t pins);
uint32_t gpioGetOutput(uint32_t pins);

typedef void (*GpioIntHandler_f)(uint32_t);

void exti_handler(uint32_t line);

void gpioEnableInterrupt(uint32_t pin, GpioIntMode_t mode);
void gpioRegisterInterruptHandler(uint32_t pin, GpioIntHandler_f handler);




#endif