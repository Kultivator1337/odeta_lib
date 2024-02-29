#ifndef __ODETA_GPIO__
#define __ODETA_GPIO__

#include "odeta_lib.h"

#define _PA_N_PINS 14
#define _PB_N_PINS 15
#define _PC_N_PINS 3

#define _PA_SH(x) (x>12?2:0)
#define _PB_SH(x) (x>10?1:0)
#define _PC_SH(x) (13)

#define _PA(x) (0x01UL<<(x - _PA_SH(x)))
#define _PB(x) (0x01UL<<(x - _PB_SH(x) + _PA_N_PINS))
#define _PC(x) (0x01UL<<(x - _PC_SH(x) + _PA_N_PINS + _PB_N_PINS))

#define PIN(x, y) _P##x(y)

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

void setPinsAsOutput(uint32_t pins, OutputMode_t mode);
void setPinsAsInput(uint32_t pins, InputMode_t mode);

void setOutput(uint32_t pins, uint8_t state);
uint32_t getInput(uint32_t pins);
uint32_t getOutput(uint32_t pins);



#endif