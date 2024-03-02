#include "odeta_gpio.h"

static GpioIntCB_t gpio_handlers[16] = {0};
static uint32_t active_debounce_lines = 0;

uint32_t _gpioGetDebounceLines(){
    return active_debounce_lines;
}

GpioIntCB_t *_gpioGetIntCallback(uint32_t line){
    return &gpio_handlers[line];
}

void _gpioResetDebounceLine(uint32_t line){
    active_debounce_lines &= ~(0x01UL<<line);
    EXTI->IMR |= (0x01UL<<line);
    EXTI->PR |= (0x01UL<<line);
}

static void _gpioDebounce(uint32_t line){
    //Disable EXTI line
    EXTI->IMR &= ~(0x01UL<<line);
    //Set debounce counter; it will be decremented by SysTick interrupt
    gpio_handlers[line].debounce_cnt = 0;
    active_debounce_lines |= (0x01UL<<line);
}

void _exti_handler(uint32_t line){
    if (line > 15) return;
    if (gpio_handlers[line].f){
        if (gpio_handlers[line].debounce_t){
            gpio_handlers[line].trig_lvl = gpioGetInput(gpio_handlers[line].pin);
            _gpioDebounce(line);
        }
        else
            (gpio_handlers[line].f)(line);
    }
    EXTI->PR |= (0x01UL<<line);
}

void gpioEnableInterrupt(uint32_t pin, GpioIntMode_t mode, uint32_t debounce){
    //set NVIC
    uint32_t line = _PINS_2_PORT(A, pin) | _PINS_2_PORT(B, pin) | _PINS_2_PORT(C, pin);
    line = _MSB(line);

    gpio_handlers[line].pin = pin;
    gpio_handlers[line].mode = mode;
    gpio_handlers[line].debounce_t = debounce;

    IRQn_Type irq_line = (line>=10) ? EXTI15_10_IRQn : ( (line>=5) ? EXTI9_5_IRQn : (EXTI0_IRQn+line) );

    NVIC_SetPriority(irq_line, 4);
    NVIC_EnableIRQ(irq_line);

    //set EXTI

    EXTI->RTSR |= (_IS_SET(mode, INT_RISING)<<line);
    EXTI->RTSR &= ~((!_IS_SET(mode, INT_RISING))<<line);

    EXTI->FTSR |= (_IS_SET(mode, INT_FALLING)<<line);
    EXTI->FTSR &= ~((!_IS_SET(mode, INT_FALLING))<<line);

    EXTI->IMR |= (0x01<<line);
}

void gpioRegisterInterruptHandler(uint32_t pin, GpioIntHandler_f handler){
    uint32_t line = _PINS_2_PORT(A, pin) | _PINS_2_PORT(B, pin) | _PINS_2_PORT(C, pin);
    line = _MSB(line);

    gpio_handlers[line].f = handler;
}

static void _setPortPinsAsOutput(GPIO_TypeDef *port, uint32_t pins, OutputMode_t mode){
    while( pins ){
        uint32_t pin_no = _MSB(pins);
        port->MODER &= ~(0x01UL<<(2*(pin_no)+1));
        port->MODER |= (0x01UL<<(2*(pin_no)));

        port->OTYPER &= ~(_IS_SET(mode, OUTPUT_PUSHPULL)<<(pin_no));
        port->OTYPER |= (_IS_SET(mode, OUTPUT_OD)<<(pin_no));

        port->OSPEEDR &= ~(0x01UL<<(2*(pin_no)));
        port->OSPEEDR |= (0x01UL<<(2*(pin_no)+1));

        port->PUPDR &= ~(((_IS_SET(mode, OUTPUT_PULLUP)<<1)|_IS_SET(mode, OUTPUT_PULLDOWN))<<(2*(pin_no)));
        port->PUPDR |= (((_IS_SET(mode, OUTPUT_PULLDOWN)<<1)|_IS_SET(mode, OUTPUT_PULLUP))<<(2*(pin_no)));
        pins &= ~(0x01UL<<pin_no);
    }
}

static void _setPortPinsAsInput(GPIO_TypeDef *port, uint32_t pins, InputMode_t mode){
    while( pins ){
        uint32_t pin_no = _MSB(pins);
        port->MODER &= ~(0x11UL<<(2*(pin_no)));

        port->PUPDR &= ~(((_IS_SET(mode, INPUT_PULLUP)<<1)|_IS_SET(mode, INPUT_PULLDOWN))<<(2*(pin_no)));
        port->PUPDR |= (((_IS_SET(mode, INPUT_PULLDOWN)<<1)|_IS_SET(mode, INPUT_PULLUP))<<(2*(pin_no)));
        pins &= ~(0x01UL<<pin_no);

    }
}

//static void _setPortOutput(GPIO_TypeDef *port, uint32_t pins){
#define _SET_PORT_OUTPUT(P, pins, state){\
    GPIO##P->ODR &= ~_PINS_2_PORT(P, pins^(pins&state));\
    GPIO##P->ODR |=  (_PINS_2_PORT(P, pins&state));\
}


#define _GET_PORT_INPUT(P, pins) _PORT_2_PINS(P, GPIO##P->IDR&(_PINS_2_PORT(P, pins)))

#define _GET_PORT_OUTPUT(P, pins) _PORT_2_PINS(P, GPIO##P->ODR&(_PINS_2_PORT(P, pins)))




void gpioSetPinsAsOutput(uint32_t pins, OutputMode_t mode){
    _setPortPinsAsOutput(GPIOA, _PINS_2_PORT(A, pins), mode);
    _setPortPinsAsOutput(GPIOB, _PINS_2_PORT(B, pins), mode);
    _setPortPinsAsOutput(GPIOC, _PINS_2_PORT(C, pins), mode);
}


void gpioSetPinsAsInput(uint32_t pins, InputMode_t mode){
    _setPortPinsAsInput(GPIOA, _PINS_2_PORT(A, pins), mode);
    _setPortPinsAsInput(GPIOB, _PINS_2_PORT(B, pins), mode);
    _setPortPinsAsInput(GPIOC, _PINS_2_PORT(C, pins), mode);
}



void gpioSetOutput(uint32_t pins, uint32_t state){
    _SET_PORT_OUTPUT(A, pins, state);
    _SET_PORT_OUTPUT(B, pins, state);
    _SET_PORT_OUTPUT(C, pins, state);
}


uint32_t gpioGetInput(uint32_t pins){
    uint32_t ret = 0;
    ret |= _GET_PORT_INPUT(A, pins);
    ret |= _GET_PORT_INPUT(B, pins);
    ret |= _GET_PORT_INPUT(C, pins);

    return ret;
}

uint32_t gpioGetInputDebounced(uint32_t pins, uint32_t debounce_cnt){
    int32_t ret = gpioGetInput(pins);
    int32_t state;
    int32_t debounce = 0;
    for(uint32_t i = 0; i < 2*debounce_cnt; i++){
        msDelay(1);
        if (debounce == debounce_cnt)
            return ret;
        state = gpioGetInput(pins);
        if (state == ret)
            debounce++;
        else
            debounce = 0;
        ret = state;
    }
    return ret;
}

uint32_t gpioGetOutput(uint32_t pins){
    uint32_t ret = 0;
    ret |= _GET_PORT_OUTPUT(A, pins);
    ret |= _GET_PORT_OUTPUT(B, pins);
    ret |= _GET_PORT_OUTPUT(C, pins);

    return ret;
}
