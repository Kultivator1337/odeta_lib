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

void exti_handler(uint32_t line){
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
    uint32_t line = _PORT(A, pin) | _PORT(B, pin) | _PORT(C, pin);
    line = _BIT_POS(line);

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
    uint32_t line = _PORT(A, pin) | _PORT(B, pin) | _PORT(C, pin);
    line = _BIT_POS(line);

    gpio_handlers[line].f = handler;
}

static void _setPortPinsAsOutput(GPIO_TypeDef *port, uint32_t pins, OutputMode_t mode){
    while( pins ){
        uint32_t pin_no = _BIT_POS(pins);
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
        uint32_t pin_no = _BIT_POS(pins);
        port->MODER &= ~(0x11UL<<(2*(pin_no)));

        port->PUPDR &= ~(((_IS_SET(mode, INPUT_PULLUP)<<1)|_IS_SET(mode, INPUT_PULLDOWN))<<(2*(pin_no)));
        port->PUPDR |= (((_IS_SET(mode, INPUT_PULLDOWN)<<1)|_IS_SET(mode, INPUT_PULLUP))<<(2*(pin_no)));
        pins &= ~(0x01UL<<pin_no);

    }
}

static void _setPortOutput(GPIO_TypeDef *port, uint32_t pins, uint8_t state){
    if (state)
        port->ODR |= pins;
    else
        port->ODR &= ~pins;
}

static uint32_t _getPortInput(GPIO_TypeDef *port, uint32_t pins){
    return ((port->IDR & pins) == pins);
}

static uint32_t _getPortOutput(GPIO_TypeDef *port, uint32_t pins){
    return ((port->ODR & pins) == pins);
}

void gpioSetPinsAsOutput(uint32_t pins, OutputMode_t mode){
    _setPortPinsAsOutput(GPIOA, _PORT(A, pins), mode);
    _setPortPinsAsOutput(GPIOB, _PORT(B, pins), mode);
    _setPortPinsAsOutput(GPIOC, _PORT(C, pins), mode);
}


void gpioSetPinsAsInput(uint32_t pins, InputMode_t mode){
    _setPortPinsAsInput(GPIOA, _PORT(A, pins), mode);
    _setPortPinsAsInput(GPIOB, _PORT(B, pins), mode);
    _setPortPinsAsInput(GPIOC, _PORT(C, pins), mode);
}



void gpioSetOutput(uint32_t pins, uint8_t state){
    _setPortOutput(GPIOA, _PORT(A, pins), state);
    _setPortOutput(GPIOB, _PORT(B, pins), state);
    _setPortOutput(GPIOC, _PORT(C, pins), state);
}


uint32_t gpioGetInput(uint32_t pins){
    uint32_t ret = 1;
    ret &= _getPortInput(GPIOA, _PORT(A, pins));
    ret &= _getPortInput(GPIOB, _PORT(B, pins));
    ret &= _getPortInput(GPIOC, _PORT(C, pins));

    return ret;
}

uint32_t gpioGetOutput(uint32_t pins){
    uint32_t ret = 1;
    ret &= _getPortOutput(GPIOA, _PORT(A, pins));
    ret &= _getPortOutput(GPIOB, _PORT(B, pins));
    ret &= _getPortOutput(GPIOC, _PORT(C, pins));

    return ret;
}
