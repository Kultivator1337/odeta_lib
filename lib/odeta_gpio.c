#include "odeta_gpio.h"

static GpioIntHandler_f gpio_handlers[16] = {0};

void exti_handler(uint32_t line){
    if (line > 15) return;
    if (gpio_handlers[line]){
        (gpio_handlers[line])(line);
    }
    EXTI->PR |= (0x01UL<<line);
}

void gpioEnableInterrupt(uint32_t pin, GpioIntMode_t mode){
    //set NVIC
    uint32_t line = _PORT(A, pin) | _PORT(B, pin) | _PORT(C, pin);
    line = _BIT_POS(line);

    NVIC_SetPriority();
    NVIC_EnableIRQ();

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
    gpio_handlers[line] = handler;
}

void setPortPinsAsOutput(GPIO_TypeDef *port, uint32_t pins, OutputMode_t mode){
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

void setPortPinsAsInput(GPIO_TypeDef *port, uint32_t pins, InputMode_t mode){
    while( pins ){
        uint32_t pin_no = _BIT_POS(pins);
        port->MODER &= ~(0x11UL<<(2*(pin_no)));

        port->PUPDR &= ~(((_IS_SET(mode, INPUT_PULLUP)<<1)|_IS_SET(mode, INPUT_PULLDOWN))<<(2*(pin_no)));
        port->PUPDR |= (((_IS_SET(mode, INPUT_PULLDOWN)<<1)|_IS_SET(mode, INPUT_PULLUP))<<(2*(pin_no)));
        pins &= ~(0x01UL<<pin_no);

    }
}

void setPortOutput(GPIO_TypeDef *port, uint32_t pins, uint8_t state){
    if (state)
        port->ODR |= pins;
    else
        port->ODR &= ~pins;
}

uint32_t getPortInput(GPIO_TypeDef *port, uint32_t pins){
    return ((port->IDR & pins) == pins);
}

uint32_t getPortOutput(GPIO_TypeDef *port, uint32_t pins){
    return ((port->ODR & pins) == pins);
}

void gpioSetPinsAsOutput(uint32_t pins, OutputMode_t mode){
    setPortPinsAsOutput(GPIOA, _PORT(A, pins), mode);
    setPortPinsAsOutput(GPIOB, _PORT(B, pins), mode);
    setPortPinsAsOutput(GPIOC, _PORT(C, pins), mode);
}


void gpioSetPinsAsInput(uint32_t pins, InputMode_t mode){
    setPortPinsAsInput(GPIOA, _PORT(A, pins), mode);
    setPortPinsAsInput(GPIOB, _PORT(B, pins), mode);
    setPortPinsAsInput(GPIOC, _PORT(C, pins), mode);
}



void gpioSetOutput(uint32_t pins, uint8_t state){
    setPortOutput(GPIOA, _PORT(A, pins), state);
    setPortOutput(GPIOB, _PORT(B, pins), state);
    setPortOutput(GPIOC, _PORT(C, pins), state);
}


uint32_t gpioGetInput(uint32_t pins){
    uint32_t ret = 1;
    ret &= getPortInput(GPIOA, _PORT(A, pins));
    ret &= getPortInput(GPIOB, _PORT(B, pins));
    ret &= getPortInput(GPIOC, _PORT(C, pins));

    return ret;
}

uint32_t gpioGetOutput(uint32_t pins){
    uint32_t ret = 1;
    ret &= getPortOutput(GPIOA, _PORT(A, pins));
    ret &= getPortOutput(GPIOB, _PORT(B, pins));
    ret &= getPortOutput(GPIOC, _PORT(C, pins));

    return ret;
}
