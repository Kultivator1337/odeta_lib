#include "odeta_gpio.h"

void setPortPinsAsOutput(GPIO_TypeDef *port, uint32_t pins, OutputMode_t mode){
    while( pins ){
        uint32_t pin_no = 31 - __CLZ(pins);
        uint32_t sh = (port == GPIOA)?_PA_SH(pin_no):(port==GPIOB)?_PB_SH(pin_no):_PC_SH(pin_no);
        port->MODER &= ~(0x01UL<<(2*(pin_no + sh)+1));
        port->MODER |= (0x01UL<<(2*(pin_no + sh)));

        port->OTYPER &= ~(_IS_SET(mode, OUTPUT_PUSHPULL)<<(pin_no + sh));
        port->OTYPER |= (_IS_SET(mode, OUTPUT_OD)<<(pin_no + pin_no + sh));

        port->OSPEEDR &= ~(0x01UL<<(2*(pin_no + sh)));
        port->OSPEEDR |= (0x01UL<<(2*(pin_no + sh)+1));

        port->PUPDR &= ~(((_IS_SET(mode, OUTPUT_PULLUP)<<1)|_IS_SET(mode, OUTPUT_PULLDOWN))<<(2*(pin_no + sh)));
        port->PUPDR |= (((_IS_SET(mode, OUTPUT_PULLDOWN)<<1)|_IS_SET(mode, OUTPUT_PULLUP))<<(2*(pin_no + sh)));
        pins &= ~(0x01UL<<pin_no);
    }
}

void setPortPinsAsInput(GPIO_TypeDef *port, uint32_t pins, InputMode_t mode){
    while( pins ){
        uint32_t pin_no = 31 - __CLZ(pins);
        uint32_t sh = (port == GPIOA)?_PA_SH(pin_no):(port==GPIOB)?_PB_SH(pin_no):_PC_SH(pin_no);
        port->MODER &= ~(0x11UL<<(2*(pin_no + sh)));

        port->PUPDR &= ~(((_IS_SET(mode, INPUT_PULLUP)<<1)|_IS_SET(mode, INPUT_PULLDOWN))<<(2*(pin_no + sh)));
        port->PUPDR |= (((_IS_SET(mode, INPUT_PULLDOWN)<<1)|_IS_SET(mode, INPUT_PULLUP))<<(2*(pin_no + sh)));
        pins &= ~(0x01UL<<pin_no);

    }
}

void setPortOutput(GPIO_TypeDef *port, uint32_t pins, uint8_t state){
    while( pins ){
        uint32_t pin_no = 31 - __CLZ(pins);
        uint32_t sh = (port == GPIOA)?_PA_SH(pin_no):(port==GPIOB)?_PB_SH(pin_no):_PC_SH(pin_no);
        
        port->ODR &= ~((state?0x00UL:0x01UL)<<((pin_no + sh)));
        port->ODR |= ((state?0x01UL:0x00UL)<<((pin_no + sh)));
        pins &= ~(0x01UL<<pin_no);

    }
}

uint32_t getPortInput(GPIO_TypeDef *port, uint32_t pins){
    uint32_t ret = 1;
    while( pins ){
        uint32_t pin_no = 31 - __CLZ(pins);
        uint32_t sh = (port == GPIOA)?_PA_SH(pin_no):(port==GPIOB)?_PB_SH(pin_no):_PC_SH(pin_no);
        ret &= (_IS_SET(port->IDR, (0x01UL<<(pin_no+sh))));
        pins &= ~(0x01UL<<pin_no);

    }
    return ret;
}

uint32_t getPortOutput(GPIO_TypeDef *port, uint32_t pins){
    uint32_t ret = 1;
    while( pins ){
        uint32_t pin_no = 31 - __CLZ(pins);
        uint32_t sh = (port == GPIOA)?_PA_SH(pin_no):(port==GPIOB)?_PB_SH(pin_no):_PC_SH(pin_no);
        ret &= (_IS_SET(port->ODR, (0x01UL<<(pin_no+sh))));
        pins &= ~(0x01UL<<pin_no);

    }
    return ret;
}

void setPinsAsOutput(uint32_t pins, OutputMode_t mode){
    GpioPins_t *p = (GpioPins_t *)&pins;
    
    setPortPinsAsOutput(GPIOA, p->pa_pins, mode);
    setPortPinsAsOutput(GPIOB, p->pb_pins, mode);
    setPortPinsAsOutput(GPIOC, p->pc_pins, mode);
}


void setPinsAsInput(uint32_t pins, InputMode_t mode){
    GpioPins_t *p = (GpioPins_t *)&pins;    
    
    setPortPinsAsInput(GPIOA, p->pa_pins, mode);
    setPortPinsAsInput(GPIOB, p->pb_pins, mode);
    setPortPinsAsInput(GPIOC, p->pc_pins, mode);
}



void setOutput(uint32_t pins, uint8_t state){
    GpioPins_t *p = (GpioPins_t *)&pins;
    setPortOutput(GPIOA, p->pa_pins, state);
    setPortOutput(GPIOB, p->pb_pins, state);
    setPortOutput(GPIOC, p->pc_pins, state);
}


uint32_t getInput(uint32_t pins){
    uint32_t ret = 1;
    GpioPins_t *p = (GpioPins_t *)&pins;

    ret &= getPortInput(GPIOA, p->pa_pins);
    ret &= getPortInput(GPIOB, p->pb_pins);
    ret &= getPortInput(GPIOC, p->pc_pins);

    return ret;
}

uint32_t getOutput(uint32_t pins){
    uint32_t ret = 1;
    GpioPins_t *p = (GpioPins_t *)&pins;

    ret &= getPortOutput(GPIOA, p->pa_pins);
    ret &= getPortOutput(GPIOB, p->pb_pins);
    ret &= getPortOutput(GPIOC, p->pc_pins);

    return ret;
}
