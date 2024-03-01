#include "app.h"

void app(){
    uint32_t state = 0;

    gpioSetPinsAsOutput(LED, OUTPUT_PUSHPULL|OUTPUT_NOPULL);
  
    while (1)
    {
        msDelay(750);
        state = gpioGetOutput(LED);
        gpioSetOutput(LED, !state);
    }
}