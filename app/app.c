#include "app.h"

void app(){
    gpioSetPinsAsOutput(LED, OUTPUT_PUSHPULL|OUTPUT_NOPULL);

    while (1)
    {
        msDelay(750);
        gpioSetOutputPin(LED, ~gpioGetOutput(LED));
    }
}