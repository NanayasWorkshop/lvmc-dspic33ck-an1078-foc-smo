#ifndef _PORTCONFIG_H
#define _PORTCONFIG_H

#include <xc.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define SW1                   PORTEbits.RE11
#define SW2                   PORTEbits.RE12

#define BUTTON_START_STOP        SW1
#define BUTTON_SPEED_HALF_DOUBLE      SW2

#define LED2                    LATEbits.LATE7
#define LED1                    LATEbits.LATE6

void SetupGPIOPorts(void);

#ifdef __cplusplus
    }
#endif
#endif  