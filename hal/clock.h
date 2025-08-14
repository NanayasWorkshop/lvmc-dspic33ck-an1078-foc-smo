#ifndef _CLOCK_H
#define _CLOCK_H

#ifdef __cplusplus
    extern "C" {
#endif
#include <xc.h>
#include <stdint.h>

#define FOSC                    200000000UL
#define FOSC_MHZ                200U  
#define FCY                     100000000UL
#define FCY_MHZ                 100U

void InitOscillator(void);
void EnableREFCLKOutput(uint16_t);
#ifdef __cplusplus  
}
#endif
#endif      // end of CLOCK_H


