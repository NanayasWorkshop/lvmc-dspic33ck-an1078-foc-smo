#ifndef _ADC_H
#define _ADC_H

#ifdef __cplusplus
    extern "C" {
#endif

#include <xc.h>
#include <stdint.h>

#include "userparms.h"

#define ADCBUF_IPHASE1    -ADCBUF0
#define ADCBUF_IPHASE2    -ADCBUF1
#define ADCBUF_IBUS       ADCBUF4  
        
#define ADCBUF_SPEED_REF      ADCBUF11
#define ADCBUF_VBUS           ADCBUF15
#define ADCBUF_MOSFET_TEMP    ADCBUF12
        

#ifdef SINGLE_SHUNT       
 #define EnableADCInterrupt()   _ADCAN4IE = 1
 #define DisableADCInterrupt()  _ADCAN4IE = 0
 #define ClearADCIF()           _ADCAN4IF = 0
 #define ClearADCIF_ReadADCBUF() ADCBUF4        
 #define _ADCInterrupt _ADCAN4Interrupt  
#else
 #define EnableADCInterrupt()   _ADCAN11IE = 1
 #define DisableADCInterrupt()  _ADCAN11IE = 0
 #define ClearADCIF()           _ADCAN11IF = 0
 #define ClearADCIF_ReadADCBUF() ADCBUF11        
 #define _ADCInterrupt _ADCAN11Interrupt  
#endif      
void InitializeADCs(void);

#ifdef __cplusplus
    }
#endif
#endif

