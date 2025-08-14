#ifndef _PWM_H
#define _PWM_H

#ifdef __cplusplus
    extern "C" {
#endif
#include <xc.h>
#include <stdint.h>
#include "clock.h"

#define PWM_PDC1      PG1DC
#define PWM_PDC2      PG2DC
#define PWM_PDC3      PG4DC

#define PWM_PHASE1    PG1PHASE 
#define PWM_PHASE2    PG2PHASE
#define PWM_PHASE3    PG4PHASE  
        
#define PWM_TRIGA     PG1TRIGA 
#define PWM_TRIGB     PG1TRIGB   
#define PWM_TRIGC     PG1TRIGC

#define _PWMInterrupt           _PWM1Interrupt
#define ClearPWMIF()            _PWM1IF = 0   

#define PWMFREQUENCY_HZ         20000
#define DEADTIME_MICROSEC       1.0
#define LOOPTIME_SEC            0.00005
#define LOOPTIME_MICROSEC       50

#define BOOTSTRAP_CHARGING_TIME_SECS 0.01

#define BOOTSTRAP_CHARGING_COUNTS (uint16_t)((BOOTSTRAP_CHARGING_TIME_SECS/LOOPTIME_SEC )* 2)

#define ENABLE_PWM_FAULT

#define DDEADTIME               (uint16_t)(DEADTIME_MICROSEC*FOSC_MHZ)
#define LOOPTIME_TCY            (uint16_t)(((LOOPTIME_MICROSEC*FOSC_MHZ)/2)-1)

#define ADC_SAMPLING_POINT      0x0000

#define MIN_DUTY            0x0000
void InitPWMGenerators(void);        
#ifdef __cplusplus  
    }
#endif

#endif   


