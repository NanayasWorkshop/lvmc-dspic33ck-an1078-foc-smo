#ifndef __SINGLESHUNT_H
#define	__SINGLESHUNT_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#include <clock.h>     
#include "pwm.h"    
#include "general.h"
#include "userparms.h"    
#include "motor_control_noinline.h" 
      
#define KCURRBUS        Q15(-0.5) 
#define SSTCRITINSEC	3.0E-6	
#define SSTCRIT         (uint16_t)(SSTCRITINSEC*FCY*2)  
#define SS_SAMPLE_DELAY  100  
    
typedef struct
{
    int16_t T1;
    int16_t T2;
    int16_t T7;
    int16_t Ta1;
    int16_t Ta2;
    int16_t Tb1;
    int16_t Tb2;
    int16_t Tc1;
    int16_t Tc2;
    int16_t sectorSVM;
    int16_t tcrit;
    int16_t tDelaySample;
    
    int16_t Ia;
    int16_t Ib;
    int16_t Ic;
    int16_t Ibus1;
    int16_t Ibus2;	
    int16_t trigger1;
    int16_t trigger2;       
    int16_t adcSamplePoint;
    MC_DUTYCYCLEOUT_T pwmDutycycle1;
    MC_DUTYCYCLEOUT_T pwmDutycycle2;
    
} SINGLE_SHUNT_PARM_T;

typedef enum tagSSADCSAMPLE_STATE
{ 
    SS_SAMPLE_BUS1 = 0,
    SS_SAMPLE_BUS2 = 1,  
     
}SSADCSAMPLE_STATE;
			
extern SINGLE_SHUNT_PARM_T singleShuntParam;
uint16_t SingleShunt_CalculateSpaceVectorPhaseShifted(MC_ABC_T *pABC,
                                                     uint16_t iPwmPeriod,
                                                     SINGLE_SHUNT_PARM_T *);
void SingleShunt_PhaseCurrentReconstruction(SINGLE_SHUNT_PARM_T *);
void SingleShunt_InitializeParameters(SINGLE_SHUNT_PARM_T *);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

