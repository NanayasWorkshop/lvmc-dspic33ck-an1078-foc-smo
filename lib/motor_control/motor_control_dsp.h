
#ifndef _MOTOR_CONTROL_DSP_H_
#define _MOTOR_CONTROL_DSP_H_

#ifdef __cplusplus  
    extern "C" {
#endif
        
#ifndef DSP_ACCUMULATOR_A_DEFINED		
#define DSP_ACCUMULATOR_A_DEFINED		
/** DSP accumulator A */
volatile register int a_Reg asm("A");
#endif

#ifndef DSP_ACCUMULATOR_B_DEFINED
#define DSP_ACCUMULATOR_B_DEFINED		
/** DSP accumulator B */
volatile register int b_Reg asm("B");
#endif

#ifdef __cplusplus  
    }
#endif

#endif 




