#ifndef _MOTOR_CONTROL_H_  
#define _MOTOR_CONTROL_H_

#include <stdint.h>
#ifdef __XC16__
#include <xc.h>
#endif 

#ifdef __cplusplus  
    extern "C" {
#endif

#ifdef __MATLAB_MEX__ 
#define inline
#endif 


#include "motor_control_declarations.h"
#include "motor_control_inline_declarations.h"


#ifdef __XC16__   
#include "./motor_control_inline_dspic.h"
#endif

#ifdef __cplusplus  
    }
#endif
#endif 


