#ifndef _MOTOR_CONTROL_TYPES_H_  
#define _MOTOR_CONTROL_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus 
    extern "C" {
#endif

 typedef struct
{
    int16_t min;
    
    int16_t max;    
} MC_minmax16_t;

typedef struct
{
    int16_t alpha;
    
    int16_t beta;
} MC_ALPHABETA_T;

typedef struct
{
    int16_t cos;

    int16_t sin;
} MC_SINCOS_T;

typedef struct
{
    int16_t d;
    
    int16_t q;
} MC_DQ_T;

typedef struct
{
    uint16_t dutycycle1;
    
    uint16_t dutycycle2;
    
    uint16_t dutycycle3;
} MC_DUTYCYCLEOUT_T;

typedef struct
{
    int16_t a;
    
    int16_t b;
    
    int16_t c;
} MC_ABC_T;

typedef struct
{
    int32_t integrator;
    
    int16_t kp;
    
    int16_t ki;
    
    int16_t kc;

    int16_t outMax;
    
    int16_t outMin;
} MC_PISTATE_T;

typedef struct
{
    MC_PISTATE_T piState;
    
    int16_t inReference;
    
    int16_t inMeasure;
} MC_PIPARMIN_T;

typedef struct
{
    int16_t out;        
} MC_PIPARMOUT_T;

typedef  struct
{

    int16_t offset; 
    
    int16_t diff;    
}MC_SINDIFF_T;


#ifdef __cplusplus
    }
#endif
#endif