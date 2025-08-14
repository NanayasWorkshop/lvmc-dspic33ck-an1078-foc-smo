#ifndef _MOTOR_CONTROL_INLINE_DSPIC_H_
#define _MOTOR_CONTROL_INLINE_DSPIC_H_

#include <xc.h>
#include <stdint.h>
#include "motor_control_util.h"
#include "motor_control_inline_internal.h"

#include "motor_control_dsp.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
#ifdef __dsPIC33F__
__psv__ extern uint16_t MC_SineTableInFlash[] __attribute__((space(psv)));
#elif defined(__dsPIC33E__) || defined(__dsPIC33C__)
__eds__ extern uint16_t MC_SineTableInFlash[] __attribute__((space(psv)));
#else
#error The selected device is not compatible with the Motor Control library!
#endif

extern uint16_t       MC_SineTableInRam[];

enum {
   MC_LATENCY_CONTROL     = 0x8000,
   MC_UNSIGNED_MULTIPLY   = 0x1000,
   MC_MIXED_MULTPLY       = 0x2000,
   MC_DO_lOOP_TERMINATION = 0x800,
   MC_DO_LOOP_7           = 0x700,
   MC_DO_LOOP_6           = 0x600,
   MC_DO_LOOP_5           = 0x500,
   MC_DO_LOOP_4           = 0x400,
   MC_DO_LOOP_3           = 0x300,
   MC_DO_LOOP_2           = 0x200,
   MC_DO_LOOP_1           = 0x100, 
   MC_SAT_ACCA_ENABLE     = 0x80,
   MC_SAT_ACCB_ENABLE     = 0x40,
   MC_SAT_DATASPACE       = 0x20,
   MC_SAT_SUPER_MODE      = 0x10,
   MC_CPU_HIGH_PRIORITY   = 0x8,
   MC_STACK_FRAME_ACTIVE  = 0x4,
   MC_ROUND_MODE_BIASED   = 0x2,
   MC_INTEGER_MODE        = 0x1,
};

enum {
   MC_CORECONTROL = ( MC_SAT_ACCA_ENABLE | MC_SAT_ACCB_ENABLE | MC_SAT_DATASPACE | MC_ROUND_MODE_BIASED ) 
};

static inline uint16_t MC_TransformPark_InlineC( const MC_ALPHABETA_T *alphabeta, const MC_SINCOS_T *sincos, MC_DQ_T *dq)
{
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    a_Reg = __builtin_mpy(alphabeta->alpha, sincos->cos,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, alphabeta->beta, sincos->sin,0,0,0,0,0,0,0,0);
    dq->d = __builtin_sacr(a_Reg,0);

    a_Reg = __builtin_mpy(alphabeta->beta, sincos->cos,0,0,0,0,0,0);
    a_Reg = __builtin_msc(a_Reg, alphabeta->alpha, sincos->sin,0,0,0,0,0,0,0,0);
    dq->q = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}

       
static inline uint16_t MC_ControllerPIUpdate_InlineC(int16_t in_Ref, int16_t in_Meas, MC_PISTATE_T *state, int16_t *out)
{
    int16_t error;
    int16_t out_Buffer;
    int16_t output;

    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    a_Reg = __builtin_lac(in_Ref,0);
    b_Reg = __builtin_lac(in_Meas,0);
    a_Reg = __builtin_subab(a_Reg,b_Reg);
    error = __builtin_sacr(a_Reg,0);

    MC_UTIL_writeAccB32(state->integrator);

    a_Reg = __builtin_mpy(error, state->kp,0,0,0,0,0,0);
    a_Reg = __builtin_sftac(a_Reg,-4);
    a_Reg = __builtin_addab(a_Reg,b_Reg);
    out_Buffer = __builtin_sacr(a_Reg,0);

    if(out_Buffer > state->outMax)
    {
        output = state->outMax;
    }
    else if(out_Buffer < state->outMin)
    {
        output = state->outMin;
    }
    else
    {
        output = out_Buffer;
    }
    *out = output;    

    a_Reg = __builtin_mpy(error, state->ki,0,0,0,0,0,0);

    error = out_Buffer - output;
    a_Reg = __builtin_msc(a_Reg, error, state->kc,0,0,0,0,0,0,0,0);

    a_Reg = __builtin_addab(a_Reg,b_Reg);
    asm volatile ("" : "+w"(a_Reg):); 

    state->integrator = MC_UTIL_readAccA32();
    CORCON = mcCorconSave;
    
    return 1;
}


static inline uint16_t MC_TransformClarke_InlineC( const MC_ABC_T *abc, MC_ALPHABETA_T *alphabeta)
{
    const uint16_t MC_ONEBYSQ3 = 18919u;
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    alphabeta->alpha = abc->a;

    a_Reg = __builtin_mpy(abc->a, MC_ONEBYSQ3,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, MC_ONEBYSQ3, abc->b,0,0,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, MC_ONEBYSQ3, abc->b,0,0,0,0,0,0,0,0);
    alphabeta->beta = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}


static inline void MC_TransformClarkeABC_InlineC(const MC_ABC_T *abc, MC_ALPHABETA_T *alphabeta)
{
    const int16_t tan30Q16      = 37837U; 
    const int16_t one_thirdQ16  = 21845U; 
    const int16_t two_thirdsQ16 = 43690U; 
    
    alphabeta->alpha = MC_UTIL_mulus16(two_thirdsQ16, abc->a)
                     - MC_UTIL_mulus16(one_thirdQ16, abc->b)
                     - MC_UTIL_mulus16(one_thirdQ16, abc->c);
    
    alphabeta->beta  = MC_UTIL_mulus16(tan30Q16, abc->b)
                     - MC_UTIL_mulus16(tan30Q16, abc->c);
}

static inline uint16_t MC_TransformParkInverse_InlineC(const MC_DQ_T *dq, const MC_SINCOS_T *sincos, MC_ALPHABETA_T *alphabeta)
{
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    a_Reg = __builtin_mpy(dq->d, sincos->cos,0,0,0,0,0,0);
    a_Reg = __builtin_msc(a_Reg, dq->q, sincos->sin,0,0,0,0,0,0,0,0);
    alphabeta->alpha = __builtin_sacr(a_Reg,0);

    a_Reg = __builtin_mpy(dq->d, sincos->sin,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, dq->q, sincos->cos,0,0,0,0,0,0,0,0);
    alphabeta->beta = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}


static inline uint16_t MC_TransformClarkeInverseSwappedInput_InlineC( const MC_ALPHABETA_T *alphabeta, MC_ABC_T *abc)
{
    const int16_t MC_SQ3OV2 = 28378u;  
    const int16_t MC_POINT5 = 0x4000;
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    abc->a = alphabeta->beta;

    a_Reg = __builtin_clr();
    a_Reg = __builtin_msc(a_Reg, alphabeta->beta, MC_POINT5,0,0,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, alphabeta->alpha, MC_SQ3OV2,0,0,0,0,0,0,0,0);
    abc->b = __builtin_sacr(a_Reg,0);

    a_Reg = __builtin_clr();
    a_Reg = __builtin_msc(a_Reg, alphabeta->beta, MC_POINT5,0,0,0,0,0,0,0,0);
    a_Reg = __builtin_msc(a_Reg, alphabeta->alpha, MC_SQ3OV2,0,0,0,0,0,0,0,0);
    abc->c = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}

static inline uint16_t MC_TransformClarkeInverse_InlineC( const MC_ALPHABETA_T *alphabeta, MC_ABC_T *abc)
{
    const int16_t MC_SQ3OV2 = 28378u; 
    const int16_t MC_NEGPOINT5 = 0xC000; 
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    abc->a = alphabeta->alpha;
    
    a_Reg = __builtin_mpy(alphabeta->alpha, MC_NEGPOINT5,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, alphabeta->beta, MC_SQ3OV2,0,0,0,0,0,0,0,0);
    abc->b = __builtin_sacr(a_Reg,0);

    a_Reg = __builtin_mpy(alphabeta->alpha, MC_NEGPOINT5,0,0,0,0,0,0);
    a_Reg = __builtin_msc(a_Reg, alphabeta->beta, MC_SQ3OV2,0,0,0,0,0,0,0,0);
    abc->c = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}

static inline void MC_TransformClarkeInverseNoAccum_InlineC( const MC_ALPHABETA_T *alphabeta, MC_ABC_T *abc)
{
    const uint16_t cos30Q16 = 56756u;

	const int16_t alpha_sin30 = alphabeta->alpha >> 1;
	const int16_t beta_cos30 = MC_UTIL_mulus16(cos30Q16, alphabeta->beta);

    abc->a = alphabeta->alpha;	
    abc->b = -alpha_sin30 + beta_cos30;
    abc->c = -alpha_sin30 - beta_cos30;
}

static inline uint16_t MC_CalculateSpaceVectorPhaseShifted_InlineC( const MC_ABC_T *abc, uint16_t period, MC_DUTYCYCLEOUT_T *pdcout)
{
    int16_t T1, T2, Ta, Tb, Tc;
    
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    if (abc->a >= 0)
    {
        if (abc->b >= 0)
        {
            T1 = abc->a;
            T2 = abc->b;
                a_Reg = __builtin_mulus(period, T1);
                T1 = __builtin_sacr(a_Reg,0);
                a_Reg = __builtin_mulus(period, T2);
                T2 = __builtin_sacr(a_Reg,0);
                Tc = period-T1-T2;
                Tc = Tc >> 1;
                Tb = Tc + T1;
                Ta = Tb + T2;
            pdcout->dutycycle1 = Ta;
            pdcout->dutycycle2 = Tb;
            pdcout->dutycycle3 = Tc;
        }
        else
        {
            if (abc->c >= 0)
            {
                T1 = abc->c;
                T2 = abc->a;
                    a_Reg = __builtin_mulus(period, T1);
                    T1 = __builtin_sacr(a_Reg,0);
                    a_Reg = __builtin_mulus(period, T2);
                    T2 = __builtin_sacr(a_Reg,0);
                    Tc = period-T1-T2;
                    Tc = Tc >> 1;
                    Tb = Tc + T1;
                    Ta = Tb + T2;
                pdcout->dutycycle1 = Tc;
                pdcout->dutycycle2 = Ta;
                pdcout->dutycycle3 = Tb;
            }
            else
            {
                T1 = -abc->c;
                T2 = -abc->b;
                    a_Reg = __builtin_mulus(period, T1);
                    T1 = __builtin_sacr(a_Reg,0);
                    a_Reg = __builtin_mulus(period, T2);
                    T2 = __builtin_sacr(a_Reg,0);
                    Tc = period-T1-T2;
                    Tc = Tc >> 1;
                    Tb = Tc + T1;
                    Ta = Tb + T2;
                pdcout->dutycycle1 = Tb;
                pdcout->dutycycle2 = Ta;
                pdcout->dutycycle3 = Tc;
            }
        }
    }
    else
    {
        if (abc->b >= 0)
        {
            if (abc->c >= 0)
            {
                T1 = abc->b;
                T2 = abc->c;
                    a_Reg = __builtin_mulus(period, T1);
                    T1 = __builtin_sacr(a_Reg,0);
                    a_Reg = __builtin_mulus(period, T2);
                    T2 = __builtin_sacr(a_Reg,0);
                    Tc = period-T1-T2;
                    Tc = Tc >> 1;
                    Tb = Tc + T1;
                    Ta = Tb + T2;
                pdcout->dutycycle1 = Tb;
                pdcout->dutycycle2 = Tc;
                pdcout->dutycycle3 = Ta;
            }
            else
            {
                T1 = -abc->a;
                T2 = -abc->c;
                    a_Reg = __builtin_mulus(period, T1);
                    T1 = __builtin_sacr(a_Reg,0);
                    a_Reg = __builtin_mulus(period, T2);
                    T2 = __builtin_sacr(a_Reg,0);
                    Tc = period-T1-T2;
                    Tc = Tc >> 1;
                    Tb = Tc + T1;
                    Ta = Tb + T2;
                pdcout->dutycycle1 = Ta;
                pdcout->dutycycle2 = Tc;
                pdcout->dutycycle3 = Tb;
            }
        }
        else
        {
            T1 = -abc->b;
            T2 = -abc->a;
                a_Reg = __builtin_mulus(period, T1);
                T1 = __builtin_sacr(a_Reg,0);
                a_Reg = __builtin_mulus(period, T2);
                T2 = __builtin_sacr(a_Reg,0);
                Tc = period-T1-T2;
                Tc = Tc >> 1;
                Tb = Tc + T1;
                Ta = Tb + T2;
            pdcout->dutycycle1 = Tc;
            pdcout->dutycycle2 = Tb;
            pdcout->dutycycle3 = Ta;
        }
    }

    CORCON = mcCorconSave;
    return 1;
}

static inline void MC_CalculateZeroSequenceModulation_InlineC(const MC_ABC_T *pabc_in, MC_ABC_T *pabc_out, int16_t min, int16_t max)
{
    const int16_t center_out = MC_UTIL_AverageS16(min, max);    
    const MC_minmax16_t minmax_in = MC_UTIL_MinMax3_S16(pabc_in->a, pabc_in->b, pabc_in->c);
    const int16_t center_in = MC_UTIL_AverageS16(minmax_in.min, minmax_in.max);
    pabc_out->a = MC_adjust_zero_sequence(pabc_in->a - center_in, center_out, min, max);
    pabc_out->b = MC_adjust_zero_sequence(pabc_in->b - center_in, center_out, min, max);
    pabc_out->c = MC_adjust_zero_sequence(pabc_in->c - center_in, center_out, min, max);
}

static inline uint16_t MC_CalculateSineCosine_InlineC_Ram( int16_t angle, MC_SINCOS_T *sincos )
{
    uint16_t remainder, index, y0, y1, delta, return_value;
    uint32_t result;

    return_value = 0;
    
    result = __builtin_muluu(128,angle);
    index = result >> 16;
    remainder = (uint16_t) result ;

    if(remainder == 0)
    {
        sincos->sin = MC_SineTableInRam[index];
        index = index+32;
        if (index > 127)
        {
            index = index - 128;
        }    
        sincos->cos = MC_SineTableInRam[index];
        return_value = 1;
    }
    else
    {

        y0 = MC_SineTableInRam[index];
        index = index+1;
        if (index > 127)
        {
            index = index - 128;
        }
        y1 = MC_SineTableInRam[index];
        delta = y1 - y0;
        result = __builtin_mulus(remainder,delta);
        sincos->sin = y0 + ( result >>16 );

        index = index+31;
        if (index > 127)
        {
            index = index - 128;
        }
        
        y0 = MC_SineTableInRam[index];
        index = index+1;
        if (index > 127)
        {
            index = index - 128;
        }
        y1 = MC_SineTableInRam[index];
        delta = y1 - y0;
        result = __builtin_mulus(remainder,delta);
        sincos->cos = y0 + ( result >>16);
        return_value = 2;
    }
    return  return_value ;
}

#ifdef __cplusplus 
    }
#endif

#endif 




