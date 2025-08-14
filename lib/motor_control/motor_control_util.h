#ifndef _MOTOR_CONTROL_UTIL_H_
#define	_MOTOR_CONTROL_UTIL_H_

#include <stdint.h>

#include "motor_control_dsp.h"    

#if __XC16_VERSION__ < 1026
#include <xc.h>
#endif


#ifdef	__cplusplus
extern "C" {
#endif

inline static int16_t MC_UTIL_mulus16(uint16_t a, int16_t b)
{
    return __builtin_mulus(a,b) >> 16;
}

inline static int16_t MC_UTIL_AverageS16(int16_t a, int16_t b)
{
    return (int16_t)((((int32_t)a) + b) >> 1);
}

inline static MC_minmax16_t MC_UTIL_MinMax3_S16(int16_t a, int16_t b, int16_t c)
{
    asm (
        "    cpslt   %[a], %[b]\n"
        "    exch    %[a], %[b]\n"
        "    cpslt   %[a], %[c]\n"
        "    exch    %[a], %[c]\n"
        "    cpslt   %[b], %[c]\n"
        "    exch    %[b], %[c]\n"
        : [a]"+r"(a),
          [b]"+r"(b),
          [c]"+r"(c)
    );

    MC_minmax16_t result;
    result.min = a;
    result.max = c;
    return result;
}

inline static int32_t MC_UTIL_readAccA32()
{
#if __XC16_VERSION__ >= 1030
    return __builtin_sacd(a_Reg, 0);
#elif __XC16_VERSION__ >= 1026
    const int32_t tmp = __builtin_sacd(a_Reg, 0);
    asm volatile ("");
    return tmp;
#else
    int32_t result;
    asm volatile ("" : "+w"(a_Reg):); 
    result = ACCAH;
    result <<= 16;
    result |= ACCAL; 
    return result;
#endif
}

inline static void MC_UTIL_writeAccB32(int32_t input)
{
#if __XC16_VERSION__ >= 1030
    b_Reg = __builtin_lacd(input, 0);
#elif __XC16_VERSION__ >= 1026
    const int32_t tmp = input;
    asm volatile ("" :: "r"(tmp)); 
    b_Reg = __builtin_lacd(tmp, 0);
#else
    uint32_t temp_dword;
    uint16_t temp_word;
    temp_dword = 0xFFFF0000 & input;
    temp_dword = temp_dword >> 16;
    temp_word = (uint16_t)temp_dword;
    b_Reg = __builtin_lac(temp_word, 0);
    asm volatile ("" : "+w"(b_Reg):); 
    temp_word = (uint16_t)(0xFFFF & input);
    ACCBL = temp_word;
#endif

}



#ifdef	__cplusplus
}
#endif

#endif