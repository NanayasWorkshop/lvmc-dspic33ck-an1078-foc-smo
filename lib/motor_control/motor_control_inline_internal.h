#ifndef _MOTOR_CONTROL_INLINE_INTERNAL_H_
#define _MOTOR_CONTROL_INLINE_INTERNAL_H_

#include <stdint.h>

static inline int16_t MC_adjust_zero_sequence(int16_t x, int16_t ofs_out, int16_t min, int16_t max)
{
    int16_t w;
    asm volatile (
        "    add     %[ofs_out], %[x], %[w]\n" 
        "    cpslt   %[min], %[w]\n"
        "    mov     %[min], %[w]\n"  
        "    btss    SR, #2\n"     
        "    cpslt   %[w], %[max]\n"
        "    mov     %[max], %[w]\n"
        : [w]"=&r"(w)
        : [x]"r"(x),
          [ofs_out]"r"(ofs_out),
          [min]"r"(min),
          [max]"r"(max)
    );
    return w;
}

#ifdef __cplusplus 
    extern "C" {
#endif

#endif
