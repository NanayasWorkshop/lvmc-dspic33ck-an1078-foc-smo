#ifndef __GENERAL_H
#define __GENERAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <libq.h>

#define Q15(Float_Value)	\
((Float_Value < 0.0) ? (int16_t)(32768 * (Float_Value) - 0.5) \
: (int16_t)(32767 * (Float_Value) + 0.5))

 #define _0_05DEG 9	// The value for 0.05 degrees is converted
					// to Q15 as follows:
					// .05 * 32768 / 180 = 9.1, approx 9.   



#ifdef __cplusplus
    }
#endif

#endif
