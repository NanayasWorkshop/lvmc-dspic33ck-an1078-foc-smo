#ifndef _MOTOR_CONTROL_INLINE_DECLARATIONS_H_    
#define _MOTOR_CONTROL_INLINE_DECLARATIONS_H_

#include <stdint.h>
#include "motor_control_types.h"

#if __XC16_VERSION__ > 1011  
#define MC_ATTRB_INLINE
#else
#define MC_ATTRB_INLINE
#endif

#ifdef __cplusplus 
    extern "C" {
#endif

static inline uint16_t MC_ATTRB_INLINE MC_CalculateSineCosine_InlineC_Ram(int16_t angle,
                                                                   MC_SINCOS_T *pSinCos);
static inline uint16_t MC_ATTRB_INLINE MC_TransformClarkeInverseSwappedInput_InlineC( const MC_ALPHABETA_T *pAlphaBeta,
                                                                              MC_ABC_T *pABC);
static inline uint16_t MC_ATTRB_INLINE MC_TransformClarkeInverse_InlineC( const MC_ALPHABETA_T *pAlphaBeta,
                                                                              MC_ABC_T *pABC);
static inline void MC_ATTRB_INLINE MC_TransformClarkeInverseNoAccum_InlineC( const MC_ALPHABETA_T *pAlphaBeta,
                                                                              MC_ABC_T *pABC);
static inline uint16_t MC_ATTRB_INLINE MC_CalculateSpaceVectorPhaseShifted_InlineC( const MC_ABC_T *pABC,
                                                                            uint16_t iPwmPeriod,
                                                                            MC_DUTYCYCLEOUT_T *pDutyCycleOut);
static inline void MC_ATTRB_INLINE MC_CalculateZeroSequenceModulation_InlineC(const MC_ABC_T *pabc_in, MC_ABC_T *pabc_out, int16_t min, int16_t max);
static inline uint16_t MC_ATTRB_INLINE MC_TransformClarke_InlineC( const  MC_ABC_T *pABC,
                                                           MC_ALPHABETA_T *pAlphaBeta);
static inline void MC_ATTRB_INLINE MC_TransformClarkeABC_InlineC( const MC_ABC_T *abc, MC_ALPHABETA_T *alphabeta);
static inline uint16_t MC_ATTRB_INLINE MC_TransformPark_InlineC(const MC_ALPHABETA_T *alphabeta,
                                                                const MC_SINCOS_T *sincos, 
                                                                MC_DQ_T *dq);
static inline uint16_t MC_ATTRB_INLINE MC_TransformParkInverse_InlineC(const MC_DQ_T *dq, const MC_SINCOS_T *sincos,
                                                                        MC_ALPHABETA_T *alphabeta);
static inline uint16_t MC_ATTRB_INLINE MC_ControllerPIUpdate_InlineC(int16_t inReference,
                                                              int16_t inMeasure,
                                                              MC_PISTATE_T *pPIState,
                                                              int16_t *pPIParmOutput);

#ifdef __cplusplus  
    }
#endif
#endif 