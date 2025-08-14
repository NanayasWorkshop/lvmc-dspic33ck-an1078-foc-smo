#ifndef _MOTOR_CONTROL_DECLARATIONS_H_    
#define _MOTOR_CONTROL_DECLARATIONS_H_

#include <stdint.h>
#include "motor_control_types.h"

#if __XC16_VERSION__ > 1011    
#define MC_ATTRB
#else
#define MC_ATTRB
#endif

#ifdef __cplusplus  
    extern "C" {
#endif

uint16_t MC_ATTRB MC_CalculateSineCosine_Assembly_Ram(int16_t angle,
                                                      MC_SINCOS_T *pSinCos);

uint16_t MC_ATTRB MC_TransformParkInverse_Assembly( const MC_DQ_T *pDQ,
                                                              const MC_SINCOS_T *pSinCos,
                                                              MC_ALPHABETA_T *pAlphaBeta);

uint16_t MC_ATTRB MC_TransformClarkeInverseSwappedInput_Assembly( const MC_ALPHABETA_T *pAlphaBeta,
                                                                  MC_ABC_T *pABC);

uint16_t MC_ATTRB MC_TransformClarkeInverse_Assembly( const MC_ALPHABETA_T *pAlphaBeta,
                                                                     MC_ABC_T *pABC);

void MC_ATTRB MC_TransformClarkeInverseNoAccum_Assembly( const MC_ALPHABETA_T *pAlphaBeta,
                                                                    MC_ABC_T *pABC);

uint16_t MC_ATTRB MC_CalculateSpaceVectorPhaseShifted_Assembly( const MC_ABC_T *pABC,
                                                                uint16_t iPwmPeriod,
                                                                MC_DUTYCYCLEOUT_T *pDutyCycleOut);

uint16_t MC_ATTRB MC_CalculateSpaceVector_Assembly( const MC_ABC_T *pABC,
                                                               uint16_t iPwmPeriod,
                                                               MC_DUTYCYCLEOUT_T *pDutyCycleOut);

uint16_t MC_ATTRB MC_TransformClarke_Assembly( const MC_ABC_T *pABC,
                                               MC_ALPHABETA_T *pAlphaBeta);

uint16_t MC_ATTRB MC_TransformPark_Assembly( const MC_ALPHABETA_T *pAlphaBeta,
                                             const MC_SINCOS_T *pSinCos,
                                             MC_DQ_T *pDQ);

uint16_t MC_ATTRB MC_ControllerPIUpdate_Assembly(int16_t inReference,
                                                 int16_t inMeasure,
                                                 MC_PISTATE_T *pPIState,
                                                 int16_t *pPIParmOutput);

#ifdef __cplusplus 
    }
#endif
#endif 
