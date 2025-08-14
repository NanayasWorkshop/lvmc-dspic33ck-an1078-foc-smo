#ifndef FdWeak_H
#define FdWeak_H

#include "userparms.h"
#include "smcpos.h"
#include "general.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int16_t	qK1;            // Nominal speed value
	int16_t	qIdRef;
	int16_t	qFwOnSpeed;
	int16_t	qFwActiv;
	int16_t	qIndex;
	int16_t	qFWPercentage;
	int16_t	qInterpolPortion;
    int16_t	qFwCurve[18];	// Curve for magnetizing current
    } tFdWeakParm;
extern tFdWeakParm FdWeakParm;
int16_t FieldWeakening(int16_t qMotorSpeed );
void FWInit (void);

#ifdef __cplusplus
}
#endif

#endif




