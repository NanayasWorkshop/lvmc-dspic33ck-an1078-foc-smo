#include "fdweak.h"
#include "general.h"
#include "smcpos.h"

tFdWeakParm	FdWeakParm;

int16_t FieldWeakening(int16_t qMotorSpeed)
{
    int16_t iTempInt1, iTempInt2;

    if (qMotorSpeed <= FdWeakParm.qFwOnSpeed) 
    {
        FdWeakParm.qIdRef = FdWeakParm.qFwCurve[0];
    } 
    else 
    {
        FdWeakParm.qIndex = (qMotorSpeed - FdWeakParm.qFwOnSpeed) >> SPEED_INDEX_CONST;

        iTempInt1 = FdWeakParm.qFwCurve[FdWeakParm.qIndex] -
                    FdWeakParm.qFwCurve[FdWeakParm.qIndex + 1];
        iTempInt2 = (FdWeakParm.qIndex << SPEED_INDEX_CONST) +
                    FdWeakParm.qFwOnSpeed;
        iTempInt2 = qMotorSpeed - iTempInt2;

        FdWeakParm.qIdRef = FdWeakParm.qFwCurve[FdWeakParm.qIndex]-
                (int16_t) (__builtin_mulss(iTempInt1, iTempInt2) >> SPEED_INDEX_CONST);
    }
    return FdWeakParm.qIdRef;
}

void FWInit(void)
{
    /* Field Weakening constant for constant torque range */
    FdWeakParm.qIdRef = IDREF_BASESPEED;
    FdWeakParm.qFwOnSpeed = FWONSPEED;

    FdWeakParm.qFwCurve[0] = IDREF_SPEED0;
    FdWeakParm.qFwCurve[1] = IDREF_SPEED1;
    FdWeakParm.qFwCurve[2] = IDREF_SPEED2;
    FdWeakParm.qFwCurve[3] = IDREF_SPEED3;
    FdWeakParm.qFwCurve[4] = IDREF_SPEED4;
    FdWeakParm.qFwCurve[5] = IDREF_SPEED5;
    FdWeakParm.qFwCurve[6] = IDREF_SPEED6;
    FdWeakParm.qFwCurve[7] = IDREF_SPEED7;
    FdWeakParm.qFwCurve[8] = IDREF_SPEED8;
    FdWeakParm.qFwCurve[9] = IDREF_SPEED9;
    FdWeakParm.qFwCurve[10] = IDREF_SPEED10;
    FdWeakParm.qFwCurve[11] = IDREF_SPEED11;
    FdWeakParm.qFwCurve[12] = IDREF_SPEED12;
    FdWeakParm.qFwCurve[13] = IDREF_SPEED13;
    FdWeakParm.qFwCurve[14] = IDREF_SPEED14;
    FdWeakParm.qFwCurve[15] = IDREF_SPEED15;
    FdWeakParm.qFwCurve[16] = IDREF_SPEED16;
    FdWeakParm.qFwCurve[17] = IDREF_SPEED17;
}
