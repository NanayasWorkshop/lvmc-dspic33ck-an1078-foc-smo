#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Control Parameter data type */
typedef struct
{
    int16_t   qVelRef;
    int16_t   qVdRef;
    int16_t   qVqRef;
    int16_t   qRefRamp;
    int16_t   qDiff;
    int16_t  targetSpeed;
    int16_t   speedRampCount; 
} CTRL_PARM_T;

/* Motor Parameter data type */
typedef struct
{
    uint32_t startupRamp;
    uint16_t startupLock;
    uint16_t tuningAddRampup;	
    uint16_t tuningDelayRampup;
} MOTOR_STARTUP_DATA_T;

/* General system flag data type */
typedef union
{
    struct
    {
        unsigned RunMotor:1;
        unsigned OpenLoop:1;
        unsigned ChangeMode:1;
        unsigned ChangeSpeed:1;
        unsigned    :12;
    } bits;
    uint16_t Word;
} UGF_T;

extern CTRL_PARM_T ctrlParm;
extern MOTOR_STARTUP_DATA_T motorStartUpData;
extern MC_ALPHABETA_T valphabeta,ialphabeta;
extern MC_ALPHABETA_T valphabeta,ialphabeta;
extern MC_SINCOS_T sincosTheta;
extern MC_DQ_T vdq,idq;
extern MC_DUTYCYCLEOUT_T pwmDutycycle;
extern MC_ABC_T   vabc,iabc;

#ifdef __cplusplus
}
#endif

#endif /* __CONTORL_H */
