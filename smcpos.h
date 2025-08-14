#ifndef smcpos_H
#define smcpos_H

#include "userparms.h"

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
typedef struct	{
				int16_t  Valpha;   		// Input: Stationary alfa-axis stator voltage
                int16_t  Ealpha;   		// Variable: Stationary alfa-axis back EMF
				int16_t  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
                int16_t  Zalpha;      	// Output: Stationary alfa-axis sliding control
                int16_t  Gsmopos;    	// Parameter: Motor dependent control gain
                int16_t  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current
                int16_t  Fsmopos;    	// Parameter: Motor dependent plant matrix
                int16_t  Vbeta;   		// Input: Stationary beta-axis stator voltage
                int16_t  Ebeta;  		// Variable: Stationary beta-axis back EMF
				int16_t  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
                int16_t  Zbeta;      	// Output: Stationary beta-axis sliding control
                int16_t  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current
                int16_t  Ialpha;  		// Input: Stationary alfa-axis stator current
                int16_t  IalphaError; 	// Variable: Stationary alfa-axis current error
                int16_t  Kslide;     	// Parameter: Sliding control gain
                int16_t  MaxSMCError;  	// Parameter: Maximum current error for linear SMC
                int16_t  Ibeta;  		// Input: Stationary beta-axis stator current
                int16_t  IbetaError;  	// Variable: Stationary beta-axis current error
                int16_t  Kslf;       	// Parameter: Sliding control filter gain
                int16_t  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
                int16_t  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
				int16_t  ThetaOffset;	// Output: Offset used to compensate rotor angle
                int16_t  Theta;			// Output: Compensated rotor angle 
				int16_t  Omega;     	// Output: Rotor speed
				int16_t  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
				} SMC;

typedef SMC *SMC_handle;

typedef struct
{
    int16_t qDeltaT;
    int16_t qRho;
    int32_t qRhoStateVar;
    int16_t qOmegaMr;
    int16_t qLastIalpha;
    int16_t qLastIbeta;
    int16_t qDIalpha;
    int16_t qDIbeta;
    int16_t qEsa;
    int16_t qEsb;
    int16_t qEsd;
    int16_t qEsq;
    int16_t qDiCounter;
    int16_t qVIndalpha;
    int16_t qVIndbeta;
    int16_t qEsdf;
    int32_t qEsdStateVar;
    int16_t qEsqf;
    int32_t qEsqStateVar;
    int16_t qKfilterEsdq;
    int16_t qVelEstim;
    int16_t qVelEstimFilterK;
    int32_t qVelEstimStateVar;
    int16_t qLastValpha;
    int16_t qLastVbeta;
    int16_t qDIlimitLS;
    int16_t qDIlimitHS;
    int16_t qLastIalphaHS[8];
    int16_t qLastIbetaHS[8];
    int16_t qRhoOffset;

} ESTIM_PARM_T;

typedef struct
{
    int16_t qRs;
    int16_t qLsDt;
    int16_t qLsDtBase;
    int16_t qInvKFi;
    int16_t qInvKFiBase;
} MOTOR_ESTIM_PARM_T;

extern ESTIM_PARM_T estimator;
extern MOTOR_ESTIM_PARM_T motorParm;


#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

#define THETA_AT_ALL_SPEED 90
#define THETA_ALL (uint16_t)(THETA_AT_ALL_SPEED / 180.0 * 32768.0)
#define CONSTANT_PHASE_SHIFT THETA_ALL

#define PUSHCORCON()  {__asm__ volatile ("push CORCON");}
#define POPCORCON()   {__asm__ volatile ("pop CORCON");}
#define _PI 3.1416

void SMCInit(SMC_handle);
void SMC_Position_Estimation_Inline(SMC_handle);
void CalcEstI(void);
void CalcIError(void);
void CalcZalpha(void);
void CalcZbeta(void);
void CalcBEMF(void);
void CalcOmegaFltred(void);
int16_t FracMpy(int16_t mul_1, int16_t mul_2);
int16_t FracDiv(int16_t num_1, int16_t den_1);
int16_t atan2CORDIC(int16_t, int16_t);

extern uint16_t  trans_counter;

extern int16_t PrevTheta;

extern int16_t AccumTheta;

extern uint16_t AccumThetaCnt;

#ifdef __cplusplus
}
#endif

#endif
