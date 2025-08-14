#ifndef USERPARMS_H
#define USERPARMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <xc.h>

#define PWMFREQUENCY_HZ         20000

#define LOOPTIME_SEC            0.00005

#undef TUNING

#ifdef TUNING
    #define TUNING_DELAY_RAMPUP   0xF
#endif

#undef OPEN_LOOP_FUNCTIONING

#undef TORQUE_MODE
   
#define SINGLE_SHUNT  
    
#define INTERNAL_OPAMP_CONFIG

/****************************** Motor Parameters ******************************/
/********************  support xls file definitions begin *********************/
/* The following values are given in the xls attached file */
#define NOPOLESPAIRS 5
#define END_SPEED_RPM 500
#define NOMINAL_SPEED_RPM    2000
#define MAXIMUM_SPEED_RPM    3500
#define FW_NOMINAL_SPEED_RPM 2000
#define NORM_CURRENT_CONST     0.000671
#define NORM_LSDTBASE 9738
#define NORM_LSDTBASE_SCALINGFACTOR 8
#define NORM_RS  27503
#define NORM_RS_SCALINGFACTOR 2

/**********************  support xls file definitions end *********************/


#define NORM_CURRENT(current_real) (Q15(current_real/NORM_CURRENT_CONST/32768))

#define LOCK_TIME 4000
#define END_SPEED_RPM 500
#define OPENLOOP_RAMPSPEED_INCREASERATE 10
#define Q_CURRENT_REF_OPENLOOP NORM_CURRENT(1.0) 
#define Q15_OVER_CURRENT_THRESHOLD NORM_CURRENT(3.0)

#define MAXIMUMSPEED_ELECTR MAXIMUM_SPEED_RPM*NOPOLESPAIRS
#define NOMINALSPEED_ELECTR NOMINAL_SPEED_RPM*NOPOLESPAIRS

#define END_SPEED (END_SPEED_RPM * NOPOLESPAIRS * LOOPTIME_SEC * 65536 / 60.0)*1024
#define ENDSPEED_ELECTR END_SPEED_RPM*NOPOLESPAIRS

#define SPEEDREFRAMP   Q15(0.00003)

#define SPEEDREFRAMP_COUNT   3
#define D_CURRCNTR_PTERM       Q15(0.02)     
#define D_CURRCNTR_ITERM       Q15(0.001)   
#define D_CURRCNTR_CTERM       Q15(0.999)
#define D_CURRCNTR_OUTMAX      0x7FFF

#define Q_CURRCNTR_PTERM       Q15(0.02)     
#define Q_CURRCNTR_ITERM       Q15(0.001)   
#define Q_CURRCNTR_CTERM       Q15(0.999)
#define Q_CURRCNTR_OUTMAX      0x7FFF

#define SPEEDCNTR_PTERM        Q15(0.5)
#define SPEEDCNTR_ITERM        Q15(0.005)
#define SPEEDCNTR_CTERM        Q15(0.999)
#define SPEEDCNTR_OUTMAX       0x5000

//***********************SMC Params*********************************************//
#define LOOPTIMEINSEC (1.0/PWMFREQUENCY_HZ)
#define SPEEDLOOPFREQ	1000
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ)
#define IRP_PERCALC (uint16_t)(SPEEDLOOPTIME/LOOPTIMEINSEC)	
#define TRANSITION_STEPS   IRP_PERCALC/4

#define SMCGAIN			0.85
#define MAXLINEARSMC    0.005

#define STARTUPRAMP_THETA_OPENLOOP_SCALER       10

#define MAX_VOLTAGE_VECTOR                      0.98

#define SMO_SPEED_EST_MULTIPLIER Q15(0.9155273)

#define THETA_FILTER_CNST   Q15(0.104719*LOOPTIME_SEC*32768.0) 

#define IDREF_BASESPEED         NORM_CURRENT(0.0)

#define SPEED_INDEX_CONST 10

#define FWONSPEED FW_NOMINAL_SPEED_RPM*NOPOLESPAIRS

/* the following values indicate the d-current variation with speed 
 please consult app note for details on tuning */
#define	IDREF_SPEED0	NORM_CURRENT(0)     /* up to 2800 RPM */
#define	IDREF_SPEED1	NORM_CURRENT(-0.7)  /* ~2950 RPM */
#define	IDREF_SPEED2	NORM_CURRENT(-0.9)  /* ~3110 RPM */
#define	IDREF_SPEED3	NORM_CURRENT(-1.0)  /* ~3270 RPM */
#define	IDREF_SPEED4	NORM_CURRENT(-1.4)  /* ~3430 RPM */
#define	IDREF_SPEED5	NORM_CURRENT(-1.7)  /* ~3600 RPM */
#define	IDREF_SPEED6	NORM_CURRENT(-2.0)  /* ~3750 RPM */
#define	IDREF_SPEED7	NORM_CURRENT(-2.1)  /* ~3910 RPM */
#define	IDREF_SPEED8	NORM_CURRENT(-2.2)  /* ~4070 RPM */
#define	IDREF_SPEED9	NORM_CURRENT(-2.25) /* ~4230 RPM */
#define	IDREF_SPEED10	NORM_CURRENT(-2.3)  /* ~4380 RPM */
#define	IDREF_SPEED11	NORM_CURRENT(-2.35) /* ~4550 RPM */
#define	IDREF_SPEED12	NORM_CURRENT(-2.4)  /* ~4700 RPM */
#define	IDREF_SPEED13	NORM_CURRENT(-2.45) /* ~4860 RPM */
#define	IDREF_SPEED14	NORM_CURRENT(-2.5)  /* ~5020 RPM */
#define	IDREF_SPEED15	NORM_CURRENT(-2.5)  /* ~5180 RPM */
#define	IDREF_SPEED16	NORM_CURRENT(-2.5)  /* ~5340 RPM */
#define	IDREF_SPEED17	NORM_CURRENT(-2.5)  /* ~5500 RPM */


#if (FW_NOMINAL_SPEED_RPM < NOMINAL_SPEED_RPM)
	#error FIELDWEAKSPEEDRPM must be greater than NOMINALSPEEDINRPM for field weakening.
	#error if application does not require Field Weakening, set FIELDWEAKSPEEDRPM value
	#error equal to NOMINALSPEEDINRPM
#elif (((FW_NOMINAL_SPEED_RPM*NOPOLESPAIRS*2)/(60*SPEEDLOOPFREQ)) >= 1)
		#error FIELDWEAKSPEEDRPM will generate an Omega value greater than 1 which is the
		#error maximum in Q15 format. Reduce FIELDWEAKSPEEDRPM value, or increase speed
		#error control loop frequency, SPEEDLOOPFREQ
#endif

#ifdef __cplusplus
}
#endif

#endif /* __USERPARMS_H */
