#ifdef __XC16__  // See comments at the top of this header file
#include <xc.h>
#endif // __XC16__
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libq.h>  

#include "motor_control_noinline.h"

#include "general.h"   
#include "userparms.h"

#include "control.h"   
#include "smcpos.h"

#include "fdweak.h"

#include "hal/measure.h"


#include "hal/board_service.h"

#include "diagnostics.h"

#include "singleshunt.h"

volatile UGF_T uGF;

CTRL_PARM_T ctrlParm;
MOTOR_STARTUP_DATA_T motorStartUpData; 

volatile int16_t thetaElectrical = 0,thetaElectricalOpenLoop = 0;
uint16_t pwmPeriod;

MC_ALPHABETA_T valphabeta,ialphabeta;
MC_SINCOS_T sincosTheta;
MC_DQ_T vdq,idq;
MC_DUTYCYCLEOUT_T pwmDutycycle;
MC_ABC_T   vabc,iabc;

MC_PIPARMIN_T piInputIq;
MC_PIPARMOUT_T piOutputIq;
MC_PIPARMIN_T piInputId;
MC_PIPARMOUT_T piOutputId;
MC_PIPARMIN_T piInputOmega;
MC_PIPARMOUT_T piOutputOmega;



SMC smc1 = SMC_DEFAULTS;

uint16_t  trans_counter = 0;
int16_t Theta_error = 0;

int16_t PrevTheta = 0;	
int16_t AccumTheta = 0;
uint16_t AccumThetaCnt = 0;	

volatile uint16_t adcDataBuffer;
MCAPP_MEASURE_T measureInputs;
void InitControlParameters(void);
void DoControl(void);
void CalculateParkAngle(void);
void ResetParmeters(void);

int main ( void )
{
    Init_Peripherals();

    LED2 = 1;

    DiagnosticsInit();

    BoardServiceInit();

    CORCONbits.SATA = 1;
    CORCONbits.SATB = 1;
    CORCONbits.ACCSAT = 1;

    CORCONbits.SATA = 0;

    while(1)
    {
        InitControlParameters();

        SMCInit(&smc1);

        FWInit();

        ResetParmeters();
        
        while(1)
        {
            DiagnosticsStepMain();
            BoardService();
            if (IsPressed_Button1())
            {
                if (uGF.bits.RunMotor == 1)
                {
                    ResetParmeters();
					trans_counter = 0;
                    LED1 = 0;
                }
                else
                {
					EnablePWMOutputs();
                    uGF.bits.RunMotor = 1;
                    LED1 = 1;
                }

            }
                if (IsPressed_Button2())
                {
                    if ((uGF.bits.RunMotor == 1) && (uGF.bits.OpenLoop == 0))
                    {
                        uGF.bits.ChangeSpeed = !uGF.bits.ChangeSpeed;
                    }
                }
        }
    }

    while(1){}


void ResetParmeters(void)
{
	DisableADCInterrupt();
    
#ifdef SINGLE_SHUNT
    SingleShunt_InitializeParameters(&singleShuntParam);
    PWM_TRIGA = ADC_SAMPLING_POINT;
    PWM_TRIGB = LOOPTIME_TCY>>1;
    PWM_TRIGC = LOOPTIME_TCY-1;
    PWM_PHASE3 = MIN_DUTY;
    PWM_PHASE2 = MIN_DUTY;
    PWM_PHASE1 = MIN_DUTY;
#else
    PWM_TRIGA = ADC_SAMPLING_POINT;
#endif    
    PWM_PDC3 = MIN_DUTY;
    PWM_PDC2 = MIN_DUTY;
    PWM_PDC1 = MIN_DUTY;
	DisablePWMOutputs();
    
    uGF.bits.RunMotor = 0;
    ctrlParm.qVelRef = 0;
    uGF.bits.OpenLoop = 1;
    uGF.bits.ChangeSpeed = 0;
    uGF.bits.ChangeMode = 1;

    InitControlParameters();        
    SMCInit(&smc1);
    FWInit();
    MCAPP_MeasureCurrentInit(&measureInputs);
    MCAPP_MeasureAvgInit(&measureInputs.MOSFETTemperature,
            MOSFET_TEMP_AVG_FILTER_SCALE);

    ClearADCIF();
	adcDataBuffer = ClearADCIF_ReadADCBUF();
    EnableADCInterrupt();
}

void DoControl( void )
{
    volatile int16_t temp_qref_pow_q15;

    if (uGF.bits.OpenLoop)
    {
        if (uGF.bits.ChangeMode)
        {
            uGF.bits.ChangeMode = 0;

            ctrlParm.qVqRef = 0;
            ctrlParm.qVdRef = 0;
            
			smc1.Valpha = 0;
			smc1.Ealpha = 0;
			smc1.EalphaFinal = 0;
			smc1.Zalpha = 0;
			smc1.EstIalpha = 0;
			smc1.Vbeta = 0;
			smc1.Ebeta = 0;
			smc1.EbetaFinal = 0;
			smc1.Zbeta = 0;
			smc1.EstIbeta = 0;
			smc1.Ialpha = 0;
			smc1.IalphaError = 0;
			smc1.Ibeta = 0;
			smc1.IbetaError = 0;
			smc1.Theta = 0;
			smc1.Omega = 0;

            motorStartUpData.startupLock = 0;
            motorStartUpData.startupRamp = 0;

            #ifdef TUNING
                motorStartUpData.tuningAddRampup = 0;
                motorStartUpData.tuningDelayRampup = 0;
            #endif
        }
        ctrlParm.qVelRef = Q_CURRENT_REF_OPENLOOP;
        ctrlParm.qVqRef = ctrlParm.qVelRef;

        piInputIq.inMeasure = idq.q;
        piInputIq.inReference = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;

        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d = piOutputId.out;

    }
    else
    {
        if (uGF.bits.ChangeSpeed)
        {
            ctrlParm.targetSpeed = (__builtin_mulss(measureInputs.potValue,
                    MAXIMUMSPEED_ELECTR-NOMINALSPEED_ELECTR)>>15)+
                    NOMINALSPEED_ELECTR;
        }
        else
        {
            
            ctrlParm.targetSpeed = (__builtin_mulss(measureInputs.potValue,
                    NOMINALSPEED_ELECTR-ENDSPEED_ELECTR)>>15) +
                    ENDSPEED_ELECTR;
        }
        
        if (ctrlParm.speedRampCount < SPEEDREFRAMP_COUNT)
        {
           ctrlParm.speedRampCount++; 
        }
        else
        {
            ctrlParm.qDiff = ctrlParm.qVelRef - ctrlParm.targetSpeed;
            if (ctrlParm.qDiff < 0)
            {
                ctrlParm.qVelRef = ctrlParm.qVelRef+ctrlParm.qRefRamp;
            }
            else
            {
                ctrlParm.qVelRef = ctrlParm.qVelRef-ctrlParm.qRefRamp;
            }
            if (_Q15abs(ctrlParm.qDiff) < (ctrlParm.qRefRamp << 1))
            {
                ctrlParm.qVelRef = ctrlParm.targetSpeed;
            }
            ctrlParm.speedRampCount = 0;
        }
        #ifdef TUNING
            if (motorStartUpData.tuningDelayRampup > TUNING_DELAY_RAMPUP)
            {
                motorStartUpData.tuningDelayRampup = 0;
            }
            if ((motorStartUpData.tuningAddRampup < (MAXIMUMSPEED_ELECTR - ENDSPEED_ELECTR)) &&
                                                  (motorStartUpData.tuningDelayRampup == 0) )
            {
                motorStartUpData.tuningAddRampup++;
            }
            motorStartUpData.tuningDelayRampup++;
            ctrlParm.qVelRef = ENDSPEED_ELECTR +  motorStartUpData.tuningAddRampup;
        #endif

        if ( uGF.bits.ChangeMode )
        {
            uGF.bits.ChangeMode = 0;
            piInputOmega.piState.integrator = (int32_t)ctrlParm.qVqRef << 13;
            ctrlParm.qVelRef = ENDSPEED_ELECTR;
        }

        #ifndef	TORQUE_MODE
			piInputOmega.inMeasure = smc1.OmegaFltred;
            piInputOmega.inReference = ctrlParm.qVelRef;
            MC_ControllerPIUpdate_Assembly(piInputOmega.inReference,
                                           piInputOmega.inMeasure,
                                           &piInputOmega.piState,
                                           &piOutputOmega.out);
            ctrlParm.qVqRef = piOutputOmega.out;
        #else
            ctrlParm.qVqRef = ctrlParm.qVelRef;
        #endif

        ctrlParm.qVdRef=FieldWeakening(_Q15abs(ctrlParm.qVelRef));

        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d    = piOutputId.out;

        temp_qref_pow_q15 = (int16_t)(__builtin_mulss(piOutputId.out ,
                                                      piOutputId.out) >> 15);
        temp_qref_pow_q15 = Q15(MAX_VOLTAGE_VECTOR) - temp_qref_pow_q15;
        piInputIq.piState.outMax = _Q15sqrt(temp_qref_pow_q15);
        piInputIq.piState.outMin = -piInputIq.piState.outMax;

        piInputIq.inMeasure  = idq.q;
        piInputIq.inReference  = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;
    }
}

void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt()
{
#ifdef SINGLE_SHUNT 
    if (IFS4bits.PWM1IF ==1)
    {
        singleShuntParam.adcSamplePoint = 0;
        IFS4bits.PWM1IF = 0;
    }    

    switch(singleShuntParam.adcSamplePoint)
    {
        case SS_SAMPLE_BUS1:
            singleShuntParam.adcSamplePoint = 1;  
            singleShuntParam.Ibus1 = (int16_t)(ADCBUF_IBUS) - 
                                            measureInputs.current.offsetIbus;                        
        break;

        case SS_SAMPLE_BUS2:
            PWM_TRIGA = ADC_SAMPLING_POINT;
            singleShuntParam.adcSamplePoint = 0;
            singleShuntParam.Ibus2 = (int16_t)(ADCBUF_IBUS) - 
                                            measureInputs.current.offsetIbus;				
            ADCON3Lbits.SWCTRG = 1; 
        break;

        default:
        break;  
    }
#endif
    if ( uGF.bits.RunMotor )
    {
        if (singleShuntParam.adcSamplePoint == 0)
        {
            measureInputs.current.Ia = ADCBUF_IPHASE1;
            measureInputs.current.Ib = ADCBUF_IPHASE2;
            
#ifdef SINGLE_SHUNT
        
        SingleShunt_PhaseCurrentReconstruction(&singleShuntParam);
        MCAPP_MeasureCurrentCalibrate(&measureInputs);
        iabc.a = singleShuntParam.Ia;
        iabc.b = singleShuntParam.Ib;
#else
        MCAPP_MeasureCurrentCalibrate(&measureInputs);
        iabc.a = measureInputs.current.Ia;
        iabc.b = measureInputs.current.Ib;
#endif
        MC_TransformClarke_Assembly(&iabc,&ialphabeta);

        MC_TransformPark_Assembly(&ialphabeta,&sincosTheta,&idq);

        smc1.Ialpha = ialphabeta.alpha;
        smc1.Ibeta = ialphabeta.beta;
        smc1.Valpha = valphabeta.alpha;
        smc1.Vbeta = valphabeta.beta;
        SMC_Position_Estimation_Inline(&smc1);

        DoControl();
        CalculateParkAngle();
        if (uGF.bits.OpenLoop == 1)
        {
            thetaElectrical = thetaElectricalOpenLoop;
        }
        else
        {
            thetaElectrical = smc1.Theta + Theta_error;
        }

        MC_CalculateSineCosine_Assembly_Ram(thetaElectrical,&sincosTheta);
        MC_TransformParkInverse_Assembly(&vdq,&sincosTheta,&valphabeta);
        MC_TransformClarkeInverseSwappedInput_Assembly(&valphabeta,&vabc);
        
#ifdef SINGLE_SHUNT
       SingleShunt_CalculateSpaceVectorPhaseShifted(&vabc,pwmPeriod,&singleShuntParam);

       PWMDutyCycleSetDualEdge(&singleShuntParam.pwmDutycycle1,&singleShuntParam.pwmDutycycle2); 
#else
        MC_CalculateSpaceVectorPhaseShifted_Assembly(&vabc,pwmPeriod,
                                                    &pwmDutycycle);
        PWMDutyCycleSet(&pwmDutycycle);
        
#endif
        }
    }
        
    else
    {
        PWM_TRIGA = ADC_SAMPLING_POINT;
#ifdef SINGLE_SHUNT
        PWM_TRIGB = LOOPTIME_TCY>>1;
        PWM_TRIGC = LOOPTIME_TCY-1;
        singleShuntParam.pwmDutycycle1.dutycycle3 = MIN_DUTY;
        singleShuntParam.pwmDutycycle1.dutycycle2 = MIN_DUTY;
        singleShuntParam.pwmDutycycle1.dutycycle1 = MIN_DUTY;
        singleShuntParam.pwmDutycycle2.dutycycle3 = MIN_DUTY;
        singleShuntParam.pwmDutycycle2.dutycycle2 = MIN_DUTY;
        singleShuntParam.pwmDutycycle2.dutycycle1 = MIN_DUTY;
        PWMDutyCycleSetDualEdge(&singleShuntParam.pwmDutycycle1,
                &singleShuntParam.pwmDutycycle2);
#else
        pwmDutycycle.dutycycle3 = MIN_DUTY;
        pwmDutycycle.dutycycle2 = MIN_DUTY;
        pwmDutycycle.dutycycle1 = MIN_DUTY;
        PWMDutyCycleSet(&pwmDutycycle);
#endif
        
    }
    if (singleShuntParam.adcSamplePoint == 0)
    {
        if (uGF.bits.RunMotor == 0)
        {
            measureInputs.current.Ia = ADCBUF_IPHASE1;
            measureInputs.current.Ib = ADCBUF_IPHASE2; 
            measureInputs.current.Ibus = ADCBUF_IBUS;
        }
        if (MCAPP_MeasureCurrentOffsetStatus(&measureInputs) == 0)
        {
            MCAPP_MeasureCurrentOffset(&measureInputs);
        }
        else
        {
            BoardServiceStepIsr();
        }
        measureInputs.potValue = (int16_t)( ADCBUF_SPEED_REF>>1);
        measureInputs.dcBusVoltage = (int16_t)( ADCBUF_VBUS>>1);
        MCAPP_MeasureTemperature(&measureInputs,(int16_t)(ADCBUF_MOSFET_TEMP>>1));
        DiagnosticsStepIsr();
    }
    adcDataBuffer = ClearADCIF_ReadADCBUF();
    ClearADCIF();
}

void CalculateParkAngle(void)
{
    if (uGF.bits.OpenLoop)
    {
        if (motorStartUpData.startupLock < LOCK_TIME)
        {
            motorStartUpData.startupLock += 1;
        }
        else if (motorStartUpData.startupRamp < END_SPEED)
        {
            motorStartUpData.startupRamp += OPENLOOP_RAMPSPEED_INCREASERATE;
        }
        else 
        {
            #ifndef OPEN_LOOP_FUNCTIONING
                uGF.bits.ChangeMode = 1;
                uGF.bits.OpenLoop = 0;
            #endif
            Theta_error = thetaElectricalOpenLoop - smc1.Theta;
        }
        thetaElectricalOpenLoop += (int16_t)(motorStartUpData.startupRamp >> 
                                    STARTUPRAMP_THETA_OPENLOOP_SCALER);

    }
    else 
    {
        if ( (abs(Theta_error) > _0_05DEG)&&(trans_counter == 0))
        {
            if (Theta_error < 0)
                Theta_error += _0_05DEG;
            else
                Theta_error -= _0_05DEG;
        }       
    }
}

void InitControlParameters(void)
{

    ctrlParm.qRefRamp = SPEEDREFRAMP;
    ctrlParm.speedRampCount = SPEEDREFRAMP_COUNT;
    trans_counter = 0;

    pwmPeriod = LOOPTIME_TCY;

    piInputId.piState.kp = D_CURRCNTR_PTERM;
    piInputId.piState.ki = D_CURRCNTR_ITERM;
    piInputId.piState.kc = D_CURRCNTR_CTERM;
    piInputId.piState.outMax = D_CURRCNTR_OUTMAX;
    piInputId.piState.outMin = -piInputId.piState.outMax;
    piInputId.piState.integrator = 0;
    piOutputId.out = 0;

    piInputIq.piState.kp = Q_CURRCNTR_PTERM;
    piInputIq.piState.ki = Q_CURRCNTR_ITERM;
    piInputIq.piState.kc = Q_CURRCNTR_CTERM;
    piInputIq.piState.outMax = Q_CURRCNTR_OUTMAX;
    piInputIq.piState.outMin = -piInputIq.piState.outMax;
    piInputIq.piState.integrator = 0;
    piOutputIq.out = 0;

    piInputOmega.piState.kp = SPEEDCNTR_PTERM;
    piInputOmega.piState.ki = SPEEDCNTR_ITERM;
    piInputOmega.piState.kc = SPEEDCNTR_CTERM;
    piInputOmega.piState.outMax = SPEEDCNTR_OUTMAX;
    piInputOmega.piState.outMin = -piInputOmega.piState.outMax;
    piInputOmega.piState.integrator = 0;
    piOutputOmega.out = 0;
}

void __attribute__((__interrupt__,no_auto_psv)) _PWMInterrupt()
{
    ResetParmeters();
    ClearPWMPCIFault();
    ClearPWMIF(); 
}