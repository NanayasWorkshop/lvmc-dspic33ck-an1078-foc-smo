#include <stdint.h>
#include <stdbool.h>
#include "board_service.h"
#include "general.h"
#include "userparms.h"
#include "cmp.h"
#include "pwm.h"

BUTTON_T buttonStartStop;
BUTTON_T buttonSpeedHalfDouble;
uint16_t boardServiceISRCounter = 0;

void BoardServiceInit(void);
void BoardServiceStepIsr(void);
void BoardService(void);
void EnablePWMOutputs(void);
void DisablePWMOutputs(void);
void ClearPWMPCIFault(void);

bool IsPressed_Button1(void);
bool IsPressed_Button2(void);

void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *,MC_DUTYCYCLEOUT_T *);
void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *);
void pwmDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *,uint16_t,uint16_t);

static void ButtonGroupInitialize(void);
static void ButtonScan(BUTTON_T * ,bool);

bool IsPressed_Button1(void)
{
    if (buttonStartStop.status)
    {
        buttonStartStop.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool IsPressed_Button2(void)
{
    if (buttonSpeedHalfDouble.status)
    {
        buttonSpeedHalfDouble.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

void BoardServiceStepIsr(void)
{
    if (boardServiceISRCounter <  BOARD_SERVICE_TICK_COUNT)
    {
        boardServiceISRCounter += 1;
    }
}
void BoardService(void)
{
    if (boardServiceISRCounter ==  BOARD_SERVICE_TICK_COUNT)
    {
        ButtonScan(&buttonStartStop,BUTTON_START_STOP);

        ButtonScan(&buttonSpeedHalfDouble,BUTTON_SPEED_HALF_DOUBLE);

        boardServiceISRCounter = 0;
    }
}
void BoardServiceInit(void)
{
    ButtonGroupInitialize();
    boardServiceISRCounter = BOARD_SERVICE_TICK_COUNT;
}

void ButtonScan(BUTTON_T *pButton,bool button) 
{
    if (button == true) 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->debounceCount--;
            pButton->state = BUTTON_DEBOUNCE;
        }
    } 
    else 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->state = BUTTON_NOT_PRESSED;
        } 
        else 
        {
            pButton->state = BUTTON_PRESSED;
            pButton->status = true;
        }
        pButton->debounceCount = 0;
    }
}
void ButtonGroupInitialize(void)
{
    buttonStartStop.state = BUTTON_NOT_PRESSED;
    buttonStartStop.debounceCount = 0;
    buttonStartStop.state = false; 
    buttonSpeedHalfDouble.state = BUTTON_NOT_PRESSED;
    buttonSpeedHalfDouble.debounceCount = 0;
    buttonSpeedHalfDouble.state = false;	  
}

void Init_Peripherals(void)
{                
    InitOscillator();
    
    SetupGPIOPorts();  
    
    uint16_t cmpReference = 0;
    CMP_Initialize();
    CMP1_ModuleEnable(true);
    cmpReference = (uint16_t)(__builtin_mulss(Q15_OVER_CURRENT_THRESHOLD,2047)>>15);
    cmpReference = cmpReference + 2048; 
    CMP1_ReferenceSet(cmpReference);
    
    InitPWMGenerators();
    
    InitializeADCs();
}

void EnablePWMOutputs(void)
{
    PG4IOCONLbits.OVRENH = 0; 
    PG4IOCONLbits.OVRENL = 0; 
    PG2IOCONLbits.OVRENH = 0;
    PG2IOCONLbits.OVRENL = 0; 
    PG1IOCONLbits.OVRENH = 0;  
    PG1IOCONLbits.OVRENL = 0;   
}

void DisablePWMOutputs(void)
{
    PG4IOCONLbits.OVRDAT = 0;
    PG2IOCONLbits.OVRDAT = 0; 
    PG1IOCONLbits.OVRDAT = 0;  
    PG4IOCONLbits.OVRENH = 1; 
    PG4IOCONLbits.OVRENL = 1; 
    PG2IOCONLbits.OVRENH = 1;
    PG2IOCONLbits.OVRENL = 1; 
    PG1IOCONLbits.OVRENH = 1;  
    PG1IOCONLbits.OVRENL = 1;     
}

void ClearPWMPCIFault(void)
{
    PG1FPCILbits.SWTERM = 1;
    PG2FPCILbits.SWTERM = 1;
    PG4FPCILbits.SWTERM = 1;
}

void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *pPwmDutycycle)
{
    pwmDutyCycleLimitCheck(pPwmDutycycle,(DDEADTIME>>1),(LOOPTIME_TCY - (DDEADTIME>>1)));  
    PWM_PDC3 = pPwmDutycycle->dutycycle3;
    PWM_PDC2 = pPwmDutycycle->dutycycle2;
    PWM_PDC1 = pPwmDutycycle->dutycycle1;
}

void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *pPwmDutycycle1,MC_DUTYCYCLEOUT_T *pPwmDutycycle2)
{
    pwmDutyCycleLimitCheck(pPwmDutycycle1,(DDEADTIME>>1),(LOOPTIME_TCY - (DDEADTIME>>1)));
    
    PWM_PHASE3 = pPwmDutycycle1->dutycycle3 + (DDEADTIME>>1);
    PWM_PHASE2 = pPwmDutycycle1->dutycycle2 + (DDEADTIME>>1);
    PWM_PHASE1 = pPwmDutycycle1->dutycycle1 + (DDEADTIME>>1);
    
    pwmDutyCycleLimitCheck(pPwmDutycycle2,(DDEADTIME>>1),(LOOPTIME_TCY - (DDEADTIME>>1)));
    
    PWM_PDC3 = pPwmDutycycle2->dutycycle3 - (DDEADTIME>>1);
    PWM_PDC2 = pPwmDutycycle2->dutycycle2 - (DDEADTIME>>1);
    PWM_PDC1 = pPwmDutycycle2->dutycycle1 - (DDEADTIME>>1);
}

void pwmDutyCycleLimitCheck (MC_DUTYCYCLEOUT_T *pPwmDutycycle,uint16_t min,uint16_t max)
{
    if (pPwmDutycycle->dutycycle1 < min)
    {
        pPwmDutycycle->dutycycle1 = min;
    }
    else if (pPwmDutycycle->dutycycle1 > max)
    {
        pPwmDutycycle->dutycycle1 = max;
    }
    
    if (pPwmDutycycle->dutycycle2 < min)
    {
        pPwmDutycycle->dutycycle2 = min;
    }
    else if (pPwmDutycycle->dutycycle2 > max)
    {
        pPwmDutycycle->dutycycle2 = max;
    }
    
    if (pPwmDutycycle->dutycycle3 < min)
    {
        pPwmDutycycle->dutycycle3 = min;
    }
    else if (pPwmDutycycle->dutycycle3 > max)
    {
        pPwmDutycycle->dutycycle3 = max;
    }
}