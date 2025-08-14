#include <libq.h>
#include "userparms.h"
#include "singleshunt.h"


SINGLE_SHUNT_PARM_T singleShuntParam;
inline static void SingleShunt_CalculateSwitchingTime(SINGLE_SHUNT_PARM_T *,uint16_t );

void SingleShunt_InitializeParameters(SINGLE_SHUNT_PARM_T *pSingleShunt)
{    
	pSingleShunt->tcrit = SSTCRIT;
	pSingleShunt->tDelaySample = SS_SAMPLE_DELAY;	
    pSingleShunt->trigger1 = 0;
    pSingleShunt->trigger2 = 0;
    pSingleShunt->Ibus1 = 0;
    pSingleShunt->Ibus2 = 0;
    pSingleShunt->adcSamplePoint = 0;
}

uint16_t SingleShunt_CalculateSpaceVectorPhaseShifted(MC_ABC_T *abc,
                                    uint16_t iPwmPeriod,
                                    SINGLE_SHUNT_PARM_T *pSingleShunt)
{ 
    uint16_t mcCorconSave = CORCON;
    CORCON = 0x00E2;
    
    MC_DUTYCYCLEOUT_T *pdcout1 = &pSingleShunt->pwmDutycycle1;
    MC_DUTYCYCLEOUT_T *pdcout2 = &pSingleShunt->pwmDutycycle2;   
    if (abc->a >= 0)
    {
        if (abc->b >= 0)
        {
            pSingleShunt->sectorSVM  = 3; 
            pSingleShunt->T1 = abc->a;
            pSingleShunt->T2 = abc->b;
            SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
            pdcout1->dutycycle1 = pSingleShunt->Ta1;
            pdcout1->dutycycle2 = pSingleShunt->Tb1;
            pdcout1->dutycycle3 = pSingleShunt->Tc1;
            pdcout2->dutycycle1 = pSingleShunt->Ta2;
            pdcout2->dutycycle2 = pSingleShunt->Tb2;
            pdcout2->dutycycle3 = pSingleShunt->Tc2;
        }
        else
        {
            if (abc->c >= 0)
            {
                pSingleShunt->sectorSVM  = 5;
                pSingleShunt->T1 = abc->c;
                pSingleShunt->T2 = abc->a;
                SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
                pdcout1->dutycycle1 = pSingleShunt->Tc1;
                pdcout1->dutycycle2 = pSingleShunt->Ta1;
                pdcout1->dutycycle3 = pSingleShunt->Tb1;
                pdcout2->dutycycle1 = pSingleShunt->Tc2;
                pdcout2->dutycycle2 = pSingleShunt->Ta2;
                pdcout2->dutycycle3 = pSingleShunt->Tb2;
            }
            else
            {
                pSingleShunt->sectorSVM  = 1;
                pSingleShunt->T1 = -abc->c;
                pSingleShunt->T2 = -abc->b;
                SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
                pdcout1->dutycycle1 = pSingleShunt->Tb1;
                pdcout1->dutycycle2 = pSingleShunt->Ta1;
                pdcout1->dutycycle3 = pSingleShunt->Tc1;
                pdcout2->dutycycle1 = pSingleShunt->Tb2;
                pdcout2->dutycycle2 = pSingleShunt->Ta2;
                pdcout2->dutycycle3 = pSingleShunt->Tc2;
            }
        }
    }
    else
    {
        if (abc->b >= 0)
        {
            if (abc->c >= 0)
            {
                pSingleShunt->sectorSVM  = 6;
                pSingleShunt->T1 = abc->b;
                pSingleShunt->T2 = abc->c;
                SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
                pdcout1->dutycycle1 = pSingleShunt->Tb1;
                pdcout1->dutycycle2 = pSingleShunt->Tc1;
                pdcout1->dutycycle3 = pSingleShunt->Ta1;
                pdcout2->dutycycle1 = pSingleShunt->Tb2;
                pdcout2->dutycycle2 = pSingleShunt->Tc2;
                pdcout2->dutycycle3 = pSingleShunt->Ta2;
            }
            else
            {
                pSingleShunt->sectorSVM  = 2;
                pSingleShunt->T1 = -abc->a;
                pSingleShunt->T2 = -abc->c;
                SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
                pdcout1->dutycycle1 = pSingleShunt->Ta1;
                pdcout1->dutycycle2 = pSingleShunt->Tc1;
                pdcout1->dutycycle3 = pSingleShunt->Tb1;
                pdcout2->dutycycle1 = pSingleShunt->Ta2;
                pdcout2->dutycycle2 = pSingleShunt->Tc2;
                pdcout2->dutycycle3 = pSingleShunt->Tb2;
            }
        }
        else
        {
            pSingleShunt->sectorSVM  = 4;
            pSingleShunt->T1 = -abc->b;
            pSingleShunt->T2 = -abc->a;
            SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
            pdcout1->dutycycle1 = pSingleShunt->Tc1;
            pdcout1->dutycycle2 = pSingleShunt->Tb1;
            pdcout1->dutycycle3 = pSingleShunt->Ta1;
            pdcout2->dutycycle1 = pSingleShunt->Tc2;
            pdcout2->dutycycle2 = pSingleShunt->Tb2;
            pdcout2->dutycycle3 = pSingleShunt->Ta2;
        }
    }
	
    pSingleShunt->trigger1 = (iPwmPeriod + pSingleShunt->tDelaySample);
    pSingleShunt->trigger1 = pSingleShunt->trigger1 - ((pSingleShunt->Ta1 + pSingleShunt->Tb1) >> 1) ;
    pSingleShunt->trigger2 = (iPwmPeriod +  pSingleShunt->tDelaySample);
    pSingleShunt->trigger2 = pSingleShunt->trigger2 - ((pSingleShunt->Tb1 + pSingleShunt->Tc1) >> 1) ;
	PWM_TRIGB = singleShuntParam.trigger1;
    PWM_TRIGC = singleShuntParam.trigger2;
    CORCON = mcCorconSave;
    return(1);
}

inline static void SingleShunt_CalculateSwitchingTime(SINGLE_SHUNT_PARM_T *pSingleShunt,
        uint16_t iPwmPeriod)
{
	
    pSingleShunt->T1 = (int16_t) (__builtin_mulss(iPwmPeriod,pSingleShunt->T1) >> 15);
    pSingleShunt->T2 = (int16_t) (__builtin_mulss(iPwmPeriod,pSingleShunt->T2) >> 15);
    pSingleShunt->T7 = (iPwmPeriod-pSingleShunt->T1-pSingleShunt->T2)>>1;

    if (pSingleShunt->T1 > pSingleShunt->tcrit)
    {
        pSingleShunt->Tc1 = pSingleShunt->T7;
        pSingleShunt->Tc2 = pSingleShunt->T7;
    }

    else
    {
        pSingleShunt->Tc1 = pSingleShunt->T7 - (pSingleShunt->tcrit-pSingleShunt->T1);
        pSingleShunt->Tc2 = pSingleShunt->T7 + (pSingleShunt->tcrit-pSingleShunt->T1);
    }
    pSingleShunt->Tb1 = pSingleShunt->T7 + pSingleShunt->T1;
    pSingleShunt->Tb2 = pSingleShunt->Tb1;

    if (pSingleShunt->T2 > pSingleShunt->tcrit)
    {
        pSingleShunt->Ta1 = pSingleShunt->Tb1 + pSingleShunt->T2;
        pSingleShunt->Ta2 = pSingleShunt->Tb2 + pSingleShunt->T2;
    }

    else
    {
        pSingleShunt->Ta1 = pSingleShunt->Tb1 + pSingleShunt->tcrit;
        pSingleShunt->Ta2 = pSingleShunt->Tb2 + pSingleShunt->T2 + pSingleShunt->T2 - pSingleShunt->tcrit;
    }
	
}    

void SingleShunt_PhaseCurrentReconstruction(SINGLE_SHUNT_PARM_T *pSingleShunt)
{
    switch(pSingleShunt->sectorSVM)
    {
        case 1:
            pSingleShunt->Ib = pSingleShunt->Ibus1;
            pSingleShunt->Ic = -pSingleShunt->Ibus2;
            pSingleShunt->Ia = -pSingleShunt->Ic - pSingleShunt->Ib;
        break;
        case 2:
            pSingleShunt->Ia = pSingleShunt->Ibus1;
            pSingleShunt->Ib = -pSingleShunt->Ibus2;
            pSingleShunt->Ic = -pSingleShunt->Ia - pSingleShunt->Ib;
        break;
        case 3:
            pSingleShunt->Ia = pSingleShunt->Ibus1; 
            pSingleShunt->Ic = -pSingleShunt->Ibus2;
            pSingleShunt->Ib = -pSingleShunt->Ia - pSingleShunt->Ic;
        break;
        case 4:
            pSingleShunt->Ic = pSingleShunt->Ibus1; 
            pSingleShunt->Ia = -pSingleShunt->Ibus2; 
            pSingleShunt->Ib = -pSingleShunt->Ia - pSingleShunt->Ic;
        break;
        case 5:
            pSingleShunt->Ib = pSingleShunt->Ibus1; 
            pSingleShunt->Ia = -pSingleShunt->Ibus2; 
            pSingleShunt->Ic = -pSingleShunt->Ia - pSingleShunt->Ib;
        break;
        case 6:
            pSingleShunt->Ic = pSingleShunt->Ibus1; 
            pSingleShunt->Ib = -pSingleShunt->Ibus2;
            pSingleShunt->Ia = -pSingleShunt->Ic - pSingleShunt->Ib;
        break;   
    }  
}