#include <stdint.h>
#include <stdbool.h>

#include "measure.h"
#include "adc.h"

void MCAPP_MeasureCurrentInit(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->current;
    
    pCurrent->counter = 0;
    pCurrent->sumIa = 0;
    pCurrent->sumIb = 0;
    pCurrent->sumIbus = 0;
    pCurrent->status = 0;
}

void MCAPP_MeasureCurrentOffset(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->current;
    
    pCurrent->sumIa += pCurrent->Ia;
    pCurrent->sumIb += pCurrent->Ib;
    pCurrent->sumIbus += pCurrent->Ibus;
    pCurrent->counter++;

    if (pCurrent->counter >= OFFSET_COUNT_MAX)
    {
        pCurrent->offsetIa = (int16_t)(pCurrent->sumIa >> OFFSET_COUNT_BITS);
        pCurrent->offsetIb = (int16_t)(pCurrent->sumIb >> OFFSET_COUNT_BITS);
        pCurrent->offsetIbus =
            (int16_t)(pCurrent->sumIbus >> OFFSET_COUNT_BITS);

        pCurrent->counter = 0;
        pCurrent->sumIa = 0;
        pCurrent->sumIb = 0;
        pCurrent->sumIbus = 0;
        pCurrent->status = 1;
    }
}

void MCAPP_MeasureCurrentCalibrate(MCAPP_MEASURE_T *pMotorInputs)
{
    MCAPP_MEASURE_CURRENT_T *pCurrent;
    
    pCurrent = &pMotorInputs->current;
    
    pCurrent->Ia = pCurrent->Ia - pCurrent->offsetIa;
    pCurrent->Ib = pCurrent->Ib  - pCurrent->offsetIb;
}

int16_t MCAPP_MeasureCurrentOffsetStatus (MCAPP_MEASURE_T *pMotorInputs)
{
    return pMotorInputs->current.status;
}

void MCAPP_MeasureAvgInit(MCAPP_MEASURE_AVG_T *pFilterData,uint16_t scaler)
{
    pFilterData->scaler = scaler;
    pFilterData->maxIndex = (uint16_t)((1 << scaler));
    pFilterData->index = 0;
    pFilterData->sum = 0;
    pFilterData->status = 0;
}

int16_t MCAPP_MeasureAvg(MCAPP_MEASURE_AVG_T *pFilterData)
{    
    pFilterData->sum += pFilterData->input;

    if (pFilterData->index < pFilterData->maxIndex)
    {
        pFilterData->index++;
    }
    else
    {
        pFilterData->avg = pFilterData->sum >> pFilterData->scaler; 
        pFilterData->sum = 0;
        pFilterData->index = 0;
        pFilterData->status = 1;
    }
    return pFilterData->avg;
}

void MCAPP_MeasureTemperature(MCAPP_MEASURE_T *pData, int16_t input)
{
    pData->MOSFETTemperature.input = input;
    pData->MOSFETTemperatureAvg = MCAPP_MeasureAvg(&pData->MOSFETTemperature);
    if (pData->MOSFETTemperature.status == 1)
    {
        pData->MOSFETTemperatureAvg = (int16_t)(__builtin_mulss
                ((pData->MOSFETTemperatureAvg-OFFSET_COUNT_MOSFET_TEMP) ,
                MOSFET_TEMP_COEFF) >> 15);
    }
}