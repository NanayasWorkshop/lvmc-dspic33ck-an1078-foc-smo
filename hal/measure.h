#ifndef __MEASURE_H
#define __MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "general.h"

#define OFFSET_COUNT_BITS   (int16_t)10
#define OFFSET_COUNT_MAX    (int16_t)(1 << OFFSET_COUNT_BITS)
    
#define OFFSET_COUNT_MOSFET_TEMP 4964
#define MOSFET_TEMP_COEFF Q15(0.010071108)
#define MOSFET_TEMP_AVG_FILTER_SCALE     8

typedef struct
{
    int16_t
        offsetIa,
        offsetIb,
        offsetIbus,
        Ia,
        Ib,
        Ibus,
        counter,
        status;

    int32_t
        sumIa,
        sumIb,
        sumIbus;

} MCAPP_MEASURE_CURRENT_T;

typedef struct
{
    int16_t input;
    uint16_t index;
    uint16_t maxIndex;
    uint16_t scaler;
    int16_t avg;
    int32_t sum;
    uint16_t status;
    
}MCAPP_MEASURE_AVG_T;

typedef struct
{
    int16_t 
        potValue;
    int16_t
        dcBusVoltage;
    int16_t
        MOSFETTemperatureAvg;  
    MCAPP_MEASURE_AVG_T MOSFETTemperature;
    MCAPP_MEASURE_CURRENT_T
        current; 
            
}MCAPP_MEASURE_T;

void MCAPP_MeasureCurrentOffset (MCAPP_MEASURE_T *);
void MCAPP_MeasureCurrentCalibrate (MCAPP_MEASURE_T *);
void MCAPP_MeasureCurrentInit (MCAPP_MEASURE_T *);
int16_t MCAPP_MeasureCurrentOffsetStatus (MCAPP_MEASURE_T *);
void MCAPP_MeasureTemperature(MCAPP_MEASURE_T *,int16_t );
void MCAPP_MeasureAvgInit(MCAPP_MEASURE_AVG_T *,uint16_t );
int16_t MCAPP_MeasureAvg(MCAPP_MEASURE_AVG_T *);

#ifdef __cplusplus
}
#endif

#endif
