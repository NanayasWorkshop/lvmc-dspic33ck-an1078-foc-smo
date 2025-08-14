#ifndef __BOARD_SERVICE_H
#define __BOARD_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#ifdef __XC16__ 
    #include <xc.h>
#endif // __XC16__

#ifdef __cplusplus
    extern "C" {
#endif

#include "clock.h"
#include "pwm.h"
#include "adc.h"
#include "port_config.h"
#include "userparms.h"
#include "motor_control_noinline.h"

typedef enum tagBUTTON_STATE
{
    BUTTON_NOT_PRESSED = 0,
    BUTTON_PRESSED = 1, 
    BUTTON_DEBOUNCE = 2
} BUTTON_STATE;
    
typedef struct
{
   BUTTON_STATE state;
   uint16_t debounceCount;
   bool logicState;
   bool status;
} BUTTON_T;

#define	BUTTON_DEBOUNCE_COUNT       40
#define BOARD_SERVICE_TICK_COUNT   (uint16_t)(PWMFREQUENCY_HZ/1000)

extern void BoardServiceInit(void);
extern void BoardServiceStepIsr(void);
extern void BoardService(void);
extern bool IsPressed_Button1(void);
extern bool IsPressed_Button2(void);

extern void DisablePWMOutputs(void);
extern void EnablePWMOutputs(void);
extern void ClearPWMPCIFault(void);
extern void Init_Peripherals(void);

extern void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *,MC_DUTYCYCLEOUT_T *);
extern void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *);
extern void pwmDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *,uint16_t,uint16_t);

#ifdef __cplusplus
}
#endif

#endif 
