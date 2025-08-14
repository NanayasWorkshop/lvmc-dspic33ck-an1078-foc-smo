#ifndef __X2CSCOPE_H__
#define __X2CSCOPE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void X2CScope_Initialise();
void X2CScope_Communicate();
void X2CScope_Update();
void X2CScope_HookUARTFunctions(void (*sendSerialFcnPntr)(uint8_t), uint8_t (*receiveSerialFcnPntr)(), uint8_t (*isReceiveDataAvailableFcnPntr)(), uint8_t (*isSendReadyFcnPntr)());

#ifdef __cplusplus
}
#endif

#endif 
