#include "X2CScope.h"
#include "hal/uart1.h"
#include <stdint.h>

#define X2C_BAUDRATE_DIVIDER 54

void X2CScope_Init(void);

void DiagnosticsInit(void)
{
    UART1_InterruptReceiveDisable();
    UART1_InterruptReceiveFlagClear();
    UART1_InterruptTransmitDisable();
    UART1_InterruptTransmitFlagClear();
    UART1_Initialize();
    UART1_BaudRateDividerSet(X2C_BAUDRATE_DIVIDER);
    UART1_SpeedModeStandard();
    UART1_ModuleEnable();  
    
    X2CScope_Init();
}

void DiagnosticsStepMain(void)
{
    X2CScope_Communicate();
}

void DiagnosticsStepIsr(void)
{
    X2CScope_Update();
}


static void X2CScope_sendSerial(uint8_t data)
{
    UART1_DataWrite(data);
}

static uint8_t X2CScope_receiveSerial()
{
    const uint16_t error_mask = _U1STA_OERR_MASK 
                              | _U1STA_FERR_MASK
                              | _U1STA_PERR_MASK;
    if (UART1_StatusGet() & error_mask)
    {
        UART1_ReceiveBufferOverrunErrorFlagClear();
        return 0;
    }
    return UART1_DataRead();
}

static uint8_t X2CScope_isReceiveDataAvailable()
{
    return UART1_IsReceiveBufferDataReady();
}

static uint8_t X2CScope_isSendReady()
{
    return !UART1_StatusBufferFullTransmitGet();
}

void X2CScope_Init(void)
{
    X2CScope_HookUARTFunctions(
        X2CScope_sendSerial,
        X2CScope_receiveSerial,
        X2CScope_isReceiveDataAvailable,
        X2CScope_isSendReady);
    X2CScope_Initialise();
}