#ifndef __UART1_H
#define __UART1_H
    
#include <xc.h>

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus  
    extern "C" {
#endif
                     
extern void UART1_Initialize(void);

inline static void UART1_InterruptTransmitFlagClear(void) {_U1TXIF = 0; }

inline static void UART1_InterruptReceiveFlagClear(void) {_U1RXIF = 0; }

inline static void UART1_InterruptTransmitEnable(void) {_U1TXIE = 1; }

inline static void UART1_InterruptTransmitDisable(void) {_U1TXIE = 0; }

inline static void UART1_InterruptReceiveEnable(void) {_U1RXIE = 1; }

inline static void UART1_InterruptReceiveDisable(void) {_U1RXIE = 0; }

inline static void UART1_SpeedModeStandard(void) {U1MODEbits.BRGH = 0; }

inline static void UART1_SpeedModeHighSpeed(void) {U1MODEbits.BRGH = 1; }

inline static void UART1_BaudRateDividerSet(uint16_t baudRateDivider)
{
    U1BRG = baudRateDivider;
}

inline static void UART1_ModuleDisable(void) 
{
    U1MODEbits.UARTEN = 0;
}

inline static void UART1_ModuleEnable(void) 
{
    U1MODEbits.UARTEN = 1;
}

inline static void UART1_TransmitModeEnable(void) {U1MODEbits.UTXEN = 1; }

inline static void UART1_TransmitModeDisable(void) {U1MODEbits.UTXEN= 0; }

inline static bool UART1_IsReceiveBufferDataReady(void)
{
    return(!U1STAHbits.URXBE) ;
}

inline static bool UART1_IsReceiveBufferOverFlowDetected(void)
{
    return(U1STAbits.OERR) ;
}

inline static bool UART1_IsFrameErrorDetected(void)
{
    return(U1STAbits.FERR) ;
}

inline static bool UART1_IsParityErrorDetected(void)
{
    return(U1STAbits.PERR) ;
}

inline static bool UART1_IsReceiverIdle(void)
{
    return(U1STAHbits.RIDLE) ;
}

inline static bool UART1_IsTransmissionComplete(void)
{
    return(U1STAbits.TRMT) ;
}

inline static bool UART1_StatusBufferFullTransmitGet(void)
{
    return U1STAHbits.UTXBF;
}

inline static uint16_t UART1_StatusGet(void)
{
    return U1STA;
}

inline static void UART1_ReceiveBufferOverrunErrorFlagClear(void)
{
    U1STAbits.OERR = 0;
}

inline static void UART1_DataWrite(uint16_t data)
{
    U1TXREGbits.TXREG =(uint8_t)data;
}

inline static uint16_t UART1_DataRead(void)
{
    return U1RXREG;
}


#ifdef __cplusplus 
    }
#endif

#endif 
