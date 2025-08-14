#include <xc.h>

#include <stdint.h>
#include <stdbool.h>
#include "uart1.h"

void UART1_Initialize(void);

void UART1_Initialize (void)
{
    U1MODE = 0;
    U1MODEbits.UARTEN = 0;
    U1MODEbits.USIDL = 0;
    U1MODEbits.WAKE  = 0;
    U1MODEbits.RXBIMD = 0;
    U1MODEbits.BRKOVR = 0;
    U1MODEbits.UTXBRK = 0;
    U1MODEbits.BRGH = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.UTXEN = 1;
    U1MODEbits.URXEN = 1;
    U1MODEbits.MOD = 0;
    
    U1MODEH =  0;
    U1MODEHbits.SLPEN = 0;
    U1MODEHbits.ACTIVE = 0;

    U1MODEHbits.BCLKSEL = 0;
    U1MODEHbits.HALFDPLX = 0;
    U1MODEHbits.RUNOVF = 0;
    U1MODEHbits.URXINV = 0;
    U1MODEHbits.STSEL = 0;
    U1MODEHbits.C0EN = 0;
    U1MODEHbits.UTXINV = 0;
    U1MODEHbits.FLO = 0;

    U1STA = 0;
    U1STAbits.TXMTIE = 0;
    U1STAbits.PERIE = 0;
    U1STAbits.ABDOVE = 0;
    U1STAbits.CERIE = 0;
    U1STAbits.FERIE = 0;
    U1STAbits.RXBKIE = 0;
    U1STAbits.OERIE = 0;
    U1STAbits.TXCIE = 0;
    U1STAbits.TRMT = 0;
    U1STAbits.PERR = 0;
    U1STAbits.ABDOVF = 0;
    U1STAbits.CERIF = 0;
    U1STAbits.FERR = 0;
    U1STAbits.RXBKIF = 0;
    U1STAbits.OERR = 0;
    U1STAbits.TXCIF = 0;

    U1STAH = 0;
    U1STAHbits.UTXISEL = 7;
    U1STAHbits.URXISEL = 0;
    U1STAHbits.TXWRE = 0;
    U1STAHbits.STPMD = 0;
    U1STAHbits.UTXBE = 1;
    U1STAHbits.UTXBF = 0;
    U1STAHbits.RIDLE = 0;
    U1STAHbits.XON = 0;
    U1STAHbits.URXBE = 1;
    U1STAHbits.URXBF = 0;
    
    U1BRG =  0;

    U1BRGH = 0;

    U1RXREG = 0;
    U1RXREGbits.RXREG = 0;

    U1TXREG =  0;
    U1TXREGbits.LAST = 0;
    U1TXREGbits.TXREG = 0;
    
    U1P1 = 0;
    U1P2 = 0;
    U1P3 = 0;
    U1P3H = 0;
    U1TXCHK = 0;
    U1RXCHK = 0;
    U1SCCON = 0;
    U1SCINT =  0;

    U1INT = 0;
    U1INTbits.WUIF = 0;
    U1INTbits.ABDIF = 0;
    U1INTbits.ABDIE = 0;
    
    U1MODEHbits.ACTIVE = 0;
    U1MODEbits.UARTEN = 0;
}