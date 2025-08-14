#include <xc.h>
#include "port_config.h"

#include "userparms.h"
void MapGPIOHWFunction (void);
void SetupGPIOPorts(void);

void SetupGPIOPorts(void)
{

    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    #ifdef TRISE
        TRISE = 0xFFFF;
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;
    #endif

    MapGPIOHWFunction();

    return;
}

void MapGPIOHWFunction(void)
{
    ANSELAbits.ANSELA0 = 1;
    TRISAbits.TRISA0 = 1;
    
    ANSELBbits.ANSELB2 = 1;
    TRISBbits.TRISB2 = 1;
    
    ANSELAbits.ANSELA4 = 1;
    TRISAbits.TRISA4 = 1;

#ifdef INTERNAL_OPAMP_CONFIG     
    ANSELAbits.ANSELA1 = 1;
    TRISAbits.TRISA1 = 1;

    ANSELAbits.ANSELA2 = 1;
    TRISAbits.TRISA2 = 1;
    
    ANSELBbits.ANSELB3 = 1;
    TRISBbits.TRISB3 = 1;

    ANSELBbits.ANSELB4 = 1;
    TRISBbits.TRISB4 = 1;
    
    ANSELCbits.ANSELC1 = 1;
    TRISCbits.TRISC1 = 1;
    
    ANSELCbits.ANSELC2 = 1;
    TRISCbits.TRISC2 = 1;

    AMPCON1Hbits.NCHDIS1 = 0;
    AMPCON1Lbits.AMPEN1 = 1;
    
    AMPCON1Hbits.NCHDIS2 = 0;
    AMPCON1Lbits.AMPEN2 = 1;

    AMPCON1Hbits.NCHDIS3 = 0;
    AMPCON1Lbits.AMPEN3 = 1;

    AMPCON1Lbits.AMPON = 1;
    
#endif    

    ANSELBbits.ANSELB9= 1;
    TRISBbits.TRISB9 = 1;

    ANSELCbits.ANSELC0 = 1;
    TRISCbits.TRISC0 = 1;

    ANSELCbits.ANSELC3 = 1;
    TRISCbits.TRISC3 = 1;

    TRISBbits.TRISB14 = 0 ;          
    TRISBbits.TRISB15 = 0 ;         
    TRISBbits.TRISB12 = 0 ;          
    TRISBbits.TRISB13 = 0 ;           
    TRISDbits.TRISD1 = 0 ;          
    TRISDbits.TRISD0 = 0 ; 

    TRISEbits.TRISE7 = 0;
    TRISEbits.TRISE6 = 0;

    TRISEbits.TRISE11 = 1;
    TRISEbits.TRISE12 = 1;

    _U1RXR = 78;
    _RP77R = 0b000001;
}
