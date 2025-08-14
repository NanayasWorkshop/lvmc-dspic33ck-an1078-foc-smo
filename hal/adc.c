#include <xc.h>
#include <stdint.h>
#include "userparms.h"
#include "adc.h"

void InitializeADCs(void);
void InitializeADCs (void)
{
    ADCON1L = 0;
    ADCON1Lbits.ADON = 0;
    ADCON1Lbits.ADSIDL = 0;

    ADCON1H = 0;
    ADCON1Hbits.SHRRES = 3;
    ADCON1Hbits.FORM = 1;

    ADCON2L = 0;
    ADCON2Lbits.SHRADCS = 0;
    ADCON2Lbits.EIEN = 0;

    ADCON2H = 0;
    ADCON2Hbits.SHRSAMC = 15;

    ADCON3L  = 0;
    ADCON3Lbits.REFSEL = 0;

    ADCON3H = 0;
    ADCON3Hbits.C0EN      = 0 ;
    ADCON3Hbits.C1EN      = 0 ;
    ADCON3Hbits.SHREN = 0;
    ADCON3Hbits.CLKSEL = 0;
    ADCON3Hbits.CLKDIV = 0;
    
    ADCON4L      = 0x0000;
    ADCON4Lbits.SAMC0EN = 0;
    ADCON4Lbits.SAMC1EN = 0;

    ADCON4H      = 0x0000;
    ADCON4Hbits.C0CHS = 0;
    ADCON4Hbits.C1CHS = 0;
    
    ADCORE0L     = 0x0000;
    ADCORE0Lbits.SAMC = 8;
    ADCORE0H     = 0x0000;
    ADCORE0Hbits.ADCS = 0;
    ADCORE0Hbits.RES = 3;
    
    ADCORE1L     = 0x0000;
    ADCORE1Lbits.SAMC = 8;
    ADCORE1H     = 0x0000;
    ADCORE1Hbits.ADCS = 0;
    ADCORE1Hbits.RES = 3;
    
    ADMOD0L = 0x0000;
    ADMOD0Lbits.SIGN4 = 1;
    ADMOD0Lbits.SIGN1 = 1;
    ADMOD0Lbits.SIGN0 = 1;
   
    ADMOD0H = 0;
    ADMOD0Hbits.SIGN11 = 0; 
    ADMOD0Hbits.SIGN12 = 0;    
    ADMOD0Hbits.SIGN15 = 0;
    
    ADIEL = 0;
    ADIEH = 0;
    ADSTATL = 0;
    ADSTATH = 0;
    ADEIEL  = 0;
    ADEIEH  = 0;
    ADEISTATL = 0;
    ADEISTATH = 0;
 
    ADCON5H = 0;
    ADCON5Hbits.SHRCIE = 0;
    ADCON5Hbits.WARMTIME  = 0b1111 ;                                         
    
    ADCON1Lbits.ADON      = 1 ;  
    
    ADCON5L = 0;
    ADCON5Lbits.C0PWR     = 1 ;
    while(ADCON5Lbits.C0RDY == 0);
    ADCON3Hbits.C0EN      = 1 ;
    ADCON5Lbits.C1PWR     = 1 ;
    while(ADCON5Lbits.C1RDY == 0);
    ADCON3Hbits.C1EN      = 1 ;
    ADCON5Lbits.SHRPWR    = 1 ;
    while(ADCON5Lbits.SHRRDY == 0);
    ADCON3Hbits.SHREN     = 1 ;
    
#ifdef SINGLE_SHUNT    
     _IE4        = 1 ;
    _ADCAN4IF    = 0 ;  
    _ADCAN4IP   = 7 ;  
    _ADCAN4IE    = 0 ;
#else
    _IE11        = 1 ;
    _ADCAN11IF    = 0 ;  
    _ADCAN11IP   = 7 ;  
    _ADCAN11IE    = 0 ;  
#endif

    _IE1        = 0 ;
    _ADCAN1IF    = 0 ;  
    _ADCAN1IP   = 7 ;  
    _ADCAN1IE    = 0 ;  
    
    _IE0        = 0 ;
    _ADCAN0IF    = 0 ;  
    _ADCAN0IP   = 7 ;  
    _ADCAN0IE    = 0 ;  

    ADTRIG0Lbits.TRGSRC0 = 0x4;
    ADTRIG0Lbits.TRGSRC1 = 0x4;
    
#ifdef SINGLE_SHUNT
    ADTRIG1Lbits.TRGSRC4 = 0x5;
    ADTRIG2Hbits.TRGSRC11 = 0x1;
    ADTRIG3Lbits.TRGSRC12 = 0x1;
    ADTRIG3Hbits.TRGSRC15 = 0x1;
#else
    ADTRIG2Hbits.TRGSRC11 = 0x4;
    ADTRIG3Lbits.TRGSRC12 = 0x4;
    ADTRIG3Hbits.TRGSRC15 = 0x4;
#endif

}
