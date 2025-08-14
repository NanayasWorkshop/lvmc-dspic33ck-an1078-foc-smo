#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#include <stdint.h>
#include <stdbool.h>
#include "cmp.h"

static void CMP1_Initialize(void);

void CMP_Initialize (void)
{
    DACCTRL1L = 0;
    DACCTRL1Lbits.DACON = 0;
    DACCTRL1Lbits.DACSIDL = 0;
    DACCTRL1Lbits.CLKSEL = 3;
    DACCTRL1Lbits.CLKDIV = 1;
    DACCTRL1Lbits.FCLKDIV = 0b111;
    
    DACCTRL2L = 0;
    DACCTRL2Lbits.TMODTIME = 0;
    
    DACCTRL2H = 0;
    DACCTRL2Hbits.SSTIME = 0;

    CMP1_Initialize();    
}

void CMP1_Initialize(void)
{
    DAC1CONL = 0;
    DAC1CONLbits.DACEN = 0;
    DAC1CONLbits.IRQM = 0;
    DAC1CONLbits.CBE = 0;
    DAC1CONLbits.DACOEN = 0;
    DAC1CONLbits.FLTREN = 1;
    DAC1CONLbits.CMPSTAT = 0;
    DAC1CONLbits.CMPPOL = 0;
    DAC1CONLbits.INSEL = 2;   
    DAC1CONLbits.HYSPOL = 0; 
    DAC1CONLbits.HYSSEL = 0b11; 
    DAC1CONH = 0;
    DAC1CONHbits.TMCB = 0;
    DAC1DATL = 0;
    DAC1DATH = 0;
    SLP1CONL = 0;
    SLP1CONLbits.HCFSEL = 0;
    SLP1CONLbits.SLPSTOPA = 0 ;
    SLP1CONLbits.SLPSTOPB = 0 ;  
    SLP1CONLbits.SLPSTRT = 0 ;    
    SLP1CONH = 0;
    SLP1CONHbits.SLOPEN = 0;
    SLP1CONHbits.HME = 0 ;
    SLP1CONHbits.TWME = 0 ;  
    SLP1CONHbits.PSE = 0 ;   
    SLP1DAT = 0;
}

void CMP1_ReferenceSet(uint16_t data)
{
    DAC1DATH = data;
}

void CMP1_ModuleEnable(bool state)
{
    if (state == true)
    {
        DAC1CONLbits.DACEN = 1;
        DACCTRL1Lbits.DACON = 1;
    }
    else
    {
        DAC1CONLbits.DACOEN = 0;
        DAC1CONLbits.DACEN = 0;
        DACCTRL1Lbits.DACON = 0;
    }
}

