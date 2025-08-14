#include <xc.h>
#include <stdint.h>
#include "clock.h"

void InitOscillator(void);
void EnableREFCLKOutput(uint16_t);
 
void InitOscillator(void)

{
    CLKDIVbits.DOZEN = 0;
    
    CLKDIVbits.FRCDIV = 0;

    PLLDIVbits.VCODIV = 2;
        
    PLLFBDbits.PLLFBDIV = 150;

    CLKDIVbits.PLLPRE = 1;

    PLLDIVbits.POST1DIV = 3;
    
    PLLDIVbits.POST2DIV = 1;
    
    __builtin_write_OSCCONH(0x01);

    __builtin_write_OSCCONL(OSCCON | 0x01);

    while (OSCCONbits.OSWEN!= 0);

    while (OSCCONbits.LOCK != 1);

}

void EnableREFCLKOutput(uint16_t Divider)
{
    
    if(REFOCONLbits.ROACTIVE == 0)
    {
        REFOCONHbits.RODIV = Divider;
        REFOCONLbits.ROSLP = 1;
        REFOCONLbits.ROSIDL = 1;
        REFOCONLbits.ROSEL = 1;   
        REFOCONLbits.ROOUT = 1;
        REFOCONLbits.ROEN = 1;
    }
}
