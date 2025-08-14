#include <xc.h>
#include <stdint.h>
#include "pwm.h"
#include "userparms.h"

void InitPWMGenerator1 (void);
void InitPWMGenerator2 (void);
void InitPWMGenerator4 (void);
void InitDutyPWM124Generators(void);
void InitPWMGenerators(void);   
void ChargeBootstarpCapacitors(void);

void InitPWMGenerators(void)
{

    PCLKCON      = 0x0000;
    PCLKCONbits.DIVSEL = 0;
    PCLKCONbits.MCLKSEL = 0;
    PCLKCONbits.LOCK = 0;

    MPHASE       = 0x0000;
    MDC          = 0x0000;
    MPER         = LOOPTIME_TCY;

    FSCL          = 0x0000;
    FSMINPER     = 0x0000;
    LFSR         = 0x0000;
    CMBTRIGL     = 0x0000;
    CMBTRIGH     = 0x0000;
    LOGCONA     = 0x0000;
    LOGCONB     = 0x0000;
    LOGCONC     = 0x0000;
    LOGCOND     = 0x0000;
    LOGCONE     = 0x0000;
    LOGCONF     = 0x0000;
    PWMEVTA     = 0x0000;     
    PWMEVTB     = 0x0000;
    PWMEVTC     = 0x0000;
    PWMEVTD     = 0x0000;
    PWMEVTE     = 0x0000;
    PWMEVTF     = 0x0000;

    InitPWMGenerator1 ();
    InitPWMGenerator2 ();
    InitPWMGenerator4 (); 


    InitDutyPWM124Generators();

    IFS4bits.PWM1IF = 0;
    IEC4bits.PWM1IE = 1;
    IPC16bits.PWM1IP = 7;

	PG2CONLbits.ON = 1;
    PG4CONLbits.ON = 1;
    PG1CONLbits.ON = 1;

    ChargeBootstarpCapacitors();
}
void InitDutyPWM124Generators(void)
{

    PG4IOCONLbits.OVRDAT = 0;  // 0b00 = State for PWM4H,L, if Override is Enabled
    PG2IOCONLbits.OVRDAT = 0;  // 0b00 = State for PWM2H,L, if Override is Enabled
    PG1IOCONLbits.OVRDAT = 0;  // 0b00 = State for PWM1H,L, if Override is Enabled

    PG4IOCONLbits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM4H
    PG2IOCONLbits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM2H
    PG1IOCONLbits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM1H

    PG4IOCONLbits.OVRENL = 1;  // 0 = PWM generator provides data for PWM4L pin
    PG2IOCONLbits.OVRENL = 1;  // 0 = PWM generator provides data for PWM2L pin
    PG1IOCONLbits.OVRENL = 1;  // 0 = PWM generator provides data for PWM1L pin

    PG4DC = 0;
    PG2DC = 0;
    PG1DC = 0;

}

void ChargeBootstarpCapacitors(void)
{
    uint16_t i = BOOTSTRAP_CHARGING_COUNTS;
    uint16_t prevStatusCAHALF = 0,currStatusCAHALF = 0;
    uint16_t k = 0;
    
    // Enable PWMs only on PWMxL ,to charge bootstrap capacitors at the beginning
    // Hence PWMxH is over-ridden to "LOW"
    PG4IOCONLbits.OVRDAT = 0;  // 0b00 = State for PWM4H,L, if Override is Enabled
    PG2IOCONLbits.OVRDAT = 0;  // 0b00 = State for PWM2H,L, if Override is Enabled
    PG1IOCONLbits.OVRDAT = 0;  // 0b00 = State for PWM1H,L, if Override is Enabled

    PG4IOCONLbits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM4H
    PG2IOCONLbits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM2H
    PG1IOCONLbits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM1H

    PG4IOCONLbits.OVRENL = 1;  // 1 = OVRDAT<0> provides data for output on PWM4L
    PG2IOCONLbits.OVRENL = 1;  // 1 = OVRDAT<0> provides data for output on PWM2L
    PG1IOCONLbits.OVRENL = 1;  // 1 = OVRDAT<0> provides data for output on PWM1L

    PWM_PDC3 = LOOPTIME_TCY - (DDEADTIME/2 + 5);
    PWM_PDC2 = LOOPTIME_TCY - (DDEADTIME/2 + 5);
    PWM_PDC1 = LOOPTIME_TCY - (DDEADTIME/2 + 5);
    
    while(i)
    {
        prevStatusCAHALF = currStatusCAHALF;
        currStatusCAHALF = PG1STATbits.CAHALF;
        if(prevStatusCAHALF != currStatusCAHALF)
        {
            if(currStatusCAHALF == 0)
            {
                i--; 
                k++;
                if(i == (BOOTSTRAP_CHARGING_COUNTS - 50))
                {
                    PG1IOCONLbits.OVRENL = 0;
                }
                else if(i == (BOOTSTRAP_CHARGING_COUNTS - 150))
                {
                    PG2IOCONLbits.OVRENL = 0;  
                }
                else if(i == (BOOTSTRAP_CHARGING_COUNTS - 250))
                {
                    PG4IOCONLbits.OVRENL = 0;  
                }
                if(k > 25)
                {
                    if(PG4IOCONLbits.OVRENL == 0)
                    {
                        if(PWM_PDC3 > 2)
                        {
                            PWM_PDC3 -= 2;
                        }
                        else
                        {
                           PWM_PDC3 = 0; 
                        }
                    }
                    if(PG2IOCONLbits.OVRENL == 0)
                    {
                        if(PWM_PDC2 > 2)
                        {
                            PWM_PDC2 -= 2;
                        }
                        else
                        {
                            PWM_PDC2 = 0; 
                        }
                    }
                    if(PG1IOCONLbits.OVRENL == 0)
                    {
                        if(PWM_PDC1 > 2)
                        {
                            PWM_PDC1 -= 2;
                        }
                        else
                        {
                            PWM_PDC1 = 0; 
                        }
                    }
                    k = 0;
                }
            }
        }
    }

    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0;

    PG4IOCONLbits.OVRENH = 0;  // 0 = PWM generator provides data for PWM4H pin
    PG2IOCONLbits.OVRENH = 0;  // 0 = PWM generator provides data for PWM2H pin
    PG1IOCONLbits.OVRENH = 0;  // 0 = PWM generator provides data for PWM1H pin
}

void InitPWMGenerator1 (void)
{

    PG1CONL      = 0x0000;
    PG1CONLbits.ON = 0;
    PG1CONLbits.CLKSEL = 1;
#ifdef SINGLE_SHUNT
    PG1CONLbits.MODSEL = 6;
#else
    PG1CONLbits.MODSEL = 4;
#endif     
    PG1CONLbits.TRGCNT = 0;
    
    PG1CONH      = 0x0000;
    PG1CONHbits.MDCSEL = 0;
    PG1CONHbits.MPERSEL = 1;
    PG1CONHbits.MPHSEL = 0;
    PG1CONHbits.MSTEN = 1;
    PG1CONHbits.UPDMOD = 0;
    PG1CONHbits.TRGMOD = 0;
    PG1CONHbits.SOCS = 0;
    
    PG1STAT      = 0x0000;
    PG1IOCONL    = 0x0000;

    PG1IOCONLbits.CLMOD = 0;
    PG1IOCONLbits.SWAP = 0;
    PG1IOCONLbits.OVRENH = 0;
    PG1IOCONLbits.OVRENL = 0;
    PG1IOCONLbits.OVRDAT = 0;
    PG1IOCONLbits.OSYNC = 0;
    PG1IOCONLbits.FLTDAT = 0;
    PG1IOCONLbits.CLDAT = 0;
    PG1IOCONLbits.FFDAT = 0;
    PG1IOCONLbits.DBDAT = 0;

    PG1IOCONH    = 0x0000;
    PG1IOCONHbits.CAPSRC = 0;
    PG1IOCONHbits.DTCMPSEL = 0;
    PG1IOCONHbits.PMOD = 0;
    PG1IOCONHbits.PENH = 1;
    PG1IOCONHbits.PENL = 1;
    PG1IOCONHbits.POLH = 0;
    PG1IOCONHbits.POLL = 0;
    
    PG1EVTL      = 0x0000;
    PG1EVTLbits.ADTR1PS = 0;
    PG1EVTLbits.ADTR1EN3  = 0;
    PG1EVTLbits.ADTR1EN2 = 0;
    PG1EVTLbits.ADTR1EN1 = 1;
    PG1EVTLbits.UPDTRG = 1;
    PG1EVTLbits.PGTRGSEL = 0;
    
    PG1EVTH      = 0x0000;
    PG1EVTHbits.FLTIEN = 1;
    PG1EVTHbits.CLIEN = 0;
    PG1EVTHbits.FFIEN = 0;
    PG1EVTHbits.SIEN = 0;
    PG1EVTHbits.IEVTSEL = 3;    
#ifdef SINGLE_SHUNT
    PG1EVTHbits.ADTR2EN3 = 1;
    PG1EVTHbits.ADTR2EN2 = 1;
#else
    PG1EVTHbits.ADTR2EN3 = 0;
    PG1EVTHbits.ADTR2EN2 = 0;
#endif
    PG1EVTHbits.ADTR2EN1 = 0;
    PG1EVTHbits.ADTR1OFS = 0;
    
#ifndef ENABLE_PWM_FAULT
    PG1FPCIL     = 0x0000;
    PG1FPCIH     = 0x0000;
#else
    PG1FPCIL     = 0x0000;
    PG1FPCILbits.TSYNCDIS = 0;
    PG1FPCILbits.TERM = 0;
    PG1FPCILbits.AQPS = 0;
    PG1FPCILbits.AQSS = 0;
    PG1FPCILbits.PSYNC = 0;
    PG1FPCILbits.PPS = 0;
    PG1FPCILbits.PSS = 0b11011; //PCI Source : Comparator 1 output
    
    PG1FPCIH     = 0x0000;
    PG1FPCIHbits.BPEN   = 0;
    PG1FPCIHbits.BPSEL   = 0;
    PG1FPCIHbits.ACP   = 3;
    PG1FPCIHbits.PCIGT  = 0;
    PG1FPCIHbits.TQPS   = 0;
    PG1FPCIHbits.TQSS  = 3;
#endif    

    PG1CLPCIL    = 0x0000;
    PG1CLPCIH    = 0x0000;
    PG1FFPCIL    = 0x0000;
    PG1FFPCIH    = 0x0000;
    PG1SPCIL     = 0x0000;
    PG1SPCIH     = 0x0000;
    
    PG1LEBL      = 0x0000;
    PG1LEBH      = 0x0000;
    
    PG1PHASE     = 0x0000;
    PG1DC        = MIN_DUTY;
    PG1DCA       = 0x0000;
    PG1PER       = 0x0000;
    PG1DTL       = DDEADTIME;
    PG1DTH       = DDEADTIME;

    PG1TRIGA     = ADC_SAMPLING_POINT;
    PG1TRIGB     = 0x0000;
    PG1TRIGC     = 0x0000;
    
}

void InitPWMGenerator2 (void)
{

    PG2CONL      = 0x0000;
    PG2CONLbits.ON = 0;
    PG2CONLbits.CLKSEL = 1;
#ifdef SINGLE_SHUNT
    PG2CONLbits.MODSEL = 6;
#else
    PG2CONLbits.MODSEL = 4;
#endif 
    PG2CONLbits.TRGCNT = 0;
    
    PG2CONH      = 0x0000;
    PG2CONHbits.MDCSEL = 0;
    PG2CONHbits.MPERSEL = 1;
    PG2CONHbits.MPHSEL = 0;
    PG2CONHbits.MSTEN = 0;
	PG2CONHbits.UPDMOD = 0b010;
    PG2CONHbits.TRGMOD = 0;
    PG2CONHbits.SOCS = 1;
    
    PG2STAT      = 0x0000;
    PG2IOCONL    = 0x0000;

    PG2IOCONLbits.CLMOD = 0;
    PG2IOCONLbits.SWAP = 0;
    PG2IOCONLbits.OVRENH = 0;
    PG2IOCONLbits.OVRENL = 0;
    PG2IOCONLbits.OVRDAT = 0;
    PG2IOCONLbits.OSYNC = 0;
    PG2IOCONLbits.FLTDAT = 0;
    PG2IOCONLbits.CLDAT = 0;
    PG2IOCONLbits.FFDAT = 0;
    PG2IOCONLbits.DBDAT = 0;

    PG2IOCONH    = 0x0000;
    PG2IOCONHbits.CAPSRC = 0;
    PG2IOCONHbits.DTCMPSEL = 0;
    PG2IOCONHbits.PMOD = 0;
    PG2IOCONHbits.PENH = 1;
    PG2IOCONHbits.PENL = 1;
    PG2IOCONHbits.POLH = 0;
    PG2IOCONHbits.POLL = 0;

    PG2EVTL      = 0x0000;
    PG2EVTLbits.ADTR1PS = 0;
    PG2EVTLbits.ADTR1EN3  = 0;
    PG2EVTLbits.ADTR1EN2 = 0;
    PG2EVTLbits.ADTR1EN1 = 0;
    PG2EVTLbits.UPDTRG = 0;
    PG2EVTLbits.PGTRGSEL = 0;

    PG2EVTH      = 0x0000;
    PG2EVTHbits.FLTIEN = 0;
    PG2EVTHbits.CLIEN = 0;
    PG2EVTHbits.FFIEN = 0;
    PG2EVTHbits.SIEN = 0;
    PG2EVTHbits.IEVTSEL = 3;
    PG2EVTHbits.ADTR2EN3 = 0;
    PG2EVTHbits.ADTR2EN2 = 0;
    PG2EVTHbits.ADTR2EN1 = 0;
    PG2EVTHbits.ADTR1OFS = 0;

#ifndef ENABLE_PWM_FAULT
    PG2FPCIL     = 0x0000;
    PG2FPCIH     = 0x0000;
#else
    PG2FPCIL     = 0x0000;
    PG2FPCILbits.TSYNCDIS = 0;
    PG2FPCILbits.TERM = 0;
    PG2FPCILbits.AQPS = 0;
    PG2FPCILbits.AQSS = 0;
    PG2FPCILbits.PSYNC = 0;
    PG2FPCILbits.PPS = 0;
    PG2FPCILbits.PSS = 0b11011; //PCI Source : Comparator 1 output

    PG2FPCIH     = 0x0000;
    PG2FPCIHbits.BPEN   = 0;
    PG2FPCIHbits.BPSEL   = 0;
    PG2FPCIHbits.ACP   = 3;
    PG2FPCIHbits.PCIGT  = 0;
    PG2FPCIHbits.TQPS   = 0;
    PG2FPCIHbits.TQSS  = 3;
#endif

    PG2CLPCIL    = 0x0000;
    PG2CLPCIH    = 0x0000;
    PG2FFPCIL    = 0x0000;
    PG2FFPCIH    = 0x0000;
    PG2SPCIL     = 0x0000;
    PG2SPCIH     = 0x0000;
    PG2LEBL      = 0x0000;
    PG2LEBH      = 0x0000;

    PG2PHASE     = 0x0000;
    PG2DC        = MIN_DUTY;
    PG2DCA       = 0x0000;
    PG2PER       = 0x0000;
    PG2DTL       = DDEADTIME;
    PG2DTH       = DDEADTIME;

    PG2TRIGA     = 0x0000;
    PG2TRIGB     = 0x0000;
    PG2TRIGC     = 0x0000;

}

void InitPWMGenerator4 (void)
{

    PG4CONL      = 0x0000;
    PG4CONLbits.ON = 0;
    PG4CONLbits.CLKSEL = 1;
#ifdef SINGLE_SHUNT
    PG4CONLbits.MODSEL = 6;
#else
    PG4CONLbits.MODSEL = 4;
#endif  
    PG4CONLbits.TRGCNT = 0;

    PG4CONH      = 0x0000;
    PG4CONHbits.MDCSEL = 0;
    PG4CONHbits.MPERSEL = 1;
    PG4CONHbits.MPHSEL = 0;
    PG4CONHbits.MSTEN = 0;
    PG4CONHbits.UPDMOD = 0b010;
    PG4CONHbits.TRGMOD = 0;
    PG4CONHbits.SOCS = 1;

    PG4STAT      = 0x0000;
    PG4IOCONL    = 0x0000;

    PG4IOCONLbits.CLMOD = 0;
    PG4IOCONLbits.SWAP = 0;
    PG4IOCONLbits.OVRENH = 0;
    PG4IOCONLbits.OVRENL = 0;
    PG4IOCONLbits.OVRDAT = 0;
    PG4IOCONLbits.OSYNC = 0;
    PG4IOCONLbits.FLTDAT = 0;
    PG4IOCONLbits.CLDAT = 0;
    PG4IOCONLbits.FFDAT = 0;
    PG4IOCONLbits.DBDAT = 0;

    PG4IOCONH    = 0x0000;
    PG4IOCONHbits.CAPSRC = 0;
    PG4IOCONHbits.DTCMPSEL = 0;
    PG4IOCONHbits.PMOD = 0;
    PG4IOCONHbits.PENH = 1;
    PG4IOCONHbits.PENL = 1;
    PG4IOCONHbits.POLH = 0;
    PG4IOCONHbits.POLL = 0;

    PG4EVTL      = 0x0000;
    PG4EVTLbits.ADTR1PS = 0;
    PG4EVTLbits.ADTR1EN3  = 0;
    PG4EVTLbits.ADTR1EN2 = 0;
    PG4EVTLbits.ADTR1EN1 = 0;
    PG4EVTLbits.UPDTRG = 0;
    PG4EVTLbits.PGTRGSEL = 0;

    PG4EVTH      = 0x0000;
    PG4EVTHbits.FLTIEN = 0;
    PG4EVTHbits.CLIEN = 0;
    PG4EVTHbits.FFIEN = 0;
    PG4EVTHbits.SIEN = 0;
    PG4EVTHbits.IEVTSEL = 3;
    PG4EVTHbits.ADTR2EN3 = 0;
    PG4EVTHbits.ADTR2EN2 = 0;
    PG4EVTHbits.ADTR2EN1 = 0;
    PG4EVTHbits.ADTR1OFS = 0;

#ifndef ENABLE_PWM_FAULT
    PG4FPCIL     = 0x0000;
    PG4FPCIH     = 0x0000;
#else
    PG4FPCIL     = 0x0000;
    PG4FPCILbits.TSYNCDIS = 0;
    PG4FPCILbits.TERM = 0;
    PG4FPCILbits.AQPS = 0;
    PG4FPCILbits.AQSS = 0;
    PG4FPCILbits.PSYNC = 0;
    PG4FPCILbits.PPS = 0;
    PG4FPCILbits.PSS = 0b11011;

    PG4FPCIH     = 0x0000;
    PG4FPCIHbits.BPEN   = 0;
    PG4FPCIHbits.BPSEL   = 0;
    PG4FPCIHbits.ACP   = 3;

    PG4FPCIHbits.PCIGT  = 0;

    PG4FPCIHbits.TQPS   = 0;

    PG4FPCIHbits.TQSS  = 3;
#endif

    PG4CLPCIL    = 0x0000;
    PG4CLPCIH    = 0x0000;
    PG4FFPCIL    = 0x0000;
    PG4FFPCIH    = 0x0000;
    PG4SPCIL     = 0x0000;
    PG4SPCIH     = 0x0000;

    PG4LEBL      = 0x0000;
    PG4LEBH      = 0x0000;

    PG4PHASE     = 0x0000;
    PG4DC        = MIN_DUTY;
    PG4DCA       = 0x0000;
    PG4PER       = 0x0000;
    PG4DTL       = DDEADTIME;
    PG4DTH       = DDEADTIME;

    PG4TRIGA     = 0x0000;
    PG4TRIGB     = 0x0000;
    PG4TRIGC     = 0x0000;

}