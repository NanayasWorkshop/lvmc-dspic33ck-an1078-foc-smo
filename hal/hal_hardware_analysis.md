# Hardware Abstraction Layer Analysis

## System Hardware Overview

**Target MCU**: dsPIC33CK256MP508 at 200MHz (100MHz peripheral clock)
**PWM Frequency**: 20kHz center-aligned with dead time
**ADC Strategy**: Multi-core simultaneous sampling
**Current Sensing**: Single shunt with optional internal op-amps

## Clock Configuration

### Primary Clock Setup
```c
FOSC = 200MHz    // System clock
FCY = 100MHz     // Peripheral clock (instruction cycle)
```

**PLL Configuration:**
- **Input**: 8MHz FRC oscillator
- **PLL Multiplier**: 150x (8MHz × 150 = 1200MHz VCO)
- **Post-scalers**: ÷3, ÷2 → 200MHz output
- **Peripheral Clock**: FOSC ÷ 2 = 100MHz

### Timing Relationships
```c
LOOPTIME_TCY = 4999        // 50μs @ 100MHz = 5000 cycles - 1
DDEADTIME = 200           // 2μs @ 100MHz = 200 cycles
ADC_SAMPLING_POINT = 0    // Sample at PWM center
```

## PWM Generation Strategy

### Multi-Generator Architecture
**Three PWM generators** for 3-phase control:
- **PG1 (Master)**: PWM1H/L, generates ADC triggers, master timing
- **PG2 (Slave)**: PWM2H/L, synchronized to PG1
- **PG4 (Slave)**: PWM4H/L, synchronized to PG1

### PWM Mode Selection
```c
#ifdef SINGLE_SHUNT
    PG1CONLbits.MODSEL = 6;  // Dual Edge Center-Aligned PWM mode
#else
    PG1CONLbits.MODSEL = 4;  // Center-Aligned PWM mode
#endif
```

**Single Shunt**: Dual-edge center-aligned mode (MODSEL = 6)
- Allows independent duty cycle control for up/down counting
- Enables dual duty cycle compensation algorithm

**Dual Shunt**: Standard center-aligned mode (MODSEL = 4)
- Single duty cycle per PWM period
- Simplified timing, simultaneous current sampling

### Master-Slave Synchronization
```c
// PG1 - Master Configuration
PG1CONHbits.MSTEN = 1;     // Broadcasts update signals
PG1CONHbits.SOCS = 0;      // Local EOC timing

// PG2, PG4 - Slave Configuration  
PG2CONHbits.MSTEN = 0;     // Receives update signals
PG2CONHbits.UPDMOD = 0b010; // Slaved SOC update mode
PG2CONHbits.SOCS = 1;      // PWM1 trigger selected
```

### Key PWM Features
- **Dead time**: 2μs on all phases prevents shoot-through
- **Center-aligned**: Minimizes current ripple and EMI
- **Master-slave sync**: Ensures perfect phase alignment
- **Fault protection**: Hardware overcurrent shutdown

## Bootstrap Capacitor Charging

### Sophisticated Startup Sequence

**Problem**: Gate driver bootstrap capacitors need controlled charging to prevent damage.

**Solution**: Multi-phase charging with gradual duty cycle progression and staggered enable sequence.

### Charging Algorithm Implementation
```c
#define BOOTSTRAP_CHARGING_COUNTS 400  // ~20ms charging time at 20kHz PWM
```

**Phase-by-Phase Enable Sequence:**
```c
while(i > 0) {
    // Wait for PWM counter direction change
    currStatusCAHALF = PG1STATbits.CAHALF;
    if(prevStatusCAHALF != currStatusCAHALF && currStatusCAHALF == 0) {
        i--;  // Decrement counter
        
        // Staggered enable sequence
        if(i == (BOOTSTRAP_CHARGING_COUNTS - 50)) {
            PG1IOCONLbits.OVRENL = 0;  // Enable Phase 1 low-side PWM
        }
        else if(i == (BOOTSTRAP_CHARGING_COUNTS - 150)) {
            PG2IOCONLbits.OVRENL = 0;  // Enable Phase 2 low-side PWM  
        }
        else if(i == (BOOTSTRAP_CHARGING_COUNTS - 250)) {
            PG4IOCONLbits.OVRENL = 0;  // Enable Phase 4 low-side PWM
        }
```

**Gradual Duty Cycle Reduction:**
```c
        // Every 25 PWM half-cycles, reduce duty gradually
        if(k > 25) {
            if(PG4IOCONLbits.OVRENL == 0) {
                PWM_PDC3 = (PWM_PDC3 > 2) ? (PWM_PDC3 - 2) : 0;
            }
            if(PG2IOCONLbits.OVRENL == 0) {
                PWM_PDC2 = (PWM_PDC2 > 2) ? (PWM_PDC2 - 2) : 0;
            }
            if(PG1IOCONLbits.OVRENL == 0) {
                PWM_PDC1 = (PWM_PDC1 > 2) ? (PWM_PDC1 - 2) : 0;
            }
            k = 0;
        }
    }
}
```

### Initial Charging Setup
```c
// Start with high duty cycle for rapid initial charging
PWM_PDC3 = LOOPTIME_TCY - (DDEADTIME/2 + 5);  // ~98% duty cycle
PWM_PDC2 = LOOPTIME_TCY - (DDEADTIME/2 + 5);
PWM_PDC1 = LOOPTIME_TCY - (DDEADTIME/2 + 5);

// Force all high-side switches OFF during charging
PG4IOCONLbits.OVRDAT = 0;   // Override data = LOW
PG2IOCONLbits.OVRDAT = 0;
PG1IOCONLbits.OVRDAT = 0;

PG4IOCONLbits.OVRENH = 1;   // Enable override for high-side
PG2IOCONLbits.OVRENH = 1;
PG1IOCONLbits.OVRENH = 1;
```

### Hardware Protection Features
- **Override control**: High-side switches forced OFF via `OVRENH = 1`
- **Sequential enable**: Low-side switches enabled one by one (50, 150, 250 cycle delays)
- **Gradual reduction**: Duty cycles reduced in 2-count steps every 25 cycles
- **Direction monitoring**: Uses `CAHALF` bit to synchronize with PWM counter

## ADC Configuration & Timing

### Multi-Core ADC Architecture
**Three ADC cores** for simultaneous sampling:

**Core Assignment:**
```c
// Dedicated Core 0: Motor phase currents
ADCON4Hbits.C0CHS = 0;  // AN0 (Phase A current)
ADCORE0Hbits.RES = 3;   // 12-bit resolution
ADCORE0Lbits.SAMC = 8;  // 9 TADCORE sample time

// Dedicated Core 1: Reserved for expansion  
ADCON4Hbits.C1CHS = 0;  // AN1 (Phase B current)
ADCORE1Hbits.RES = 3;   // 12-bit resolution
ADCORE1Lbits.SAMC = 8;  // 9 TADCORE sample time

// Shared Core: Auxiliary signals
ADCON2Hbits.SHRSAMC = 15; // 16 TADCORE sample time
ADCON1Hbits.SHRRES = 3;   // 12-bit resolution
```

### ADC Power Sequencing
```c
// Turn on ADC module first
ADCON1Lbits.ADON = 1;

// Power up and enable each core sequentially
ADCON5Lbits.C0PWR = 1;           // Power up Core 0
while(ADCON5Lbits.C0RDY == 0);   // Wait for ready
ADCON3Hbits.C0EN = 1;            // Enable Core 0

ADCON5Lbits.C1PWR = 1;           // Power up Core 1  
while(ADCON5Lbits.C1RDY == 0);   // Wait for ready
ADCON3Hbits.C1EN = 1;            // Enable Core 1

ADCON5Lbits.SHRPWR = 1;          // Power up Shared Core
while(ADCON5Lbits.SHRRDY == 0);  // Wait for ready  
ADCON3Hbits.SHREN = 1;           // Enable Shared Core
```

### ADC Trigger Source Configuration

**Single Shunt Mode - Multiple Triggers:**
```c
// Phase current offset calibration (when motor stopped)
ADTRIG0Lbits.TRGSRC0 = 0x4;  // AN0: PWM1 Trigger 1 (TRIGA)
ADTRIG0Lbits.TRGSRC1 = 0x4;  // AN1: PWM1 Trigger 1 (TRIGA)

// Bus current sampling (when motor running)  
ADTRIG1Lbits.TRGSRC4 = 0x5;  // AN4: PWM1 Trigger 2 (TRIGB)

// Auxiliary measurements (software triggered)
ADTRIG2Hbits.TRGSRC11 = 0x1; // AN11 (pot): Software trigger
ADTRIG3Lbits.TRGSRC12 = 0x1; // AN12 (temp): Software trigger  
ADTRIG3Hbits.TRGSRC15 = 0x1; // AN15 (Vbus): Software trigger
```

**Dual Shunt Mode - Synchronized Triggers:**
```c
// All motor-related measurements on PWM1 Trigger 1
ADTRIG0Lbits.TRGSRC0 = 0x4;   // AN0: PWM1 Trigger 1
ADTRIG0Lbits.TRGSRC1 = 0x4;   // AN1: PWM1 Trigger 1
ADTRIG2Hbits.TRGSRC11 = 0x4;  // AN11: PWM1 Trigger 1 (if needed)
```

### ADC Interrupt Configuration

**Single Shunt Interrupt Handling:**
```c
#ifdef SINGLE_SHUNT       
 #define EnableADCInterrupt()   _ADCAN4IE = 1   // Bus current trigger
 #define DisableADCInterrupt()  _ADCAN4IE = 0
 #define ClearADCIF()           _ADCAN4IF = 0
 #define _ADCInterrupt          _ADCAN4Interrupt  
#else
 #define EnableADCInterrupt()   _ADCAN11IE = 1  // Potentiometer trigger
 #define DisableADCInterrupt()  _ADCAN11IE = 0
 #define ClearADCIF()           _ADCAN11IF = 0
 #define _ADCInterrupt          _ADCAN11Interrupt  
#endif
```

### ADC Data Format and Scaling
```c
// Signed conversion for current measurements
ADMOD0Lbits.SIGN4 = 1;  // AN4 (Bus current) - signed
ADMOD0Lbits.SIGN1 = 1;  // AN1 (Phase B) - signed  
ADMOD0Lbits.SIGN0 = 1;  // AN0 (Phase A) - signed

// Unsigned conversion for auxiliary measurements
ADMOD0Hbits.SIGN11 = 0; // AN11 (Potentiometer) - unsigned
ADMOD0Hbits.SIGN12 = 0; // AN12 (Temperature) - unsigned
ADMOD0Hbits.SIGN15 = 0; // AN15 (DC bus voltage) - unsigned

// Data access with polarity correction
#define ADCBUF_IPHASE1    -ADCBUF0  // Invert for correct polarity
#define ADCBUF_IPHASE2    -ADCBUF1  // Invert for correct polarity
#define ADCBUF_IBUS       ADCBUF4   // Direct reading
```

### Noise Mitigation Strategy
- **Center sampling**: ADC triggers at PWM valley (minimum switching noise)
- **Dedicated cores**: Isolated analog supplies per core
- **Sample delay**: Accounts for switching transients in single shunt mode
- **Fractional format**: `ADCON1Hbits.FORM = 1` for direct Q15 compatibility

## Current Sensing Hardware

### Single Shunt Configuration
**Bus current sensor** on DC link with precision timing:
```c
// Primary measurement point
ADCBUF_IBUS = ADCBUF4      // DC bus current (main sensing)

// Phase current buffers (for offset calibration when motor stopped)
ADCBUF_IPHASE1 = -ADCBUF0  // Phase A (offset cal only)
ADCBUF_IPHASE2 = -ADCBUF1  // Phase B (offset cal only)
```

### Internal Op-Amp Configuration
**Three op-amps** for current amplification:

**Op-Amp Power and Control:**
```c
#ifdef INTERNAL_OPAMP_CONFIG
    // Enable wide input range for all op-amps
    AMPCON1Hbits.NCHDIS1 = 0;  // Wide input range enabled
    AMPCON1Hbits.NCHDIS2 = 0;
    AMPCON1Hbits.NCHDIS3 = 0;
    
    // Enable individual op-amps
    AMPCON1Lbits.AMPEN1 = 1;   // Op-amp 1 enabled
    AMPCON1Lbits.AMPEN2 = 1;   // Op-amp 2 enabled  
    AMPCON1Lbits.AMPEN3 = 1;   // Op-amp 3 enabled
    
    // Master enable
    AMPCON1Lbits.AMPON = 1;    // Global op-amp enable
#endif
```

**Op-Amp Signal Routing:**

**Op-Amp 1 (Phase A Current):**
```c
// Input pins
ANSELAbits.ANSELA2 = 1;  // RA2 - Positive input (AN9)
TRISAbits.TRISA2 = 1;    // Input mode
ANSELAbits.ANSELA1 = 1;  // RA1 - Negative input (ANA1)  
TRISAbits.TRISA1 = 1;    // Input mode

// Output pin
ANSELAbits.ANSELA0 = 1;  // RA0 - Output to AN0
TRISAbits.TRISA0 = 1;    // Input mode (ADC input)
```

**Op-Amp 2 (Phase B Current):**
```c
// Input pins
ANSELBbits.ANSELB4 = 1;  // RB4 - Positive input
TRISBbits.TRISB4 = 1;    // Input mode
ANSELBbits.ANSELB3 = 1;  // RB3 - Negative input
TRISBbits.TRISB3 = 1;    // Input mode

// Output pin  
ANSELBbits.ANSELB2 = 1;  // RB2 - Output to AN1
TRISBbits.TRISB2 = 1;    // Input mode (ADC input)
```

**Op-Amp 3 (Bus Current):**
```c
// Input pins
ANSELCbits.ANSELC2 = 1;  // RC2 - Positive input (AN14)
TRISCbits.TRISC2 = 1;    // Input mode
ANSELCbits.ANSELC1 = 1;  // RC1 - Negative input (AN13)
TRISCbits.TRISC1 = 1;    // Input mode

// Output pin
ANSELAbits.ANSELA4 = 1;  // RA4 - Output to AN4  
TRISAbits.TRISA4 = 1;    // Input mode (ADC input)
```

## GPIO Hardware Mapping

### Motor Control Outputs
```c
// PWM Phase A
TRISBbits.TRISB14 = 0;   // RB14 (Pin 1) - PWM1H output
TRISBbits.TRISB15 = 0;   // RB15 (Pin 3) - PWM1L output

// PWM Phase B  
TRISBbits.TRISB12 = 0;   // RB12 (Pin 78) - PWM2H output
TRISBbits.TRISB13 = 0;   // RB13 (Pin 80) - PWM2L output

// PWM Phase C
TRISDbits.TRISD1 = 0;    // RD1 (Pin 73) - PWM4H output
TRISDbits.TRISD0 = 0;    // RD0 (Pin 74) - PWM4L output
```

### User Interface Hardware
```c
// Status LEDs (outputs)
TRISEbits.TRISE6 = 0;    // RE6 (Pin 37) - LED1 (system status)
TRISEbits.TRISE7 = 0;    // RE7 (Pin 39) - LED2 (motor running)

// Control Buttons (inputs)  
TRISEbits.TRISE11 = 1;   // RE11 (Pin 59) - SW1 (start/stop)
TRISEbits.TRISE12 = 1;   // RE12 (Pin 62) - SW2 (speed mode)

// Button definitions for application use
#define SW1                   PORTEbits.RE11
#define SW2                   PORTEbits.RE12
#define BUTTON_START_STOP     SW1
#define BUTTON_SPEED_HALF_DOUBLE SW2

#define LED1                  LATEbits.LATE6
#define LED2                  LATEbits.LATE7
```

### Analog Input Configuration
```c
// Motor current sensing
ANSELAbits.ANSELA0 = 1;   // AN0: Phase A current (or op-amp output)
TRISAbits.TRISA0 = 1;     // Input mode
ANSELBbits.ANSELB2 = 1;   // AN1: Phase B current (or op-amp output)  
TRISBbits.TRISB2 = 1;     // Input mode
ANSELAbits.ANSELA4 = 1;   // AN4: Bus current (or op-amp output)
TRISAbits.TRISA4 = 1;     // Input mode

// System monitoring
ANSELBbits.ANSELB9 = 1;   // AN11: Speed reference potentiometer
TRISBbits.TRISB9 = 1;     // Input mode
ANSELCbits.ANSELC0 = 1;   // AN12: Temperature sensor
TRISCbits.TRISC0 = 1;     // Input mode
ANSELCbits.ANSELC3 = 1;   // AN15: DC bus voltage
TRISCbits.TRISC3 = 1;     // Input mode
```

### Communication Interface
```c
// UART for diagnostics/tuning
_U1RXR = 78;             // UART RX mapped to RP78 (RD14, Pin 13)
_RP77R = 0b000001;       // UART TX mapped to RP77 (RD13, Pin 14)
```

## Interrupt Architecture

### Priority Hierarchy
```c
Priority 7: ADC Current Sampling     // Highest - real-time critical
Priority 7: PWM Fault Detection      // Highest - safety critical  
Priority 1: UART Communication       // Lowest - non-real-time
```

### ADC Interrupt Strategy

**Single Shunt Dual Interrupt System:**
```c
void _ADCAN4Interrupt() {  // Bus current sampling
    if (IFS4bits.PWM1IF == 1) {
        singleShuntParam.adcSamplePoint = 0;  // Reset sample sequence
        IFS4bits.PWM1IF = 0;                  // Clear PWM interrupt
    }
    
    switch(singleShuntParam.adcSamplePoint) {
        case SS_SAMPLE_BUS1:
            singleShuntParam.adcSamplePoint = 1;
            // Store first bus current sample
            singleShuntParam.Ibus1 = ADCBUF_IBUS - offsetIbus;
            break;
            
        case SS_SAMPLE_BUS2:
            PWM_TRIGA = ADC_SAMPLING_POINT;  // Reset trigger
            singleShuntParam.adcSamplePoint = 0;
            // Store second bus current sample  
            singleShuntParam.Ibus2 = ADCBUF_IBUS - offsetIbus;
            ADCON3Lbits.SWCTRG = 1;  // Trigger auxiliary measurements
            
            // Execute full control loop only on second sample
            if (uGF.bits.RunMotor) {
                // Current reconstruction, FOC algorithm, PWM update
            }
            break;
    }
}
```

**Dual Shunt Single Interrupt System:**
```c
void _ADCAN11Interrupt() {  // Phase current sampling  
    // Read phase currents directly
    measureInputs.current.Ia = ADCBUF_IPHASE1;
    measureInputs.current.Ib = ADCBUF_IPHASE2;
    
    if (uGF.bits.RunMotor) {
        // Execute full control loop every PWM cycle
    }
}
```

## Fault Protection System

### Hardware Overcurrent Protection
**Analog comparator** with programmable DAC reference:

**Comparator Configuration:**
```c
// DAC setup for current threshold
DACCTRL1Lbits.DACON = 1;     // Enable DAC
DACCTRL1Lbits.CLKSEL = 3;    // Clock selection
DACCTRL1Lbits.CLKDIV = 1;    // Clock divider

// Comparator 1 setup
DAC1CONLbits.DACEN = 1;      // Enable comparator
DAC1CONLbits.FLTREN = 1;     // Enable filter  
DAC1CONLbits.INSEL = 2;      // Input selection
DAC1CONLbits.HYSSEL = 0b11;  // Hysteresis setting

// Set threshold value (3A normalized)
uint16_t cmpReference = (Q15_OVER_CURRENT_THRESHOLD * 2047) >> 15;
cmpReference += 2048;  // Offset for bipolar signal
DAC1DATH = cmpReference;
```

### PWM Fault Configuration
**Hardware shutdown on overcurrent:**
```c
#ifdef ENABLE_PWM_FAULT
// PG1 Fault Configuration (Master)
PG1FPCILbits.PSS = 0b11011;    // PCI Source: Comparator 1 output
PG1FPCILbits.ACP = 3;          // Latched acceptance mode
PG1FPCIHbits.TQSS = 3;         // Termination qualifier

// Slave generators inherit fault from master
PG2FPCILbits.PSS = 0b11011;    // Same comparator source
PG4FPCILbits.PSS = 0b11011;    // Same comparator source
#endif
```

### Software Fault Handling
```c
void _PWMInterrupt() {         // Fault recovery ISR
    ResetParameters();         // Reset all control variables
    ClearPWMPCIFault();       // Clear hardware fault flags
    ClearPWMIF();             // Clear interrupt flag
    
    // System automatically restarts when fault condition clears
}
```

**Manual Fault Clearing:**
```c
void ClearPWMPCIFault(void) {
    PG1FPCILbits.SWTERM = 1;   // Software termination
    PG2FPCILbits.SWTERM = 1;
    PG4FPCILbits.SWTERM = 1;
}
```

## Button Interface and Debouncing

### Button Hardware Configuration
```c
typedef struct {
    BUTTON_STATE state;        // Current button state
    uint16_t debounceCount;    // Debounce counter
    bool logicState;           // Physical pin state
    bool status;               // Processed button event
} BUTTON_T;

typedef enum {
    BUTTON_NOT_PRESSED = 0,
    BUTTON_PRESSED = 1, 
    BUTTON_DEBOUNCE = 2
} BUTTON_STATE;
```

### Debouncing Algorithm
```c
void ButtonScan(BUTTON_T *pButton, bool button) {
    if (button == true) {  // Button physically pressed (active low)
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) {
            pButton->debounceCount--;
            pButton->state = BUTTON_DEBOUNCE;
        }
    } else {  // Button released
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) {
            pButton->state = BUTTON_NOT_PRESSED;
        } else {
            pButton->state = BUTTON_PRESSED;
            pButton->status = true;  // Signal button press event
        }
        pButton->debounceCount = 0;
    }
}
```

**Button Service Timing:**
```c
#define BUTTON_DEBOUNCE_COUNT 40           // 40ms debounce time
#define BOARD_SERVICE_TICK_COUNT (PWMFREQUENCY_HZ/1000)  // 1kHz service rate

// Called every PWM cycle
void BoardServiceStepIsr(void) {
    if (boardServiceISRCounter < BOARD_SERVICE_TICK_COUNT) {
        boardServiceISRCounter++;
    }
}

// Called from main loop
void BoardService(void) {
    if (boardServiceISRCounter == BOARD_SERVICE_TICK_COUNT) {
        ButtonScan(&buttonStartStop, BUTTON_START_STOP);
        ButtonScan(&buttonSpeedHalfDouble, BUTTON_SPEED_HALF_DOUBLE);
        boardServiceISRCounter = 0;
    }
}
```

## Diagnostic Integration

### X2CScope Real-Time Debugging
**UART-based real-time monitoring** for motor parameters:

**UART Configuration for Diagnostics:**
```c
#define X2C_BAUDRATE_DIVIDER 54  // Configurable baud rate

void DiagnosticsInit(void) {
    // Configure UART1 for X2CScope
    UART1_InterruptReceiveDisable();
    UART1_InterruptTransmitDisable();
    UART1_Initialize();
    UART1_BaudRateDividerSet(X2C_BAUDRATE_DIVIDER);
    UART1_SpeedModeStandard();
    UART1_ModuleEnable();
    
    // Initialize X2CScope with UART hooks
    X2CScope_Init();
}
```

**UART Function Hooks:**
```c
// Transmit function
static void X2CScope_sendSerial(uint8_t data) {
    UART1_DataWrite(data);
}

// Receive function with error handling
static uint8_t X2CScope_receiveSerial() {
    const uint16_t error_mask = _U1STA_OERR_MASK | _U1STA_FERR_MASK | _U1STA_PERR_MASK;
    if (UART1_StatusGet() & error_mask) {
        UART1_ReceiveBufferOverrunErrorFlagClear();
        return 0;  // Return dummy data on error
    }
    return UART1_DataRead();
}

// Status functions  
static uint8_t X2CScope_isReceiveDataAvailable() {
    return UART1_IsReceiveBufferDataReady();
}

static uint8_t X2CScope_isSendReady() {
    return !UART1_StatusBufferFullTransmitGet();
}
```

**Integration with Control Loop:**
```c
// Called every PWM cycle (20kHz)
void DiagnosticsStepIsr(void) {
    X2CScope_Update();  // Update scope data
}

// Called from main loop (lower priority)
void DiagnosticsStepMain(void) {
    X2CScope_Communicate();  // Handle UART communication
}
```

### Temperature Monitoring
**MOSFET temperature measurement with averaging:**
```c
#define MOSFET_TEMP_AVG_FILTER_SCALE 8         // 256-point average
#define OFFSET_COUNT_MOSFET_TEMP 4964          // ADC offset for 0°C
#define MOSFET_TEMP_COEFF Q15(0.010071108)     // °C per LSB

void MCAPP_MeasureTemperature(MCAPP_MEASURE_T *pData, int16_t input) {
    pData->MOSFETTemperature.input = input;
    pData->MOSFETTemperatureAvg = MCAPP_MeasureAvg(&pData->MOSFETTemperature);
    
    if (pData->MOSFETTemperature.status == 1) {
        // Convert to temperature in °C
        pData->MOSFETTemperatureAvg = 
            ((pData->MOSFETTemperatureAvg - OFFSET_COUNT_MOSFET_TEMP) * MOSFET_TEMP_COEFF) >> 15;
    }
}
```

## Power Management and System Control

### PWM Output Control
**Safe enable/disable of motor drive:**
```c
void EnablePWMOutputs(void) {
    // Remove override, allow PWM generators to control outputs
    PG4IOCONLbits.OVRENH = 0;  // PWM generator controls PWM4H
    PG4IOCONLbits.OVRENL = 0;  // PWM generator controls PWM4L
    PG2IOCONLbits.OVRENH = 0;  // PWM generator controls PWM2H
    PG2IOCONLbits.OVRENL = 0;  // PWM generator controls PWM2L
    PG1IOCONLbits.OVRENH = 0;  // PWM generator controls PWM1H
    PG1IOCONLbits.OVRENL = 0;  // PWM generator controls PWM1L
}

void DisablePWMOutputs(void) {
    // Force all outputs LOW via override
    PG4IOCONLbits.OVRDAT = 0;  // Override data = LOW
    PG2IOCONLbits.OVRDAT = 0;
    PG1IOCONLbits.OVRDAT = 0;
    
    // Enable override for all outputs
    PG4IOCONLbits.OVRENH = 1;  // Override controls PWM4H
    PG4IOCONLbits.OVRENL = 1;  // Override controls PWM4L
    PG2IOCONLbits.OVRENH = 1;  // Override controls PWM2H
    PG2IOCONLbits.OVRENL = 1;  // Override controls PWM2L
    PG1IOCONLbits.OVRENH = 1;  // Override controls PWM1H
    PG1IOCONLbits.OVRENL = 1;  // Override controls PWM1L
}
```

### Duty Cycle Limiting
**Hardware protection against illegal duty cycles:**
```c
void pwmDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *pPwmDutycycle, uint16_t min, uint16_t max) {
    // Phase 1 limiting
    if (pPwmDutycycle->dutycycle1 < min) {
        pPwmDutycycle->dutycycle1 = min;
    } else if (pPwmDutycycle->dutycycle1 > max) {
        pPwmDutycycle->dutycycle1 = max;
    }
    
    // Phase 2 limiting  
    if (pPwmDutycycle->dutycycle2 < min) {
        pPwmDutycycle->dutycycle2 = min;
    } else if (pPwmDutycycle->dutycycle2 > max) {
        pPwmDutycycle->dutycycle2 = max;
    }
    
    // Phase 3 limiting
    if (pPwmDutycycle->dutycycle3 < min) {
        pPwmDutycycle->dutycycle3 = min;
    } else if (pPwmDutycycle->dutycycle3 > max) {
        pPwmDutycycle->dutycycle3 = max;
    }
}

// Usage with dead time consideration
pwmDutyCycleLimitCheck(pPwmDutycycle, (DDEADTIME>>1), (LOOPTIME_TCY - (DDEADTIME>>1)));
```

## Hardware Timing Critical Paths

### Real-Time Constraints
```c
// Timing budgets for critical operations
ADC_Interrupt_Budget:    <25μs    // 50% of PWM period
PWM_Update_Deadline:     <50μs    // Must complete before next PWM period  
Bootstrap_Charging:      20ms     // Non-interruptible startup sequence
Current_Offset_Cal:      51.2ms   // 1024 samples × 50μs
```

### Performance Optimization Strategies

**Assembly Math Routines:**
- Park/Clarke transforms: ~2μs execution time
- PI controllers: ~1μs per controller
- Space vector calculation: ~3μs
- Sin/Cos calculation: ~1μs with lookup table

**Q15 Fixed-Point Benefits:**
- Eliminates floating-point overhead (~10x speed improvement)
- Maintains numerical precision for control algorithms
- Direct compatibility with DSP hardware acceleration

**Multi-Core ADC Optimization:**
- Parallel sampling reduces conversion time from 12μs to 4μs
- Dedicated cores eliminate cross-talk between measurements
- Independent trigger sources optimize sampling timing

**Single Shunt Timing Optimization:**
```c
// Dual duty cycle PWM allows precise current sampling windows
// Minimum window: 3μs (300 instruction cycles at 100MHz)
// Sample delay: 1μs (accounts for switching transients)
// Reconstruction time: <2μs (lookup table based on sector)
```

## Configuration Flexibility

### Compile-Time Hardware Selection
```c
// Current sensing configuration
#define SINGLE_SHUNT              // Enable single shunt algorithm
//#undef SINGLE_SHUNT            // Use dual shunt sensing

// Op-amp configuration  
#define INTERNAL_OPAMP_CONFIG     // Use internal op-amps
//#undef INTERNAL_OPAMP_CONFIG   // Use external op-amps

// Fault protection
#define ENABLE_PWM_FAULT          // Enable hardware fault protection
//#undef ENABLE_PWM_FAULT        // Disable fault protection
```

### Board Compatibility Matrix

**MCHV-2/MCHV-3 (High-Voltage Boards):**
- Voltage range: 24V-300V DC bus
- Current sensing: External op-amps with isolated supplies
- Gate drivers: Bootstrap capacitors with charge pump backup
- Protection: Hardware overcurrent and overvoltage

**MCLV-2 (Low-Voltage Board):**
- Voltage range: 8V-24V DC bus  
- Current sensing: Internal op-amps sufficient
- Gate drivers: Simple bootstrap capacitors
- Protection: Software-based current limiting

**Custom Board Adaptation:**
```c
// Pin mapping changes in port_config.c
// PWM assignments can be remapped to different generators
// ADC channel assignments configurable via ADTRIG registers
// Op-amp routing configurable via INTERNAL_OPAMP_CONFIG
```

## Summary

This HAL design provides **robust, high-performance hardware abstraction** with the following key strengths:

### **Timing Precision**
- Bootstrap charging with 25-cycle precision control
- ADC sampling synchronized to PWM valleys for minimum noise
- Single shunt dual duty cycle compensation with 1-cycle accuracy

### **Safety and Reliability**  
- Multi-layered fault protection (hardware + software)
- Gradual startup sequences prevent component damage
- Comprehensive parameter validation and limiting

### **Performance Optimization**
- Multi-core ADC architecture for parallel sampling
- Assembly-optimized math routines for real-time execution  
- Q15 fixed-point arithmetic for numerical stability

### **Flexibility and Portability**
- Compile-time configuration for different board types
- Scalable from low-voltage hobby applications to industrial drives
- Comprehensive diagnostic integration for development and maintenance

The hardware abstraction successfully bridges the gap between complex motor control algorithms and the sophisticated dsPIC33CK peripheral set, enabling industrial-grade motor control performance while maintaining code clarity and maintainability.