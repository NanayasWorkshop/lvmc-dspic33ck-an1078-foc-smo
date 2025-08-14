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

### PWM Modes
**Single Shunt**: Dual-edge center-aligned mode (MODSEL = 6)
**Dual Shunt**: Standard center-aligned mode (MODSEL = 4)

### Key PWM Features
- **Dead time**: 2μs on all phases prevents shoot-through
- **Center-aligned**: Minimizes current ripple and EMI
- **Master-slave sync**: Ensures perfect phase alignment
- **Fault protection**: Hardware overcurrent shutdown

## Bootstrap Capacitor Charging

### Sophisticated Startup Sequence

**Problem**: Gate driver bootstrap capacitors need controlled charging to prevent damage.

**Solution**: Multi-phase charging with gradual duty cycle progression.

### Charging Algorithm
```c
#define BOOTSTRAP_CHARGING_COUNTS 400  // ~20ms charging time
```

**Phase 1** (400 → 350 counts): All high-side OFF, Phase 1 low-side PWM
**Phase 2** (350 → 250 counts): Enable Phase 2 low-side PWM  
**Phase 3** (250 → 150 counts): Enable Phase 4 low-side PWM
**Phase 4** (150 → 0 counts): Gradually reduce all duty cycles to zero

### Timing Control
```c
// Initial high duty cycle for rapid charging
PWM_PDC = LOOPTIME_TCY - (DDEADTIME/2 + 5)

// Monitor PWM counter state
currStatusCAHALF = PG1STATbits.CAHALF  // Counter direction indicator

// Gradual duty reduction every 25 cycles
if (k > 25) {
    PWM_PDC -= 2;  // Slow ramp down
    k = 0;
}
```

### Hardware Protection
- **Override control**: High-side switches forced OFF via `OVRENH = 1`
- **Sequential enable**: Low-side switches enabled one by one
- **Duty limiting**: Prevents excessive inrush current

## ADC Configuration & Timing

### Multi-Core ADC Architecture
**Three ADC cores** for simultaneous sampling:
- **Dedicated Core 0**: Motor phase currents (AN0, AN1)
- **Dedicated Core 1**: Reserved for expansion
- **Shared Core**: Auxiliary signals (pot, voltage, temperature)

### ADC Specifications
```c
Resolution: 12-bit signed/unsigned
Sample Time: 15 TADCORE cycles
Clock: 100MHz peripheral clock
Conversion Time: ~1μs per channel
```

### Trigger Strategy

**Single Shunt Mode:**
- **PWM1 Trigger 1** (TRIGA): Phase current sampling
- **PWM1 Trigger 2** (TRIGB): First bus current sample  
- **PWM1 Trigger 3** (TRIGC): Second bus current sample
- **Software Trigger**: Auxiliary measurements

**Dual Shunt Mode:**
- **PWM1 Trigger 1**: Simultaneous Ia, Ib sampling at PWM center
- **Software Trigger**: Auxiliary measurements

### Noise Mitigation
- **Center sampling**: ADC triggers at PWM valley (minimum switching noise)
- **Dedicated cores**: Isolated analog supplies
- **Sample delay**: Accounts for switching transients
- **Signed conversion**: Handles bipolar current signals

## Current Sensing Hardware

### Single Shunt Configuration
**Bus current sensor** on DC link with precision timing:
```c
ADCBUF_IBUS = ADCBUF4      // DC bus current
ADCBUF_IPHASE1 = -ADCBUF0  // Phase A (for offset calibration)
ADCBUF_IPHASE2 = -ADCBUF1  // Phase B (for offset calibration)
```

### Internal Op-Amp Configuration
**Three op-amps** for current amplification (when `INTERNAL_OPAMP_CONFIG` defined):

**Op-Amp 1 (Phase A)**:
- Input+: AN9 (RA2)
- Input-: ANA1 (RA1)  
- Output: AN0 (RA0)

**Op-Amp 2 (Phase B)**:
- Input+: RB4
- Input-: RB3
- Output: AN1 (RB2)

**Op-Amp 3 (Bus Current)**:
- Input+: AN14 (RC2)
- Input-: AN13 (RC1)
- Output: AN4 (RA4)

### Op-Amp Settings
```c
NCHDIS = 0    // Wide input range enabled
AMPEN = 1     // Op-amp enabled
AMPON = 1     // Global op-amp enable
```

## GPIO Hardware Mapping

### Motor Control Outputs
```c
PWM1H: RB14 (Pin 1)   // Phase A High
PWM1L: RB15 (Pin 3)   // Phase A Low  
PWM2H: RB12 (Pin 78)  // Phase B High
PWM2L: RB13 (Pin 80)  // Phase B Low
PWM4H: RD1  (Pin 73)  // Phase C High
PWM4L: RD0  (Pin 74)  // Phase C Low
```

### User Interface
```c
// Status LEDs
LED1: RE6 (Pin 37)    // System status
LED2: RE7 (Pin 39)    // Motor running

// Control Buttons  
SW1: RE11 (Pin 59)    // Start/Stop
SW2: RE12 (Pin 62)    // Speed mode toggle
```

### Analog Inputs
```c
// Motor Measurements
AN0:  RA0  // Phase A current (or op-amp output)
AN1:  RB2  // Phase B current (or op-amp output)  
AN4:  RA4  // Bus current (or op-amp output)
AN11: RB9  // Speed reference potentiometer
AN12: RC0  // Temperature sensor
AN15: RC3  // DC bus voltage
```

### Communication
```c
// UART for diagnostics/tuning
UART_RX: RD14 (Pin 13, RP78)
UART_TX: RD13 (Pin 14, RP77)
```

## Interrupt Architecture

### Priority Hierarchy
```c
Priority 7: ADC Current Sampling     // Highest - real-time critical
Priority 7: PWM Fault Detection      // Highest - safety critical  
Priority 1: UART Communication       // Lowest - non-real-time
```

### ADC Interrupt Strategy

**Single Shunt**: Dual interrupt system
```c
_ADCAN4Interrupt:  // Bus current sampling (2x per PWM cycle)
    - Sample 1: Store Ibus1, set next trigger
    - Sample 2: Store Ibus2, reconstruct phases
    - Execute full control loop on Sample 2
```

**Dual Shunt**: Single interrupt system  
```c
_ADCAN11Interrupt: // Phase current sampling (1x per PWM cycle)
    - Read Ia, Ib directly
    - Execute full control loop
```

## Fault Protection System

### Hardware Overcurrent Protection
**Analog comparator** with DAC reference:
```c
Comparator 1: Monitors bus current
Reference: Programmable via DAC (3A threshold)
Response: Hardware PWM shutdown via PCI
```

### PWM Fault Configuration
```c
PCI Source: Comparator 1 output (PSS = 0b11011)
Fault Action: Immediate PWM disable
Termination: Software controlled restart
```

### Software Fault Handling
```c
_PWMInterrupt():  // Fault recovery ISR
    - Reset all parameters
    - Clear fault flags  
    - Prepare for restart
```

## Power Management

### ADC Core Power Sequencing
```c
// Enable and wait for each core
ADCON5Lbits.C0PWR = 1;
while(ADCON5Lbits.C0RDY == 0);  // Wait for ready
ADCON3Hbits.C0EN = 1;           // Enable core
```

### Op-Amp Power Control
```c
AMPCON1Lbits.AMPON = 1;     // Master enable
AMPCON1Lbits.AMPEN1 = 1;    // Individual amp enables
AMPCON1Lbits.AMPEN2 = 1;
AMPCON1Lbits.AMPEN3 = 1;
```

## Diagnostic Integration

### X2CScope Real-Time Debugging
**UART-based real-time monitoring** for motor parameters:
```c
Baud Rate: Variable (54x divider default)
Protocol: X2CScope proprietary
Features: Real-time variable watch, parameter tuning
```

### Board Service Functions
**Low-frequency maintenance tasks** (1kHz):
- Button debouncing and scanning
- LED status updates  
- System health monitoring
- Non-critical parameter updates

## Hardware Timing Critical Paths

### Real-Time Constraints
```c
ADC Interrupt: Must complete in <25μs (50% of PWM period)
PWM Update: Must occur before next PWM period
Bootstrap Charging: 20ms total, non-interruptible
Current Offset: 1024 samples for accurate calibration
```

### Performance Optimization
- **Assembly math routines**: Park/Clarke transforms, PI controllers
- **Q15 fixed-point**: Eliminates floating-point overhead
- **Dual duty cycles**: Optimizes single shunt sampling
- **Multi-core ADC**: Parallel sampling reduces conversion time

## Configuration Flexibility

### Compile-Time Hardware Selection
```c
#define SINGLE_SHUNT              // vs dual shunt sensing
#define INTERNAL_OPAMP_CONFIG     // vs external op-amps  
#define ENABLE_PWM_FAULT          // Hardware fault protection
```

### Board Compatibility
- **MCHV-2/MCHV-3**: High-voltage development boards
- **MCLV-2**: Low-voltage development board  
- **Custom boards**: Configurable via pin mapping changes

This HAL design provides **robust, high-performance hardware abstraction** suitable for industrial motor control applications while maintaining flexibility for different board configurations and motor types.