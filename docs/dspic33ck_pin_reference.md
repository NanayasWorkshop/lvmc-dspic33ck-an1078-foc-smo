# dsPIC33CK256MP508 Pin Reference

## Power & Config
- **VDD**: 12,31,51,71 - +3.3V digital supply
- **VSS**: 11,32,50,70 - Digital ground  
- **AVDD**: 25 - +3.3V analog supply
- **AVSS**: 26 - Analog ground
- **OSCI**: 34 - Oscillator input (crystal X2)
- **OSCO**: 35 - Oscillator output
- **MCLR**: 9 - Reset (SW4, ICSP, PKOB)

## Programming
- **PGD3**: 55 - Program data
- **PGC3**: 56 - Program clock

## Internal Op Amps (Current Sensing)
### Op Amp 1 (Phase A)
- **OA1IN+**: 20 - Shunt positive
- **OA1IN-**: 18 - Shunt negative  
- **OA1OUT**: 16 - Amplified output

### Op Amp 2 (Phase B)
- **OA2IN+**: 45 - Shunt positive
- **OA2IN-**: 43 - Shunt negative
- **OA2OUT**: 41 - Amplified output

### Op Amp 3 (Bus Current)
- **OA3IN+**: 29 - Shunt positive
- **OA3IN-**: 28 - Shunt negative
- **OA3OUT**: 23 - Amplified output

## PWM Motor Control
- **PWM1H**: 1 - Phase A top MOSFET (Q1)
- **PWM1L**: 3 - Phase A bottom MOSFET (Q2)
- **PWM2H**: 78 - Phase B top MOSFET (Q3)
- **PWM2L**: 80 - Phase B bottom MOSFET (Q4)
- **PWM4H**: 73 - Phase C top MOSFET (Q5)
- **PWM4L**: 74 - Phase C bottom MOSFET (Q6)

## Analog Inputs
### External Current Amplifiers
- **IA_EXT**: 16 - Phase A current (U5-A)
- **IB_EXT**: 41 - Phase B current (U5-B)
- **IBUS_EXT**: 23 - Bus current (U15)

### Voltage Feedback
- **V_BUS**: 33 - DC bus voltage
- **V_A**: 30 - Phase A voltage
- **V_B**: 19 - Phase B voltage
- **V_C**: 17 - Phase C voltage

### Temperature & Reference
- **TEMP_LOCAL**: 15 - MOSFET temp (MCP9700)
- **TEMP_EXT**: 58 - External temp sensor
- **SPEED_REFERENCE**: 61 - Potentiometer (POT1)

### Overcurrent Detection
- **IBUS_FILT_EXT**: 21 - Filtered bus current for comparator

## Hall Sensors
- **HALL_A**: 42 - Hall sensor A
- **HALL_B**: 44 - Hall sensor B  
- **HALL_C**: 57 - Hall sensor C

## Quadrature Encoder
- **QEI_A**: 5 - Encoder phase A (RP60)
- **QEI_B**: 6 - Encoder phase B (RP61)
- **QEI_INDEX**: 7 - Encoder index (RP62)
- **QEI_HOME**: 8 - Encoder home (RP63)

## Communication
- **DEBUG_RX**: 13 - UART RX (RP78)
- **DEBUG_TX**: 14 - UART TX (RP77)

## User Interface
- **LED1**: 37 - Debug LED (LD10)
- **LED2**: 39 - Debug LED (LD11)
- **BUTTON1**: 59 - Push button SW1
- **BUTTON2**: 62 - Push button SW2
- **BUTTON3**: 64 - Push button SW3
- **TP11**: 22 - Test point
- **TP12**: 24 - Test point
- **TP13**: 79 - Test point

## mikroBUS Socket A (J11)
- **CLICK_AN_A**: 4 - Analog (AN21)
- **CLICK_RST_A**: 77 - Reset
- **CLICK_CS_A**: 75 - SPI CS (RP42)
- **CLICK_SCK_A**: 27 - SPI CLK (RP76)
- **CLICK_MISO_A**: 38 - SPI MISO (RP74)
- **CLICK_MOSI_A**: 36 - SPI MOSI (RP75)
- **CLICK_SDA_A**: 68 - I2C Data (RP68)
- **CLICK_SCL_A**: 69 - I2C Clock (RP67)
- **CLICK_TX_A**: 40 - UART TX (RP55)
- **CLICK_RX_A**: 52 - UART RX (RP71)
- **CLICK_INT_A**: 10 - Interrupt (RP79)
- **CLICK_PWM_A**: 76 - PWM (RP43)

## mikroBUS Socket B (J12)
- **CLICK_AN_B**: 2 - Analog (AN20)
- **CLICK_RST_B**: 72 - Reset (RP66)
- **CLICK_CS_B**: 48 - SPI CS (RP73)
- **CLICK_SCK_B**: 46 - SPI CLK (RP56/SCK2)
- **CLICK_MISO_B**: 49 - SPI MISO (RP72/SDO2)
- **CLICK_MOSI_B**: 47 - SPI MOSI (RP57/SDI2)
- **CLICK_SDA_B**: 63 - I2C Data (RP52)
- **CLICK_SCL_B**: 65 - I2C Clock (RP53)
- **CLICK_TX_B**: 54 - UART TX (RP69)
- **CLICK_RX_B**: 53 - UART RX (RP70)
- **CLICK_INT_B**: 67 - Interrupt (RP59)
- **CLICK_PWM_B**: 66 - PWM (RP58)

## Notes
- Jumper resistors R121/R125, R129/R133, R137/R141 select internal vs external amplifiers
- Many pins are remappable via PPS (Peripheral Pin Select)
- Hall sensors support change notification interrupts
- PWM outputs have automatic fault protection via internal comparators