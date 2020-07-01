// Included Files ---
#include "F28x_Project.h"

// Defines (PWM)
#define EPWM1_MAX_DB   0x070
#define EPWM2_MAX_DB   0x070
#define EPWM3_MAX_DB   0x070
#define EPWM4_MAX_DB   0x070
#define EPWM1_MIN_DB   0x070
#define EPWM2_MIN_DB   0x070
#define EPWM3_MIN_DB   0x070
#define EPWM4_MIN_DB   0x070
#define DB_UP          1
#define DB_DOWN        0
#define C_TBPRD        1176     // TBPRD = [1/(fpwm*Tclk)]-1  ----------  fpwm = 1/Tpwm  -->  Tpwm = (TBPRD+1)*Tclk  where tclk = 1/100Mhz
#define C_CMPA         588      // set to 50% of C_TBPRD for 50% duty cycle
#define C_TBPHS1       0
#define C_TBPHS2       588
#define C_TBPHS3       0
#define C_TBPHS4       588

#define Freq_Max       120000
#define Freq_Min       70000
#define ADC_Resolution 4096
#define System_Voltage 3.3
#define HYSTERESIS     0.1

//pid
#define Voltage_Setpoint     1.8      // from 0V to 3.3V
#define Kp  1
#define Ki  1
#define Kd  1
#define ErrorIntegralMax    50
#define ErrorIntegralMin    0

// Globals (PWM)
Uint32 EPwm1TimerIntCount;
Uint32 EPwm2TimerIntCount;
Uint32 EPwm3TimerIntCount;
Uint32 EPwm4TimerIntCount;
Uint16 EPwm1_DB_Direction;
Uint16 EPwm2_DB_Direction;
Uint16 EPwm3_DB_Direction;
Uint16 EPwm4_DB_Direction;
Uint16 A;

// Globals (ADC)
Uint16 ADC_DSP_VS_INPUT;
Uint16 ADC_DSP_CS_INVER;
Uint16 ADC_DSP_CS_OUTPUT;
Uint16 ADC_DSP_CS_RECT;
Uint16 ADC_DSP_CS_INPUT;

// Function Prototypes (PWM)
void InitEPwm1(void);
void InitEPwm2(void);
void InitEPwm3(void);
void InitEPwm4(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void epwm4_isr(void);

// Function Prototypes (ADC)
void ConfigureADC(void);
void SetupADCSoftware(void);

// Main
void main(void)
{
     A = 0;
//Initialize System Control:
    InitSysCtrl();

//Initialize GPIO
    InitGpio();

// enable PWM1, PWM2, PWM3 and PWM4
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM3=1;
    CpuSysRegs.PCLKCR2.bit.EPWM4=1;

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the F2837xD_EPwm.c file
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();

// Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
    DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
    InitPieVectTable();

// This block of code sets up the interrupt for the 4 sets of PWM signals
// The functions mapped to interrupt table
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
    PieVectTable.EPWM2_INT = &epwm2_isr;
    PieVectTable.EPWM3_INT = &epwm3_isr;
    PieVectTable.EPWM4_INT = &epwm4_isr;
    EDIS;   // This is needed to disable write to EALLOW protected registers

// Initialize the Device Peripherals:
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
    EDIS;

    InitEPwm1();
    InitEPwm2();
    InitEPwm3();
    InitEPwm4();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
    EDIS;

//setup ADC on Channel 1
    ConfigureADC();
    SetupADCSoftware();

// User specific code, enable interrupts:
// Initialize counters:
    EPwm1TimerIntCount = 0;
    EPwm2TimerIntCount = 0;
    EPwm3TimerIntCount = 0;
    EPwm4TimerIntCount = 0;

// Enable CPU INT3 which is connected to EPWM1-3 INT:
    IER |= M_INT3;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx4 = 1;

// Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    EPwm1Regs.DBFED.bit.DBFED=EPWM1_MAX_DB;
    EPwm1Regs.DBRED.bit.DBRED=EPWM1_MIN_DB;
    EPwm2Regs.DBFED.bit.DBFED=EPWM2_MAX_DB;
    EPwm2Regs.DBRED.bit.DBRED=EPWM2_MIN_DB;
    EPwm3Regs.DBFED.bit.DBFED=EPWM3_MAX_DB;
    EPwm3Regs.DBRED.bit.DBRED=EPWM3_MIN_DB;
    EPwm4Regs.DBFED.bit.DBFED=EPWM4_MAX_DB;
    EPwm4Regs.DBRED.bit.DBRED=EPWM4_MIN_DB;

    Uint16 TBPRD; // TBPRD = [1/(fpwm*Tclk)]-1  ----------  fpwm = 1/Tpwm  -->  Tpwm = (TBPRD+1)*Tclk  where tclk = 1/100Mhz
    float Freq = 120000;
    float Voltage_DSP_VS_INPUT    = AdcaResultRegs.ADCRESULT0;
    float Voltage_DSP_CS_INVER    = AdcbResultRegs.ADCRESULT1;
    float Voltage_DSP_CS_OUTPUT   = AdcbResultRegs.ADCRESULT2;
    float Voltage_DSP_CS_RECT     = AdcbResultRegs.ADCRESULT3;
    float Voltage_DSP_CS_INPUT    = AdccResultRegs.ADCRESULT4;
    float ErrorVoltage            = 0;
    float ErrorPrevious           = 0;
    float ErrorProportional       = 0;
    float ErrorIntegral           = 0;
    float ErrorDerivative         = 0;

// loop forever:
    do
    {
        //convert, wait for completion, and store results
        //start conversion on ADCA
        AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;

        //Start conversion on ADCB
        AdcbRegs.ADCSOCFRC1.bit.SOC1 = 1;
        AdcbRegs.ADCSOCFRC1.bit.SOC2 = 1;
        AdcbRegs.ADCSOCFRC1.bit.SOC3 = 1;

        //start conversion on ADCC
        AdccRegs.ADCSOCFRC1.bit.SOC4 = 1;

        //Waits for conversion to complete, then clears flag, ADCA
        while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0);
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

        //Waits for conversion to complete, then clears flag, ADCB
        while(AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0);
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

        //Waits for conversion to complete, then clears flag, ADCC
        while(AdccRegs.ADCINTFLG.bit.ADCINT1 == 0);
        AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

        //store results

        ADC_DSP_VS_INPUT    = AdcaResultRegs.ADCRESULT0;
        ADC_DSP_CS_INVER    = AdcbResultRegs.ADCRESULT1;
        ADC_DSP_CS_OUTPUT   = AdcbResultRegs.ADCRESULT2;
        ADC_DSP_CS_RECT     = AdcbResultRegs.ADCRESULT3;
        ADC_DSP_CS_INPUT    = AdccResultRegs.ADCRESULT4;

        /*
         *    ADC_Resolution       ADC_Reading
         *  ------------------ = -----------------, System_Voltage = 3v3, ADC_Resolution = 2^12
         *    System_Voltage      Analog_Voltage
         *
         */
        Voltage_DSP_VS_INPUT    = (ADC_DSP_VS_INPUT * System_Voltage)/ADC_Resolution;
        Voltage_DSP_CS_INVER    = (ADC_DSP_CS_INVER * System_Voltage)/ADC_Resolution;
        Voltage_DSP_CS_OUTPUT   = (ADC_DSP_CS_OUTPUT * System_Voltage)/ADC_Resolution;
        Voltage_DSP_CS_RECT     = (ADC_DSP_CS_RECT * System_Voltage)/ADC_Resolution;
        Voltage_DSP_CS_INPUT    = (ADC_DSP_CS_INPUT * System_Voltage)/ADC_Resolution;

        //change pwm duty cycle
        /* simple hysteresis control
        if ((Voltage_DSP_CS_OUTPUT < 1.8+HYSTERESIS) && (Freq < Freq_Max)) {
            Freq = Freq+1000;
        }
        if ((Voltage_DSP_CS_OUTPUT > 1.8-HYSTERESIS) && (Freq > Freq_Min)) {
            Freq = Freq-1000;
        }
        */

        /*----------------- pid control ---------------------------*/
        ErrorVoltage        = Voltage_Setpoint - Voltage_DSP_CS_OUTPUT;

        // Proportional Error
        ErrorProportional   = ErrorVoltage;

        // Integral Error with simple integral windup prevention
        ErrorIntegral = ErrorIntegral + ErrorVoltage;
        if (ErrorIntegral > ErrorIntegralMax){
            ErrorIntegral = ErrorIntegralMax;
        }
        if (ErrorIntegral < ErrorIntegralMin){
            ErrorIntegral = ErrorIntegralMin;
        }

        // Derivative Error
        ErrorDerivative = ErrorVoltage - ErrorPrevious;

        // PID_OUT
        /*-----------------------------------------------------------*/

        TBPRD = 1/(Freq*(1.0/100000000))-1; // TBPRD = [1/(fpwm*Tclk)]-1  ----------  fpwm = 1/Tpwm  -->  Tpwm = (TBPRD+1)*Tclk  where tclk = 1/100Mhz
        EPwm1Regs.TBPRD         = TBPRD;
        EPwm1Regs.CMPA.bit.CMPA = TBPRD/2;
        EPwm2Regs.TBPRD         = TBPRD;
        EPwm2Regs.CMPA.bit.CMPA = TBPRD/2;
        EPwm3Regs.TBPRD         = TBPRD;
        EPwm3Regs.CMPA.bit.CMPA = TBPRD/2;
        EPwm4Regs.TBPRD         = TBPRD;
        EPwm4Regs.CMPA.bit.CMPA = TBPRD/2;

        //software breakpoint, hit run again to get updated conversions
        A = A+1;
       // DELAY_US(10000);
       // asm("   ESTOP0");

    }while(1);
}

// epwm1 interrupt function
__interrupt void epwm1_isr(void)
{
    EPwm1TimerIntCount++;
    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// epwm2 interrupt function
__interrupt void epwm2_isr(void)
{
    EPwm2TimerIntCount++;
    // Clear INT flag for this timer
    EPwm2Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// epwm3 interrupt function
__interrupt void epwm3_isr(void)
{
    EPwm3TimerIntCount++;
    EPwm3Regs.ETCLR.bit.INT = 1;    // Clear INT flag for this timer
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// epwm4 interrupt function
__interrupt void epwm4_isr(void)
{
    EPwm4TimerIntCount++;
    EPwm4Regs.ETCLR.bit.INT = 1;// Clear INT flag for this timer
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// InitEPwm1 - Initialize EPWM1 configuration
void InitEPwm1()
{
    EPwm1Regs.TBPRD = C_TBPRD;                      // Set timer period
    EPwm1Regs.CMPA.bit.CMPA = C_CMPA;               // Set compare
    EPwm1Regs.TBPHS.bit.TBPHS = C_TBPHS1;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter

    // Setup TBCLK
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Count up
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
    EPwm1Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;              // Clock ratio to SYSCLKOUT. Set to none
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;                 // No clk divider
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    // Setup CMPCTL
    // Shadow register used to keep counter registers synchronized with hardware
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // enable shadow register for CH A
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;     // enables shadow register for CH B
    // Load Mode register to enable immediate write from shadow reg to active reg
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;   //immediate write enabled
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   //immediate write enabled

    // Set actions
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;              // 0x2, set PWM HIGH when cntr =0
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // 0x1 set PWM LO when cntr = cmp
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;              // 0x2, set PWM HIGH when cntr =0
    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // 0x1 set PWM LO when cntr = cmp

    // Setup Deadband
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // Enabled falling+rising edge
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       //Active HI complimentary for epwmB
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;          //Set B as src of rising+falling delay
    EPwm1Regs.DBRED.bit.DBRED = EPWM1_MIN_DB;       //Set rising edge deadtime
    EPwm1Regs.DBFED.bit.DBFED = EPWM1_MIN_DB;       //Set falling edge deadtime
    EPwm1_DB_Direction = DB_UP;

    // Interrupt where we will change the Deadband
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // 0x1, interrupt when ctr = 0
    EPwm1Regs.ETSEL.bit.INTEN = 1;                  // Enable interrupt
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;             // 0x3, interrupt on third event
}

// InitEPwm2 - Initialize EPWM2 configuration
void InitEPwm2()
{
    EPwm2Regs.TBPRD = C_TBPRD;                      // Set timer period
    EPwm2Regs.CMPA.bit.CMPA = C_CMPA;               // Set compare
    EPwm2Regs.TBPHS.bit.TBPHS = C_TBPHS2;           // Phase is 180
    EPwm2Regs.TBCTR = 0x0000;                       // Clear counter

    // Setup TBCLK
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;          //Enable phase loading
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;              // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;              // set actions for EPWM1A
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;              // set actions for EPWM1B
    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;

    // Setup the deadband
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED.bit.DBRED = EPWM2_MIN_DB;
    EPwm2Regs.DBFED.bit.DBFED = EPWM2_MIN_DB;
    EPwm2_DB_Direction = DB_UP;

    // Interrupt where we will modify the deadband
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;       // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;                  // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;             // Generate INT on 3rd event
}

// InitEPwm3 - Initialize EPWM3 configuration
void InitEPwm3()
{
    EPwm3Regs.TBPRD = C_TBPRD;                      // Set timer period
    EPwm3Regs.CMPA.bit.CMPA = C_CMPA;               // Set compare
    EPwm3Regs.TBPHS.bit.TBPHS = C_TBPHS3;           // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                       // Clear counter

    // Setup TBCLK
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Count up
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // Disable phase loading
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;              // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = 0;                 // Slow so we can observe on
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;              // set actions for EPWM1A
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;              // set actions for EPWM1B
    EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;

    // Active high complementary PWMs - Setup the deadband
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED.bit.DBRED = EPWM3_MIN_DB;
    EPwm3Regs.DBFED.bit.DBFED = EPWM3_MIN_DB;
    EPwm3_DB_Direction = DB_UP;

    // Interrupt where we will change the deadband
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;            // Generate INT on 3rd event
}

// InitEPwm4 - Initialize EPWM4 configuration
void InitEPwm4()
{
    EPwm4Regs.TBPRD = C_TBPRD;                      // Set timer period
    EPwm4Regs.CMPA.bit.CMPA = C_CMPA;               // Set compare
    EPwm4Regs.TBPHS.bit.TBPHS = C_TBPHS4;           // Phase is 180
    EPwm4Regs.TBCTR = 0x0000;                       // Clear counter

    // Setup TBCLK
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      // Count up
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // Disable phase loading
    EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;              // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = 0;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;              // set actions for EPWM1A
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLB.bit.ZRO = AQ_SET;              // set actions for EPWM1B
    EPwm4Regs.AQCTLB.bit.CAU = AQ_CLEAR;

    // Setup Deadband
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm4Regs.DBRED.bit.DBRED = EPWM1_MIN_DB;
    EPwm4Regs.DBFED.bit.DBFED = EPWM1_MIN_DB;
    EPwm4_DB_Direction = DB_UP;

    // Interrupt where we will change the Deadband
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm4Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm4Regs.ETPS.bit.INTPRD = ET_3RD;          // Generate INT on 3rd event
}


// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;          //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;          //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADCs
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

// SetupADCSoftware - Setup ADC channels and acquisition window
void SetupADCSoftware(void)
{
    Uint16 acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    //
    //Select the channels to convert and end of conversion flag
    //ADCA
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;      //SOC0 will convert pin A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;  //sample window is acqps + 1 SYSCLK cycles

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared

    //ADCB
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 2;      //SOC0 will convert pin B2
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;  //sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 3;      //SOC1 will convert pin B3
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps;  //sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 5;      //SOC2 will convert pin B5
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps;  //sample window is acqps + 1 SYSCLK cycles

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 3;  //end of SOC3 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared

    //ADCC
    AdccRegs.ADCSOC4CTL.bit.CHSEL = 2;      //SOC0 will convert pin C2
    AdccRegs.ADCSOC4CTL.bit.ACQPS = acqps;  //sample window is acqps + 1 SYSCLK cycles

    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 4;  //end of SOC4 will set INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared

    EDIS;
}
// End of file
