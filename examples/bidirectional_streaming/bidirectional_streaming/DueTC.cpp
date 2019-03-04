/*
  DueTC.cpp - Local code of Arduino DUE Timer Control defined on DueTC.h

Created by Olavi Kamppari, October 2013.
Reference to Atmel documentation 11057B-ATARM-28-May-12
for SAM3X and SAM3A Series

Programming principles:
- avoid unecessary S/W layers that are just complicating things
- use only one bit manipulation routine (encodeTC_WaveformMode) to calculate a value for Channel Mode Register (TC_CMR)
- use naming conventions specified by Atmel, such as
       TCx,  where x=indivudal timer number (0-8)
       REG_TCi_<reg>j, where
          i     = timer block number (0-2),
          j     = timer channel number in the block (0-2),
          <reg> = register name (CCR, CMR, SMMR, CV, RA, RB, RC, SR, IER, IDR, IMR, BCR, BMR,
                  QIER, QIDR, QIMR. QISR, FMR, WPMR)
- define pin connected timers and interrupts that are not conflicting with system timers and eachothers
- define all available connected timers and interrupts
- verify the results with an oscilloscope
       > observe the common false assumption that TIMER_CLOCK5 divider is 1024, when it actually is 3072

The main information that is not evident is in routine

       pmc_enable_periph_clk(TCx_IRQn);

It connects the interrupt ID number (TCx_IRQn) to a predefined interrupt handler (TCx_Handler).  The handler code is in this
module and it calls a user defined interrupt handler that is defined in the setup call (setupTC2_Interrupt). 

Released into the public domain.
*/

#include "DueTC.h"

unsigned long encodeTC_WaveformMode(
    byte BSWTRG, byte BEEVT, byte BCPC, byte BCPB,
    byte ASWTRG, byte AEEVT, byte ACPC, byte ACPA,
    byte WAVSEL, byte ENETRG, byte EEVT, byte EEVTEDG,
    byte CPCDIS, byte CPCSTOP, byte BURST, byte CLKI, byte TCCLKS) {
  // See Atmel doc 11057, section 37.7.11 on page 906
  /*
BSWTRG(30-31), Software Trigger Effect on TIOB
       0 NONE      None
       1 SET       Set
       2 CLEAR     Clear
       3 TOGGLE    Toggle
   
   BEEVT(28-29), External Event Effect on TIOB
       0 NONE      None
       1 SET       Set
       2 CLEAR     Clear
       3 TOGGLE    Toggle
   
   BCPC(26-27), RC Compare Effect on TIOB
       0 NONE      None
       1 SET       Set
       2 CLEAR     Clear
       3 TOGGLE    Toggle
   
   BCPB(24-25), RB Compare Effect on TIOB
       0 NONE      None
       1 SET       Set
       2 CLEAR     Clear
       3 TOGGLE    Toggle
   
   ASWTRG(22-23), Software Trigger Effect on TIOA
       0 NONE      None
       1 SET       Set
       2 CLEAR     Clear
       3 TOGGLE    Toggle
   
   AEEVT(20-21), External Event Effect on TIOA
       0 NONE      None
       1 SET       Set
       2 CLEAR     Clear
       3 TOGGLE    Toggle
   
   ACPC(18-19), RC Compare Effect on TIOA
       0 NONE      None
       1 SET       Set
       2 CLEAR     Clear
       3 TOGGLE    Toggle
   
   ACPA(16-17), RA Compare Effect on TIOA
       0 NONE      None
       1 SET       Set
       2 CLEAR     Clear
       3 TOGGLE    Toggle
   
   WAVE(15), This is always 1 in waform generation mode
       0: Capture Mode is enabled (Measure signals).
       1: Capture Mode is disabled (Generate wave).
   
   WAVSEL(13-14)
       0 UP        UP mode without automatic trigger on RC Compare
       1 UPDOWN    UPDOWN mode without automatic trigger on RC Compare
       2 UP_RC     UP mode with automatic trigger on RC Compare
       3 UPDOWN_RC UPDOWN mode with automatic trigger on RC Compare
   
   ENETRG(12), External Event Trigger Enable
       0: the external event has no effect on the counter and its clock.
           In this case, the selected external event only controls the TIOA output.
       1: the external event resets the counter and starts the counter clock
   
   EEVT(10-11), External Event Selection
       0 TIOB       TIOB   TIOB is input
       1 XC0        XC0    TIOB is output
       2 XC1        XC1    TIOB is output
       3 XC2        XC2    TIOB is output
   
   EEVTEDG(8-9),  External Event Edge Selection
       0 NONE      None
       1 RISING    Rising edge
       2 FALLING   Falling edge
       3 EDGE      Each edge
   
   CPCDIS(7)
       0: counter clock is not disabled when RB loading occurs.
       1: counter clock is disabled when RB loading occurs
   
   CPCSTOP(6)
       0: counter clock is not stopped when RB loading occurs.
       1: counter clock is stopped when RB loading occurs
   
   BURST(4-5)
       0 NONE       The clock is not gated by an external signal.
       1 XC0        XC0 is ANDed with the selected clock.
       2 XC1        XC1 is ANDed with the selected clock.
       3 XC2        XC2 is ANDed with the selected clock
   
   CLKI(3)
       0: counter is incremented on rising edge of the clock.
       1: counter is incremented on falling edge of the clock.   
   
   TCCLKS(0-2)
       0 TIMER_CLOCK1    Clock selected: TCLK1    divider = 2 (1<<1)    42,000 kHz
       1 TIMER_CLOCK2    Clock selected: TCLK2    divider = 8 (1<<3)    10,500 kHz
       2 TIMER_CLOCK3    Clock selected: TCLK3    divider = 32 (1<<5)    2,625 kHz
       3 TIMER_CLOCK4    Clock selected: TCLK4    divider = 128 (1<<7)    656.25 Hz
       4 TIMER_CLOCK5    Clock selected: TCLK5    divider = 3*1024     0.027 Hz
       5 XC0        Clock selected: XC0
       6 XC1        Clock selected: XC1
       7 XC2        Clock selected: XC2
       
   */
  unsigned long encodedValue;
  byte WAVE=1;                        // Force wave generation
  encodedValue=(TCCLKS & 7) +
    ((CLKI & 1) << 3) +
    ((BURST & 3) << 4) +
    ((CPCSTOP & 1) << 6) +
    ((CPCDIS & 1) << 7) +
    ((EEVTEDG & 3) << 8) +
    ((EEVT & 3) << 10) +
    ((ENETRG & 1) << 12) +
    ((WAVSEL & 3) << 13) +
    ((WAVE & 1) << 15) +
    ((ACPA &3) << 16) +
    ((ACPC &3) << 18) +
    ((AEEVT &3) << 20) +
    ((ASWTRG &3) << 22) +
    ((BCPB &3) << 24) +
    ((BCPC &3) << 26) +
    ((BEEVT &3) << 28) +
    ((BSWTRG &3) << 30);
  return (encodedValue);
}

unsigned long getTC_Waveform(byte tcClock){
  return (encodeTC_WaveformMode(
  0,0,0,0,
  0,0,0,0,
  2,0,0,0,
  0,0,0,0,tcClock));
}
unsigned long getTC_Waveform_A(byte tcClock){
  return (encodeTC_WaveformMode(
  0,0,0,0,
  0,0,3,0,
  2,0,0,0,
  0,0,0,0,tcClock));
}

unsigned long getTC_Waveform_B(byte tcClock){
  return (encodeTC_WaveformMode(
  0,0,3,0,
  0,0,0,0,
  2,0,0,0,
  0,0,0,0,tcClock));
}

void start_TC0()  {
  REG_TC0_CCR0=5;
}
void start_TC1()  {
  REG_TC0_CCR1=5;
}
void start_TC2()  {
  REG_TC0_CCR2=5;
}
void start_TC3()  {
  REG_TC1_CCR0=5;
}
void start_TC4()  {
  REG_TC1_CCR1=5;
}
void start_TC5()  {
  REG_TC1_CCR2=5;
}
void start_TC6()  {
  REG_TC2_CCR0=5;
}
void start_TC7()  {
  REG_TC2_CCR1=5;
}
void start_TC8()  {
  REG_TC2_CCR2=5;
}

void stop_TC0()  {
  REG_TC0_CCR0=2;
}
void stop_TC1()  {
  REG_TC0_CCR1=2;
}
void stop_TC2()  {
  REG_TC0_CCR2=2;
}
void stop_TC3()  {
  REG_TC1_CCR0=2;
}
void stop_TC4()  {
  REG_TC1_CCR1=2;
}
void stop_TC5()  {
  REG_TC1_CCR2=2;
}
void stop_TC6()  {
  REG_TC2_CCR0=2;
}
void stop_TC7()  {
  REG_TC2_CCR1=2;
}
void stop_TC8()  {
  REG_TC2_CCR2=2;
}


void setupTC_Pin2_Timing(unsigned int period, byte tcClock){
  analogWrite(2,128);                             // Start with PWM setting
  REG_TC0_CMR0=getTC_Waveform_A(tcClock);         // Update Command Register
  REG_TC0_RC0=period;                             // Set period (1 - 0xFFFF)
  start_TC0();                                    // Start timer and generate square wave
}

void setupTC_Pin5_Timing(unsigned int period, byte tcClock){
  analogWrite(5,128);                             // Start with PWM setting
  REG_TC2_CMR0=getTC_Waveform_A(tcClock);         // Update Command Register
  REG_TC2_RC0=period;                             // Set period (1 - 0xFFFF)
  start_TC6();                                    // Start timer and generate square wave
}

void setupTC_Pin3_Timing(unsigned int period, byte tcClock){
  analogWrite(3,128);                             // Start with PWM setting
  REG_TC2_CMR1=getTC_Waveform_A(tcClock);         // Update Command Register
  REG_TC2_RC1=period;                             // Set period (1 - 0xFFFF)
  start_TC7();                                    // Start timer and generate square wave
}

void setupTC_Pin11_Timing(unsigned int period, byte tcClock){
  analogWrite(11,128);                            // Start with PWM setting
  REG_TC2_CMR2=getTC_Waveform_A(tcClock);         // Update Command Register
  REG_TC2_RC2=period;                             // Set period (1 - 0xFFFF)
  start_TC8();                                    // Start timer and generate square wave
}

void changeTC_Pin2_Period(unsigned int period){
  stop_TC0();
  REG_TC0_RC0=period;
  start_TC0();
}

void changeTC_Pin5_Period(unsigned int period){
  stop_TC6();
  REG_TC2_RC0=period;
  start_TC6();
}

void changeTC_Pin3_Period(unsigned int period){
  stop_TC7();
  REG_TC2_RC1=period;
  start_TC7();
}

void changeTC_Pin11_Period(unsigned int period){
  stop_TC8();
  REG_TC2_RC2=period;
  start_TC8();
}

void (*TC2_CallBack)();
void (*TC3_CallBack)();
void (*TC4_CallBack)();
void (*TC5_CallBack)();

void TC2_Handler() {
  TC_GetStatus(TC0, 2);
  TC2_CallBack();
}
void TC3_Handler() {
  TC_GetStatus(TC1, 0);
  TC3_CallBack();
}
void TC4_Handler() {
  TC_GetStatus(TC1, 1);
  TC4_CallBack();
}
void TC5_Handler() {
  TC_GetStatus(TC1, 2);
  TC5_CallBack();
}

void setupTC2_Interrupt(unsigned int period, byte tcClock, void (*isr)()){
  pmc_set_writeprotect(false);                    // Enavle direct TC register manipulation
  pmc_enable_periph_clk(TC2_IRQn);                // Enable predefined interrupt handler
  TC2_CallBack=isr;                               // Route the call to user interrupt handler
  REG_TC0_CMR2=getTC_Waveform(tcClock);           // Update Command Register
  REG_TC0_RC2=period;                             // Set period (1 - 0xFFFF)
  start_TC2();                                    // Start timer and generate square wave
  REG_TC0_IER2= TC_IER_CPCS;                      // Enable RC compare interrupt
  REG_TC0_IDR2=~TC_IER_CPCS;                      // Disable all other timer interrpts 
  NVIC_EnableIRQ(TC2_IRQn);                       // Enable interrupt vector
}

void setupTC3_Interrupt(unsigned int period, byte tcClock, void (*isr)()){
  pmc_set_writeprotect(false);                    // Enavle direct TC register manipulation
  pmc_enable_periph_clk(TC3_IRQn);                // Enable predefined interrupt handler
  TC3_CallBack=isr;                               // Route the call to user interrupt handler
  REG_TC1_CMR0=getTC_Waveform(tcClock);           // Update Command Register
  REG_TC1_RC0=period;                             // Set period (1 - 0xFFFF)
  start_TC3();                                    // Start timer and generate square wave
  REG_TC1_IER0= TC_IER_CPCS;                      // Enable RC compare interrupt
  REG_TC1_IDR0=~TC_IER_CPCS;                      // Disable all other timer interrpts 
  NVIC_EnableIRQ(TC3_IRQn);                       // Enable interrupt vector
}

void setupTC4_Interrupt(unsigned int period, byte tcClock, void (*isr)()){
  pmc_set_writeprotect(false);                    // Enavle direct TC register manipulation
  pmc_enable_periph_clk(TC4_IRQn);                // Enable predefined interrupt handler
  TC4_CallBack=isr;                               // Route the call to user interrupt handler
  REG_TC1_CMR1=getTC_Waveform(tcClock);           // Update Command Register
  REG_TC1_RC1=period;                             // Set period (1 - 0xFFFF)
  start_TC4();                                    // Start timer and generate square wave
  REG_TC1_IER1= TC_IER_CPCS;                      // Enable RC compare interrupt
  REG_TC1_IDR1=~TC_IER_CPCS;                      // Disable all other timer interrpts 
  NVIC_EnableIRQ(TC4_IRQn);                       // Enable interrupt vector
}

void setupTC5_Interrupt(unsigned int period, byte tcClock, void (*isr)()){
  pmc_set_writeprotect(false);                    // Enavle direct TC register manipulation
  pmc_enable_periph_clk(TC5_IRQn);                // Enable predefined interrupt handler
  TC5_CallBack=isr;                               // Route the call to user interrupt handler
  REG_TC1_CMR2=getTC_Waveform(tcClock);           // Update Command Register
  REG_TC1_RC2=period;                             // Set period (1 - 0xFFFF)
  start_TC5();                                    // Start timer and generate square wave
  REG_TC1_IER2= TC_IER_CPCS;                      // Enable RC compare interrupt
  REG_TC1_IDR2=~TC_IER_CPCS;                      // Disable all other timer interrpts 
  NVIC_EnableIRQ(TC5_IRQn);                       // Enable interrupt vector
}

void changeTC2_Period(unsigned int period) {
  stop_TC2();
  REG_TC0_RC2=period;
  start_TC2();
}

void changeTC3_Period(unsigned int period) {
  stop_TC3();
  REG_TC1_RC0=period;
  start_TC3();
}

void changeTC4_Period(unsigned int period) {
  stop_TC4();
  REG_TC1_RC1=period;
  start_TC4();
}

void changeTC5_Period(unsigned int period) {
  stop_TC5();
  REG_TC1_RC2=period;
  start_TC5();
}