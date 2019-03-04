/*
  DueTC.h - Interface for Arduino DUE Timer Control implemented in DueTC.cpp

Created by Olavi Kamppari, October 2013.
Released into the public domain.
*/

#ifdef __arm__

#ifndef DueTC_h
#define DueTC_h

#include "Arduino.h"

// tcClock values for Master Clock (MCK) frequency 84 MHz
//
// Duty cycle with different settings
//
//  0: MCK/2     = 42 MhZ,  period 1 = 48 ns,   period 0xffff = 3.1 ms,  320 Hz - 21 MHz
//  1: MCK/8     = 10.5 MhZ,period 1 = 190 ns,  period 0xffff = 12 ms,    80 Hz - 5.3 MHz
//  2: MCK/32    = 2.6 MhZ, period 1 = 762 ns,  period 0xffff = 50 ms,    20 Hz - 1.3 MHz
//  3: MCK/128   = 656 khZ, period 1 = 3.0 us,  period 0xffff = 200 ms,    5 Hz - 328 kHz
//  4: MCK/3072  = 28 khZ,  period 1 = 72 us,   period 0xffff = 4.7 s,   0.2 Hz - 14 kHz
//
// The option 4 is also known as SLCK (slow clock) which is supposed to be at 32 kHz

// Digital Output Pins for square waves
// ++++++++++++++++++++++++++++++++++++
// Timer 0
//      TC0 in channel 0: A on PWM2,   B on PWM13
void setupTC_Pin2_Timing(unsigned int period, byte tcClock);
void changeTC_Pin2_Period(unsigned int period);

// Timer 1, no channels awailable for square waves

// Timer 2
//      TC 6 in channel 0: A on PWM5,   B on PWM4
//      TC 7 in channel 1: A on PWM3,   B on PWM10
//      TC 8 in channel 2: A on PWM11,  B on PWM12
void setupTC_Pin5_Timing(unsigned int period, byte tcClock);
void setupTC_Pin3_Timing(unsigned int period, byte tcClock);
void setupTC_Pin11_Timing(unsigned int period, byte tcClock);

void changeTC_Pin5_Period(unsigned int period);
void changeTC_Pin3_Period(unsigned int period);
void changeTC_Pin11_Period(unsigned int period);

// Timers for interrupts
// +++++++++++++++++++++
//
// Timer 1 is used for system services (such as milli, micros, and delay)
// Timers 0, 6, 7, and 8 are reserved for square wave generation
// Timers 2-5 are available for timed interrupts
//
void setupTC2_Interrupt(unsigned int period, byte tcClock, void (*isr)());
void setupTC3_Interrupt(unsigned int period, byte tcClock, void (*isr)());
void setupTC4_Interrupt(unsigned int period, byte tcClock, void (*isr)());
void setupTC5_Interrupt(unsigned int period, byte tcClock, void (*isr)());

void changeTC2_Period(unsigned int period);
void changeTC3_Period(unsigned int period);
void changeTC4_Period(unsigned int period);
void changeTC5_Period(unsigned int period);

#endif

#else
#error DueTC works only on Arduino DUE ARM based processor
#endif

