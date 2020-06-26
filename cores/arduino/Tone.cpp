/*
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA



    Tone implementation for the Arduino Due / SAM boards, written by Mikhail Khrenov
    (@redrussianbear, mkhrenov) on 2020/06/25, based on Tone for SAMD boards, with
    info on Due Timer Counters taken from "Arduino Due Timers" by Jeff ko7m.
*/

#include "Tone.h"
#include <Arduino.h>

#include "variant.h"

static uint32_t last_output_pin = -1;

static volatile Pio *port_pio_registers;
static volatile uint32_t port_bitmask;
static volatile int64_t toggleCount;
static volatile bool tone_is_active = false;

// Timer Counter names, Interrupt Request handlers, channel
#define TONE_TC         TC1
#define TONE_TC_IRQn    TC5_IRQn
#define TONE_TC_PMC     ID_TC5
#define TONE_TC_CHANNEL 2

// Map TC5_Handler to tone_handler to run pin toggling interrupt
void TC5_Handler(void) __attribute__ ((weak, alias("tone_handler")));

void tone(uint32_t outputPin, uint32_t frequency, uint32_t duration) {
    // Avoid divide by zero error by calling 'noTone' instead
    if (frequency == 0) {
        noTone(outputPin);
        return;
    }

    // Disable and clear existing interrupt requests
    NVIC_DisableIRQ(TONE_TC_IRQn);
    NVIC_ClearPendingIRQ(TONE_TC_IRQn);

    NVIC_SetPriority(TONE_TC_IRQn, 0);

    // Turn off Power Management Controller protections and enable peripheral clock
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk(TONE_TC_PMC);

    TC_Configure(TONE_TC, TONE_TC_CHANNEL, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);

    // If swapping pins, disable tone on old pin
    if (tone_is_active && (outputPin != last_output_pin))
        noTone(last_output_pin);

    // If swapping pins, record and set initial state low
    if (outputPin != last_output_pin) {
        last_output_pin = outputPin;
        digitalWrite(outputPin, LOW);
        pinMode(outputPin, OUTPUT);
        tone_is_active = true;
    }

    // Compute total number of toggles to finish duration, if duration is zero or less set to -1 (indefinite)
    toggleCount = duration > 0 ? duration * frequency / 1000 : -1;


    // Retrieve PIO registers for required output pin, store in globals
    port_pio_registers = g_APinDescription[outputPin].pPort;
    port_bitmask = g_APinDescription[outputPin].ulPin;

    // Set register A to got high at 50% period, register C to go low at 100% period?
    uint32_t cutoff = VARIANT_MCK / 128 / frequency; // / 128
    TC_SetRA(TONE_TC, TONE_TC_CHANNEL, cutoff / 2);
    TC_SetRC(TONE_TC, TONE_TC_CHANNEL, cutoff);

    TC_Start(TONE_TC, TONE_TC_CHANNEL);

    // Enable register C and A compare interrupts, disable all others
    TONE_TC->TC_CHANNEL[TONE_TC_CHANNEL].TC_IER = TC_IER_CPCS | TC_IER_CPAS;
    TONE_TC->TC_CHANNEL[TONE_TC_CHANNEL].TC_IDR = ~(TC_IER_CPCS | TC_IER_CPAS);

    NVIC_EnableIRQ(TONE_TC_IRQn);
}

void noTone(uint32_t outputPin) {
    TC_Stop(TONE_TC, TONE_TC_CHANNEL);
    digitalWrite(outputPin, LOW);
    tone_is_active = false;
}

#ifdef __cplusplus
extern "C" {
#endif
// Bound to TC5_Handler above
void tone_handler(void) {
    // Clear the interrupt
    TC_GetStatus(TONE_TC, TONE_TC_CHANNEL);

    if (toggleCount != 0) {
        // Toggle the ouput pin
        if (port_pio_registers->PIO_ODSR & port_bitmask) {
            // If high, go low
            port_pio_registers->PIO_CODR = port_bitmask;
        } else {
            // Else, go high
            port_pio_registers->PIO_SODR = port_bitmask;
        }

        if (toggleCount > 0)
            toggleCount--;
    } else {
        TC_Stop(TONE_TC, TONE_TC_CHANNEL);
        port_pio_registers->PIO_CODR = port_bitmask;         // Take pin low
        tone_is_active = false;
    }
}

#ifdef __cplusplus
}
#endif