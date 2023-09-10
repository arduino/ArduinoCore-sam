/*
  Copyright (c) 2011 Arduino.  All right reserved.

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
*/

#include "variant.h"

/*
 * Pin             |  PORT  | Label
 * ----------------+--------+-------
 *   0       	   |  PB2  	| "Serial RX"
 *   1       	   |  PB3  	| "Serial TX"
 *   2       AD5   |  PB1 	| "A0 : Vbatt sense"
 *   3       	   |  PA0  	| "Pixel DTA"
 *   4       	   |  PA1   | "Pixel CLK"
 *   5       	   |  PA9   | "User Button"
 *   6       	   |  PA20  | "USB-Sense"
 *   7       	   |  PA10  | "Acc INT 1"
 *   8             |  PA8	| "Acc INT 2"
 *   9             |  PA7	| "Gyr INT 1"
 *  10             |  PA7   | "Gyr INT 2"
 *  11       TWCK0 |  PA4   | "SCL"
 *  12       TWD0  |  PA3   | "SDA"
 *  13       MOSI  |  PA13	| "MOSI
 *  14       MISO  |  PA12	| "MISO"
 *  15       SCK   |  PA14  | "SCK"
 *  16       NPCS0 |  PA11  | "CS"
 *  17       	   |  PA15  | "SPI INT"
 */

#ifdef __cplusplus
extern "C" {
#endif


/* USB Functions used by USBCore.cpp */

/* end USB Functions */


/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=
{
  // ----------------------
	// 0/1 - UART (Serial) == UART1
	{ PIOB, PIO_PB2A_URXD1,  ID_PIOB, PIO_PERIPH_B	, PIO_DEFAULT,  PIN_ATTR_DIGITAL,NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // URXD 
	{ PIOB, PIO_PB3A_UTXD1,  ID_PIOB, PIO_PERIPH_B	, PIO_DEFAULT,  PIN_ATTR_DIGITAL,NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // UTXD 
	// 2 - ADC -BATT SENSE
	{ PIOB, PIO_PB1X1_AD5,    ID_PIOB, PIO_INPUT	, PIO_DEFAULT, PIN_ATTR_ANALOG,  ADC0,   ADC5,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD0 
	// 3,4 - pixel data   ( 3 = OUT/DATA, 4 = clk )
	{ PIOA, PIO_PA0		 ,  ID_PIOA, PIO_OUTPUT_0		, PIO_DEFAULT , PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER     }, // DO DEFAULT LOW /
	{ PIOA, PIO_PA1		 ,  ID_PIOA, PIO_OUTPUT_0		, PIO_DEFAULT , PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER     }, // DI  
	// 5 - pushbutton 
	{ PIOA, PIO_PA9		 ,  ID_PIOA, PIO_INPUT		, PIO_DEFAULT , PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER     }, // DI 
	// 6 - USB sense
	{ PIOA, PIO_PA20	,  ID_PIOA, PIO_INPUT		, PIO_DEFAULT , PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER     }, // DI  
	// 7,8,9,10 - ACC1,ACC2,GYR1,GYR2 INTERRUPT pins
	{ PIOA, PIO_PA10	,  ID_PIOA, PIO_INPUT		, PIO_DEFAULT , PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER     }, // DI  
	{ PIOA, PIO_PA8		,  ID_PIOA, PIO_INPUT		, PIO_DEFAULT , PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER     }, // DI  
	{ PIOA, PIO_PA7		,  ID_PIOA, PIO_INPUT		, PIO_DEFAULT , PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER     }, // DI  
	{ PIOA, PIO_PA6		,  ID_PIOA, PIO_INPUT		, PIO_DEFAULT , PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER     }, // DI  		
	// 11/12 - TWI0
	{ PIOA, PIO_PA3A_TWD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD0 - SDA0
	{ PIOA, PIO_PA4A_TWCK0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK0 - SCL0 
	// 13,14,15 - SPI
	{ PIOA, PIO_PA13A_MOSI,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // M0Si 
	{ PIOA, PIO_PA12A_MISO,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MISO 
	{ PIOA, PIO_PA14A_SPCK,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPCK 
	// 16 - SPI CS0
	{ PIOA, PIO_PA11A_NPCS0, ID_PIOA,PIO_PERIPH_A,PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS0
	// 17 - SPI INT
	{ PIOA, PIO_PA15		,  ID_PIOA, PIO_INPUT		, PIO_DEFAULT , PIN_ATTR_DIGITAL, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER     }, // DI  
		
  // 18 - TWI0 all pins
  { PIOA, PIO_PA3A_TWD0|PIO_PA4A_TWCK0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // DONE
   // 19 - UART1 (Serial) all pins
  { PIOB, PIO_PB3A_UTXD1|PIO_PB2A_URXD1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // DONE
 // END
  { NULL, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
} ;

uint8_t g_pinStatus[PINS_COUNT] = {0};
#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
RingBuffer rx_buffer1;
RingBuffer tx_buffer1;

UARTClass Serial(UART1, UART1_IRQn, ID_UART1, &rx_buffer1, &tx_buffer1);
void serialEvent() __attribute__((weak));
void serialEvent() { }

// IT handlers
void UART1_Handler(void)
{
	Serial.IrqHandler();
}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
}

// ----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

void init( void )
{
  SystemInit();

  // Set Systick to 1ms interval, common to all SAM3/SAM4 variants
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    // Capture error
     while (true);
  }

  // Disable watchdog // from flutter
  WDT_Disable(WDT);

  // Initialize C library
  __libc_init_array();

  // Disable pull-up on every pin
  for (unsigned i = 0; i < PINS_COUNT; i++)
	  digitalWrite(i, LOW);

  // Enable parallel access on PIO output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;
  


  // Initialize Serial port U(S)ART pins
  PIO_Configure(
    g_APinDescription[PINS_UART].pPort,
    g_APinDescription[PINS_UART].ulPinType,
    g_APinDescription[PINS_UART].ulPin,
    g_APinDescription[PINS_UART].ulPinConfiguration);
  digitalWrite(0, HIGH); // Enable pullup for RX0

  // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);

  // Initialize analogOutput module
   analogOutputInit();

//Add this here or delay(1) fails in main.cpp due to SysTick not firing. Not sure why.
 // cpu_irq_enable();
}

#ifdef __cplusplus
}
#endif

