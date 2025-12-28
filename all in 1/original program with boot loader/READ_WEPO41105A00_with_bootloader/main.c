// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file wepo_v100.c
 *
 * \brief main() function
 *
 * \details Functional description: Receiving and processing data from CC2500
 *
 * <b>Target Platforms:</b>	AVR32UC3A1256
 *
 * <b>Editor:</b> Eclipse
 *
 * <b>Compiler:</b> GNU Compiler Collection (GCC) 4.1.4
 *
 * \date 2009-04-03
 *
 * \version V1.00 Exp (experimental), Stab (stable) und Rel (released)
 * \author 						- Florian Merz, pro-micron
 *
 *
 * \version
 *
 * 		2010-03-18		flm		- Erste Version:
 * 								- Komponenten: - freeRTOS
 * 											   - CC2500 mit SPI und PDCA
 * 											   - USB, Comm. Device Class (CDC)
 *
 * <b>Copyright &copy;2010 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>*/
//-----------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------//
// INCLUDES
//-----------------------------------------------------------------------------------------------//

/* Environment include files. */
#include <stdlib.h>
#include <string.h>
#include "pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "partest.h"
#include "serial.h"
#include "flash.h"
#include "usb.h"
#include "main_app.h"
#include "flashc.h"
#include "security_task.h"
#include "spike_fw_update.h"

//-----------------------------------------------------------------------------------------------//
// Prototypes
//-----------------------------------------------------------------------------------------------//
void pll_init(void);
void reset_eth(void);

//-----------------------------------------------------------------------------------------------//
// Definitions
//-----------------------------------------------------------------------------------------------//
#define ETH_USED	   	0
#define CC2500_USED    	1
#define FREERTOS_USED  	1
#define USB_USED 	 	1

//-----------------------------------------------------------------------------------------------//
// Main Application
//-----------------------------------------------------------------------------------------------//
int main( void )
{
	/* setup pll */
	pll_init();

	/* start security Task (Watchdog) */
	init_security_task();

	
	/* USB */
	#if USB_USED
	   /* Start Application. */
	   usb_start();
	#endif

	/* Ethernet */
	#if ETH_USED
		/* reset DP83848 Transceiver */
		reset_eth();

		/* Start ethernet task. */
		vStartEthernetTaskLauncher(configMAX_PRIORITIES-2);
	#endif

	/* CC2500 */
	#if CC2500_USED
		/* init cc2500 transceiver */
		cock_application_init();
	#endif

	/* FREERTOS */
	#if FREERTOS_USED
		/* Trace Scheduler State */
		//vTaskStartTrace( trace_array, TRACE_SIZE );

		/* Start FreeRTOS. */
		vTaskStartScheduler();
	#endif

	/* Will only reach here if there was insufficient memory to create the idle task. */
		return 0;
}
/*
void pll_init(void)
{
	volatile avr32_pm_t* pm = &AVR32_PM;

	// Switch to external oscillator 0
	pm_switch_to_osc0( pm, FOSC0, OSC0_STARTUP );

	// Setup PLL0 on OSC0, mul+1=16 ,divisor by 1, lockcount=16, ie. 12Mhzx16/1 = 192MHz output.
	// Extra div by 2 => 96MHz
	pm_pll_setup(pm,	// volatile avr32_pm_t* pm
				0,		// unsigned int pll
				15,		// unsigned int mul
				1,		// unsigned int div, Sel Osc0/PLL0 or Osc1/Pll1
				0,		// unsigned int osc
				16);	// unsigned int lockcount

	pm_pll_set_option( pm, 0,   // pll0
	                       0,   // Choose the range 160-240MHz.
	                       1,   // div2
	                       0 ); // wbwdisable

	// Enable PLL0
	pm_pll_enable(pm,0);

	// Wait for PLL0 locked
	pm_wait_for_pll0_locked(pm) ;

	// switch to clock
	pm_cksel( pm, 1, 1, 1, 0, 1, 0 );
	flashc_set_wait_state( 1 );
	pm_switch_to_clock( pm, AVR32_PM_MCCTRL_MCSEL_PLL0 );
}
*/
void pll_init(void)
{
	volatile avr32_pm_t* pm = &AVR32_PM;

	/* Switch to external oscillator 0 */
	pm_switch_to_osc0( pm, FOSC0, OSC0_STARTUP );

	/* Setup PLL0 on OSC0, mul+1=11 ,divisor by 1, lockcount=16, ie. 12Mhzx11/1 = 132MHz output.*/
	pm_pll_setup(pm,	/* volatile avr32_pm_t* pm */
				0,		/* unsigned int pll */
				10,		/* unsigned int mul */
				1,		/* unsigned int div, Sel Osc0/PLL0 or Osc1/Pll1 */
				0,		/* unsigned int osc */
				16);		/* unsigned int lockcount */

	pm_pll_set_option( pm, 0,   // pll0
	                       1,   // Choose the range 160-240MHz. //0
	                       0,   // div2
	                       0 ); // wbwdisable

	/* Enable PLL0 */
	pm_pll_enable(pm,0);

	/* Wait for PLL0 locked */
	pm_wait_for_pll0_locked(pm) ;

	/* switch to clock */
	//pm_cksel( pm, 1, 1, 1, 0, 1, 0 );
	  pm_cksel( pm, 0, 0, 0, 0, 1, 0 );
	flashc_set_wait_state( 1 );
	pm_switch_to_clock( pm, AVR32_PM_MCCTRL_MCSEL_PLL0 );
}

void reset_eth(void)
{
	/* Counter for the delay loop */
	int i;

	/* since we have no hardreset, we have to reset the Eth. Transceiver manually */
	gpio_clr_gpio_pin(AVR32_PIN_PA26);

	/* delay */
	for(i=0;i<1000000;i++){}
		gpio_set_gpio_pin(AVR32_PIN_PA26);
}

