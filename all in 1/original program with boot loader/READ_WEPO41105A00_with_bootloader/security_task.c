// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file security_task.c
 *
 * \brief Implement a watchdog used for all tasks
 *
 * \details Functional description: Function for initialize and resetting the uC and watchdog
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
 * \author 				- Florian Merz, pro-micron
 *
 * \bug	...
 * \todo ...
 *
 *
 * \version
 *
 * 		2009-10-03			flm			1st release
 *
 *
 * <b>Copyright &copy;2008 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>
//-----------------------------------------------------------------------------------------------//                                                                                               */

//-----------------------------------------------------------------------------------------------//
// Header Files
//-----------------------------------------------------------------------------------------------//
#include "FreeRTOS.h"
#include "task.h"
#include "wdt.h"
#include "usb.h"
#include "security_task.h"

//-----------------------------------------------------------------------------------------------//
// Main Application
//-----------------------------------------------------------------------------------------------//
void init_security_task( void )
{
	/* Handle for the Task */
	xTaskHandle xHandleSecTask;

	/* we need no Paramteres to pass to the Task */
	unsigned char ucParameterToPass = 0;

	/* Create task  */
	xTaskCreate( vTaskSec,							// pvTaskCode
			     ( signed portCHAR * ) "SEC_TSK",	// pcName
                 265,								// usStackDepth
                 &ucParameterToPass,				// pvParameters
                 tskIDLE_PRIORITY,					// uxPriority
                 &xHandleSecTask );					// pvCreatedTask

}

void vTaskSec( void * pvParameters )
{
	/* init Watchdog 10 sec */
	wdt_enable( 100000000 );

	/* loop forever */
	while( 1 )
	{
		/* Watchdog immer zurücksetzen wenn dieser TASK abgearbeitet wird. */
		wdt_clear();
		
	}
}


