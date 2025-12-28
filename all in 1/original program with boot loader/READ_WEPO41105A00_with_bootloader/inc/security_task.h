// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file security_task.h
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
 * \version V1.00 Exp (experimental)
 * \author 				- Florian Merz, pro-micron
 *
 *
 * \version
 *
 * 		2009-10-03			flm			1st release
 *
 *
 * <b>Copyright &copy;2008 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>*/
//-----------------------------------------------------------------------------------------------//

/*! \brief cc2500_init
 *         create cc2500 task
 *
 */
void init_security_task( void );

/*! \brief TASK vTaskSec
 *        vTaskSec - Security Task
 *
 *  \param pvParameters   Input. Not Used.
 *
 */
void vTaskSec( void * pvParameters );
