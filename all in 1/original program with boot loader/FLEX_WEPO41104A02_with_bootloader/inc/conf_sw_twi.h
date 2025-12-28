/**\file *******************************************************************************************
 * 
 * \brief Software TWI for parallel read/write to SX8723c devices
 *
 * \details Functional description:
 *  Software TWI to communicate with the SX8723c in parallel mode.
 *		
 *
 * <b>Target Platforms:</b> ATmega324PV
 *
 * <b>Editor:</b>           Atmel AVR Studio 5 (Version: 5.1.208)
 *
 * <b>Compiler:</b>			AVRGCC (Version: 3.3.1.27)    
 * 
 * \date 2013-09-09
 * \version WEPO41104A02
 * \author created by Andre Kuhn
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 *
 **************************************************************************************************/

#ifndef CONF_SW_TWI_H_
#define CONF_SW_TWI_H_

/**
 * \defgroup group_SX8723c_service_sw_twi_conf Semtech SX8723c Software TWI Configuration
 *
 * \ingroup	group_SX8723c_service_sw_twi
 *
 * Configuration file of the Software TWI to set the I/O-ports and other options.
 *
 * \{
 */

#include <avr/io.h>

// uncomment if interrupts should not be blocked while send or receive an TWI packet
#define SW_TWI_BLOCK_INTERRUPT

// SDA I/O-config
#define SDA_PORT		PORTA		//!< SDA port
#define SDA_DDR			DDRA		//!< SDA direction register
#define SDA_PIN			PINA		//!< SDA pin address
#define SDA_PORT_MASK	0x0F		//!< SDA port mask

// SCL IO-config
#define SCL_PORT		PORTA		//!< SCL port
#define SCL_DDR			DDRA		//!< SCL direction register
#define SCL_PIN			PINA4		//!< SCL pin number
#define SCL_PIN_ADDR	PINA		//!< SCL pin address
#define SCL_PORT_MASK	0x10		//!< SCL port mask

/* port mask example
 * used I/O-port = 1
 *
 *_______ 
 *       |
 * PINA7 |-- unused
 * PINA6 |-- unused
 * PINA5 |-- unused
 * PINA4 |---- SCL TWI device 1-4
 * PINA3 |---- SDA TWI device 4
 * PINA2 |---- SDA TWI device 3
 * PINA1 |---- SDA TWI device 2
 * PINA0 |---- SDA TWI device 1
 *_______|
 *
 * --> PORT_MASK = 0x0F	(0b0000 1111)
*/

#define device_1	0x01	//!< port-mask of device 1
#define device_2	0x02	//!< port-mask of device 2
#define device_3	0x04	//!< port-mask of device 3
#define device_4	0x08	//!< port-mask of device 4

#define device_all	0x0F	//!< port-mask of all devices

/**
 * \}
 */

#endif /* CONF_SW_TWI_H_ */