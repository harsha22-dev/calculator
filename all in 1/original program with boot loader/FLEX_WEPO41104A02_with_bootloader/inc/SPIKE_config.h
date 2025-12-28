/**\file *******************************************************************************************
 * 
 * \brief SPIKE configuration.
 *
 * \details Functional description:
 *
 * <b>Target Platforms:</b> ATmega324P
 *
 * <b>Editor:</b>           Atmel AVR Studio 5 (Version: 5.1.208)
 *
 * <b>Compiler:</b>			AVRGCC (Version: 3.3.1.27) 
 * 
 * \date 2013-09-11
 * \version WEPO41104A02
 * \author created by flm, mih, job, ank
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 *
 **************************************************************************************************/


#ifndef SPIKE_CONFIG_H_
#define SPIKE_CONFIG_H_

// SPIKE config
#define SERIAL_NUMMBER	0xa00F		// uint16_t
#define HW_VERSION		0x0005		// uint16_t
#define SW_VERSION		0x0004		// uint16_t

#define TIMEOUT			0			// [s] max. 300s; 0 = always on

// at86rf231 config
#define SRC_ADDRESS		0xa00F		// uint16_t
#define CHANNEL			0x12		// uint8_t; 0x0B bis 0x1A

// Use temperature sensor
#define USE_DS620

#endif /* SPIKE_CONFIG_H_ */