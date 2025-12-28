/**\file *******************************************************************************************
 * 
 * \brief DS620 Functions.
 *
 * \details Functional description:
 *
 * <b>Target Platforms:</b> ATmega324P
 *
 * <b>Editor:</b>           Atmel AVR Studio 5 (Version: 5.1.208)
 *
 * <b>Compiler:</b>			AVRGCC (Version: 3.3.1.27) 
 * 
 * \date 2013-09-09
 * \version WEPO41104A02
 * \author created by flm, mih, job, ank
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 *
 **************************************************************************************************/

#ifndef DS620_H_
#define DS620_H_

/**
 * \defgroup group_sd620 Temperature Sensor DS620
 *
 * The Temperature Sensor DS620 module includes project-specific definitions
 * and function prototypes, such as the Temperature Sensor DS620 initialization function.
 *
 * \{
 */

//-----------------------------------------------------------------------------------------------//
// Header Files
//-----------------------------------------------------------------------------------------------//
#include <avr/io.h>

//-----------------------------------------------------------------------------------------------//
// Symbolic Constants, Defines and Macros
//-----------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------//
// Global variables
//-----------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------//
// Typedefs, enums and structs
//-----------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------//
// Prototypes
//-----------------------------------------------------------------------------------------------//

/*! \brief Initializes the TWI of the temperature sensor 
 */
void ds620_twi_init(void);

/*! \brief Send an TWI Start condition.
 * \param addr Chip address.
 */
unsigned char ds620_twi_start( unsigned char addr );

/*! \brief Send one byte.
 * \param data Byte to send.
 */
void ds620_twi_send( unsigned char data );

/*! \brief Read one byte.
 * \param ack 0: NACK, 1: ACK
 */
uint8_t ds620_twi_read( uint8_t ack );

/*! \brief Send a stop condition.
 */
void ds620_twi_stop( void );

/*! \brief Initializes the DS620.
 */
void ds620_init( void );

/*! \brief Set DS620 to sleep.
 */
void ds620_stop_convert( void );

/*! \brief Starts a conversation or wake up from sleep.
 */
void ds620_start_convert( void );

/*! \brief Get the temperature value of the sensor. 
 */
int16_t ds620_get_temperature( void );

/**
 * \}
 */

#endif /* DS620_H_ */