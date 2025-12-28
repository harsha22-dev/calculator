
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

#ifndef SW_TWI_H_
#define SW_TWI_H_

/**
 * \defgroup group_SX8723c_service_sw_twi Semtech SX8723c Software TWI
 *
 * \ingroup	group_SX8723c_service
 *
 * The software TWI allows a parallel communication with all SX8723c at the same time. 
 *
 * \{
 */

//-----------------------------------------------------------------------------------------------//
// Header Files
//-----------------------------------------------------------------------------------------------//
#include "conf_sw_twi.h"

#ifdef SW_TWI_BLOCK_INTERRUPT	// see conf_sw_twi.h
#include <avr/Interrupt.h>   
#endif

//-----------------------------------------------------------------------------------------------//
// Symbolic Constants, Defines and Macros
//-----------------------------------------------------------------------------------------------//
#define nop()	asm volatile("nop\n\t" ::)
#define nop_2() asm volatile("nop\n\t" "nop\n\t" ::)						 
#define nop_3() asm volatile("nop\n\t" "nop\n\t" "nop\n\t" ::)					 
#define nop_4() asm volatile("nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" ::)					   
#define nop_5() asm volatile("nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" ::)					   
#define nop_6() asm volatile("nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" ::)
#define nop_8() asm volatile("nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" ::)
						   
#define SET_SCL		{SCL_PORT |= (1 << SCL_PIN); SCL_DDR &= ~(1 << SCL_PIN); while(!(SCL_PIN_ADDR & SCL_PORT_MASK)); nop_4();}	//!< pin high and as input									
#define CLEAR_SCL	{SCL_PORT &= ~(1 << SCL_PIN); SCL_DDR |= (1 << SCL_PIN); nop_3();}	//!< pin low and as output

#define SET_SCL_SHORT	{SCL_PORT |= (1 << SCL_PIN); SCL_DDR &= ~(1 << SCL_PIN); while(!(SCL_PIN_ADDR & SCL_PORT_MASK));}  //!< pin high and as input											
#define CLEAR_SCL_SHORT	{SCL_PORT &= ~(1 << SCL_PIN); SCL_DDR |= (1 << SCL_PIN);}	//!< pin low and as output

#define SET_SDA(x)		{SDA_PORT |= (x); SDA_DDR &= ~(x); nop_2();}	//!< as input and pin high; x = port_mask									 
#define CLEAR_SDA(x)	{SDA_PORT &= ~(x); SDA_DDR |= (x); nop_2();}	//!< pin low and as output
	
#define SET_SDA_SHORT(x)	{SDA_PORT |= (x); SDA_DDR &= ~(x);}	//!< as input and pin high; x = port_mask
#define CLEAR_SDA_SHORT(x)  {SDA_PORT &= ~(x); SDA_DDR |= (x);}	//!< pin low and as output

//-----------------------------------------------------------------------------------------------//
// Global variables
//-----------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------//
// Typedefs, enums and structs
//-----------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------//
// Prototypes
//-----------------------------------------------------------------------------------------------//

/**
 * \brief TWI command to write a byte in a single register of the SX8723c.
 * \param mask Port mask, describes the SX8723c which should be written.
 * \param reg Register which should be written.
 * \param data Data byte which should be written.
 */
void sw_TWI_write_single_register(uint8_t mask, uint8_t reg, uint8_t data);

/**
 * \brief TWI command to read a byte in a single register of the SX8723c.
 * \param mask Port mask, describes the SX8723c which should be readout.
 * \param reg Register which should be readout.
 * \param data Data which is read from the SX8723c.
 */
void sw_TWI_read_single_register(uint8_t mask, uint8_t reg, uint8_t* data);

/**
 * \brief TWI command to readout the 16 bit data value of the SX8723c.
 * \param mask Port mask, describes the SX8723c which should be readout.
 * \param data Data which is read from the SX8723c.
 */
void sw_TWI_read_value_registers(uint8_t mask, int16_t* data);

/**
 * \}
 */

#endif /* SW_TWI_H_ */