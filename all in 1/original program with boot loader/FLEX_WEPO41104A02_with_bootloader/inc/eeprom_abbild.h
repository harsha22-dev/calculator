/**\file *******************************************************************************************
 * 
 * \brief EEPROM data structure and EEPROM I/O-functions
 *
 * \details Functional description:		
 *
 * <b>Target Platforms:</b> ATmega324P
 *
 * <b>Editor:</b>           Atmel AVR Studio 5 (Version: 5.1.208)
 *
 * <b>Compiler:</b>			AVRGCC (Version: 3.3.1.27)  
 * 
 * \date 2013-09-05
 * \version WEPO41104A02
 * \author created by Andre Kuhn
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 *
 **************************************************************************************************/

#ifndef EEPROM_ABBILD_H_
#define EEPROM_ABBILD_H_

//-----------------------------------------------------------------------------------------------//
// Header Files
//-----------------------------------------------------------------------------------------------//

#include "avr/eeprom.h"			// EEPROM functions
#include "SPIKE_config.h"		// SPIKE configuration file

//-----------------------------------------------------------------------------------------------//
// Symbolic Constants, Defines and Macros
//-----------------------------------------------------------------------------------------------//
#define EEPROM_SG_1		0
#define EEPROM_SG_2		1
#define EEPROM_SG_3		2
#define EEPROM_SG_4		3

#define EEPROM_SX8723C_REGCEN		0
#define EEPROM_SX8723C_REGOUT		1
#define EEPROM_SX8723C_REGTIMEOUT	2
#define EEPROM_SX8723C_REGEXTADD	3
#define EEPROM_SX8723C_REGACCFG0	4
#define EEPROM_SX8723C_REGACCFG1	5
#define EEPROM_SX8723C_REGACCFG2	6
#define EEPROM_SX8723C_REGACCFG3	7
#define EEPROM_SX8723C_REGACCFG4	8
#define EEPROM_SX8723C_REGACCFG5	9
#define EEPROM_SX8723C_REGMODE		10

//-----------------------------------------------------------------------------------------------//
// Global variables
//-----------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------//
// Typedefs, enums and structs
//-----------------------------------------------------------------------------------------------//

/**
 * \struct eeprom_data
 * \brief Data to store in EEPROM.
 */
typedef struct
__attribute__((__packed__))
{
    uint16_t serial_number;				/**< Serial number of the SPIKE.*/
    uint8_t	 SX8723c_config[4][11];		/**< SX8723c data to initialize the register.*/
    uint16_t sw_version;				/**< Firmware version of the SPIKE.*/
    uint16_t hw_version;				/**< Hardware version of the Flex.*/
    uint16_t src_address;				/**< Source address of the transceiver chip.*/
    uint16_t dest_address;				/**< Destination address of the transceiver chip.*/
    uint16_t pan_id;					/**< PAN ID for the wireless communication.*/
    uint8_t  channel;					/**< Channel for the wireless communication.*/
    uint8_t  channel_page;				/**< Channel page configuration to transceiver chip.*/
    uint8_t  tx_pwr;					/**< Tx power of the transceiver.*/
    uint16_t timeout;					/**< Timeout [s], after this time controller goes in sleep mode.*/
    uint16_t samplerate;				/**< Sample rate [1/s] of the measurement.*/
    int16_t  SG_software_offset[4];		/**< Strain gauge software offset done by offset function.*/
    int32_t  calibration_values[8];		/**< Calibration values, stored by SPIKE PC software .*/
}eeprom_data;

//-----------------------------------------------------------------------------------------------//
// Prototypes
//-----------------------------------------------------------------------------------------------//

/**
 *\brief Load configuration from internal EEPROM and save it in SRAM.
 *\param ptr_sram_data Pointer to variable in SRAM.
 */
void eeprom_read_config(eeprom_data* ptr_sram_data);

/**
 *\brief Laod configuration from internal SRAM and write it to EEPROM. 
 *\param ptr_sram_data Pointer to variable in SRAM.
 */
void eeprom_save_config(eeprom_data* ptr_sram_data);

#endif /* EEPROM_ABBILD_H_ */