/**\file *******************************************************************************************
 * 
 * \brief Main header of the program.
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
 * \author created by flm, mih, job, ank
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 *
 **************************************************************************************************/

#ifndef WEPO41104A01_H_
#define WEPO41104A01_H_

#ifndef F_CPU
#define F_CPU 10000000          //!< CPU clock (in Hz)
#endif

//-----------------------------------------------------------------------------------------------//
// Header Files
//-----------------------------------------------------------------------------------------------//

#include <avr/io.h>             
#include <inttypes.h>           
#include <avr/Interrupt.h>      
#include <ctype.h>              
#include <avr/sleep.h>          
#include <util/delay.h>  
#include <avr/signature.h>       

#include "eeprom_abbild.h"		// data in the EEPROM section
#include "SX8723c.h"			// strain gauge sensor
#include "ds620.h"				// temperature sensor
#include "at86.h"               // AT86RF231 API
#include "SPIKE_config.h"		// configuration file SPIKE		
#include "pm_config_protocol.h"	// pm configuration protocol
#include "conf_LED.h"			// LED configuration
#include "fuses.h"				// data in the fuse section 

//-----------------------------------------------------------------------------------------------//
// Symbolic Constants, Defines and Macros
//-----------------------------------------------------------------------------------------------//

#define SG_1	0		//!< number of the first strain gauge
#define SG_2	1		//!< number of the secound strain gauge
#define SG_3	2		//!< number of the third strain gauge
#define SG_4	3		//!< number of the fourth strain gauge

#define BATTERY_THRESHOLD_EMPTY		171	//!< threshold = battery empty (digit)

//-----------------------------------------------------------------------------------------------//
// Typedefs, enums and structs
//-----------------------------------------------------------------------------------------------//

//! A structure to present the states of the finite state machine.
typedef enum{
	STATE_INIT					=	0x00,	//!< State initzialization.
	STATE_SLEEP					=	0x01,	//!< State sleep.
	STATE_WAKE_UP				=	0x02,	//!< State wake up from sleet.
	STATE_MEASUREMENT_PREPARE	=	0x03,	//!< State measurement prepare.
	STATE_MEASUREMENT			=	0x04,	//!< State do measurement.
	STATE_MEASUREMENT_CLOSE		=	0x05,	//!< State measurement close.
	STATE_SEND					=	0x06,	//!< State send payload.
	STATE_CONFIG				=	0x07	//!< State get configuration.
}state_t;

//-----------------------------------------------------------------------------------------------//
// Global variables
//-----------------------------------------------------------------------------------------------//

extern volatile at86rf231_frame at86rf231_tx_frame;		//!< global transmit buffer for the AT86RF231
extern volatile at86rf231_frame at86rf231_rx_frame;		//!< global receive buffer for the AT86RF231
extern tal_state_t tal_state;							//!< Transceiver states
extern volatile uint8_t tx_end_irq_flag;			//!< Transmit interrupt flag = 1 transmission completed

uint32_t timeout_timer = 0;				//!< timeout timer to switch into sleep mode
uint8_t timeslot_number = 0;			//!< time slot number in transmitted payload
volatile uint8_t use_acceleration_sensor_flag = 1;	//!< turn acceleration measurement on if flag = 1
volatile uint8_t timer_flag = 0;		//!< flag = 1 if timer interrupt occurred 
volatile int16_t temperature = 0;		//!< measurement value of the temperature sensor
volatile bool stop_stream = 0;			//!< flag to stop sending data

#if defined(USE_DS620)
	uint16_t temperature_timer = 0;		//!< timer for next measurement
#endif

volatile state_t current_state = 0x00;		//!< current state of the state machine

spike_payload *ptr_payload_spike = (spike_payload*) at86rf231_tx_frame.payload;		//!< pointer to payload

eeprom_data eeprom_sram_data = { 0 };						//!< EEPROM image in SRAM
eeprom_data *ptr_eeprom_sram_data = &eeprom_sram_data;		//!< pointer to EEPROM image in SRAM

//-----------------------------------------------------------------------------------------------//
// Prototypes
//-----------------------------------------------------------------------------------------------//

/*!
 *\brief Main function
 */
int main(void);

/*!
 *\brief Initialized the timer/counter 1 
 * \details The timer generates the timing for the measurement.
 */
void timer_init(void);

/*!
 *\brief Initialization of the controller and all sensors
 */
void avr_init(void);

/*!
 *\brief Initialized the ADC
 */
void init_adc(void);

/*!
 *\brief Set micro controller to sleep mode
 */
void set_sleep(void);

/*!
 *\brief Function to send a data packet 
 */
void static inline send_all_data_rf231(void);

/*!
 *\brief Interrupt handler of the timer/counter 1 
 */
ISR( TIMER1_COMPA_vect );

/*!
 *\brief Interrupt handler of the analog comparator 
 */
ISR( ANALOG_COMP_vect );

void LED_blink(uint16_t duration);

#endif /* WEPO41104A01_H_ */

