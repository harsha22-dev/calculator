// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file cc2500.c
 *
 * \brief Interfacing an AD5410
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
 * \author 		- Florian Merz, pro-micron
 *
 *
 *
 * \version
 *		2009-08-11			flm					 - Grundgerüst
 *
 * <b>Copyright &copy;2008 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>
//-----------------------------------------------------------------------------------------------//                                                                                               */

/* Includes */
#include "portmacro.h"
#include "compiler.h"
#include <avr32/io.h>
#include "gpio.h"
#include "delay.h"
#include "spi.h"
#include "board.h"
#include "stdint.h"

/* Definitions */
#define AD5410_ADDR_NOP 	0x00
#define AD5410_ADDR_DATA 	0x01
#define AD5410_ADDR_CR 		0x55
#define AD5410_ADDR_RST 	0x56
#define AD5410_ADDR_READ	0x02
#define AD5410_NOP			0x00

#define RANGE_4_20mA		0x05
#define RANGE_0_20mA		0x06
#define RANGE_0_24mA		0x07

/* Typedefs */
typedef struct
{
	uint16_t reserved:	2;
	uint16_t rext:		1;
	uint16_t outen:		1;
	uint16_t sr_clock: 	4;
	uint16_t sr_step: 	3;
	uint16_t sren: 		1;
	uint16_t dcen: 		1;
    uint16_t range: 	3;
} ad5410_config_struct;

/* API */
uint8_t ad5410_init( void );
uint8_t ad5410_set_current(uint16_t current);
uint8_t ad5410_reset( void );
uint8_t ad5410_read_status_reg( uint16_t * status );
uint8_t ad5410_read_data_reg( uint16_t * data );
uint8_t ad5410_read_control_reg( uint16_t * data );
uint8_t ad5410_write(uint8_t addr, uint16_t data);
uint16_t ad5410_read(uint8_t addr);
