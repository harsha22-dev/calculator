// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file tca6424.c
 *
 * \brief Interfacing a TCA6424 I/O portexpander
 *
 * <b>Target Platforms:</b> AVR32UC3A1256
 *
 * <b>Editor:</b> Eclipse
 *
 * <b>Compiler:</b> GNU Compiler Collection (GCC) 4.1.4
 *
 * \date 2009-04-03
 *
 * \version V1.00 Exp (experimental), Stab (stable) und Rel (released)
 * \author      - Florian Merz, pro-micron
 *
 * \todo: - check spi access for errors
 *        - check if function parameters are valid
 *
 * \version
 *      2010-03-10          flm     - file generated
 *
 * <b>Copyright &copy;2008 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>
//-----------------------------------------------------------------------------------------------//                                                                                               */

/* Header Files */
#include <avr32/io.h>
#include "board.h"
#include "print_funcs.h"
#include "gpio.h"
#include "pm.h"
#include "intc.h"
#include "twi.h"
#include "tca6424.h"
#include "main_app.h"


#include <math.h>
#include <inttypes.h>
#include "at86.h"
#include "tal_constants.h"
#include "ieee_const.h"
#include "string.h"

#define TCA6424_ADDRESS           0x22        // Chip address
#define TCA6424_SPEED             50000       // Speed of TWI

volatile uint32_t LEDs=0;

uint8_t tca6424_output(uint32_t output)
{
	static uint32_t LEDs_alt=0x0001;
	if(output==LEDs_alt)
		return 1;
	LEDs_alt = output;
	output=~output;
    tca6424_write_data(0x04, (output<<24)>>24);
    tca6424_write_data(0x05, (output<<16)>>24);
    tca6424_write_data(0x06, (output<<8)>>24);
    return 1;
}

uint8_t tca6424_init(void)
{
    tca6424_write_data(0x0C, 0x00);	// Port 0 output
    tca6424_write_data(0x0D, 0x00);	// Port 1 output
    tca6424_write_data(0x0E, 0x00);	// Port 2 output
    tca6424_output(0);
    return 1;
}

void LED_on(uint32_t led)
{
	LEDs=LEDs|led;
}

void LED_off(uint32_t led)
{
	LEDs=LEDs&~led;
}

uint8_t tca6424_write_data(uint8_t reg_addr, uint8_t data)
{
    uint8_t status, ret_val = 0;

    static const gpio_map_t TWI_GPIO_MAP =
    {
      {TCA6424_TWI_SDA_PIN, TCA6424_TWI_SDA_FUNCTION},
      {TCA6424_TWI_SCL_PIN, TCA6424_TWI_SCL_FUNCTION}
    };
    twi_options_t opt;
    twi_package_t packet;

    // TWI gpio pins configuration
    gpio_enable_module(TWI_GPIO_MAP, sizeof(TWI_GPIO_MAP) / sizeof(TWI_GPIO_MAP[0]));

    // options settings
    opt.pba_hz = FOSC0;
    opt.speed = TCA6424_SPEED;
    opt.chip = TCA6424_ADDRESS;

    // initialize TWI driver with options
    status = twi_master_init(&AVR32_TWI, &opt);
    // check init result
    if (status == TWI_SUCCESS)
    {
        ret_val = 1;
    }
    else
    {
        ret_val = 0;
    }

    // TWI chip address to communicate with
    packet.chip = TCA6424_ADDRESS;
    // TWI address/commands to issue to the other chip (node)
    packet.addr = reg_addr;
    // Length of the TWI data address segment (1-3 bytes)
    packet.addr_length = 1;
    // Where to find the data to be written
    packet.buffer = (void*)(&data);
    // How many bytes do we want to write
    packet.length =1;

    // perform a write access
    status = twi_master_write(&AVR32_TWI, &packet);

    // check write result
    if (status == TWI_SUCCESS)
    {
        ret_val = 1;
    }
    else
    {
        ret_val = 0;
    }

    return ret_val;
}
