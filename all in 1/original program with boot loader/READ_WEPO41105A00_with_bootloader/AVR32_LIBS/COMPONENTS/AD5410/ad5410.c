// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file ad5410.c
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
 * \todo: - check spi access for errors
 * 		  - check if function parameters are valid
 *
 * \version
 *		2009-08-11			flm		- file generated, functions not reentrant save
 *		2009-08-12			flm		- using SPI0 and SPI1 together can result in deadlock.
 *							 		  Added second mutex in spi.c for SPI1
 *		2009-08-12			flm		- ad5410_set_current(), ad5410_reset(), ad5410_write()
 *									  are now reentrant save
 *
 * <b>Copyright &copy;2008 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>
//-----------------------------------------------------------------------------------------------//                                                                                               */

/* Header Files */
#include "ad5410.h"

/* Init AD5410 */
uint8_t ad5410_init( void )
{
    uint16_t temp;

	/* AD5410 options */
	ad5410_config_struct ad5410_config =
	{
		0x00,			// reserved, 2 Bit
		TRUE, 			// REXT: select external current setting resistor
		TRUE, 			// OUTEN: Output enable
		0x00,			// SR clock: slew rate clock, 4 Bit
		0x00,			// SR step: slew rate step, 3 Bit
		FALSE, 			// SREN: Digital slew rate control enable
		FALSE, 			// CEN: Daisy Chain enable
		RANGE_4_20mA  	// R2-R0 Output range select
	};

	/* spi options */
	spi_options_t spiOptions =
	{
		AD5410_SPI_NPCS,  	// chip select
		6500000,			// baudrate
		8,					// bits
		0,					// spck_delay
		0,					// trans_delay
		1,					// stay_act
		0,					// spi_mode
		1					// modfdis
	};

	/* set spi hardware pins (spi1) */
	static const gpio_map_t AD5410_SPI_GPIO_MAP =
	{
		{AD5410_SPI_SCK_PIN,  AD5410_SPI_SCK_FUNCTION },  // SPI Clock.
		{AD5410_SPI_MISO_PIN, AD5410_SPI_MISO_FUNCTION},  // MISO.
		{AD5410_SPI_MOSI_PIN, AD5410_SPI_MOSI_FUNCTION},  // MOSI.
		{AD5410_SPI_NPCS0_PIN, AD5410_SPI_NPCS0_FUNCTION} // Chip Select NPCS0.
	};

	/* reset spi1 */
	spi_reset( AD5410_SPI );

	/* Assign I/Os to SPI */
	gpio_enable_module( AD5410_SPI_GPIO_MAP, sizeof( AD5410_SPI_GPIO_MAP ) / sizeof( AD5410_SPI_GPIO_MAP[0] ) );

	/* Initialize as master */
	spi_initMaster( AD5410_SPI, &spiOptions );

	/* Set selection mode: variable_ps, pcs_decode, delay */
	spi_selectionMode( AD5410_SPI, 0, 0, 0 );

	/* Enable SPI */
	spi_enable( AD5410_SPI );

	/* setup chip registers */
	spi_setupChipReg( AD5410_SPI, &spiOptions, 66000000UL );

	/* write configuration register */
	ad5410_write( AD5410_ADDR_CR, *(uint16_t *)&ad5410_config );

	if( ad5410_read( AD5410_ADDR_CR ) != *(uint16_t *)&ad5410_config )

	/* read back register content */
	ad5410_read_control_reg((uint16_t*)&temp);

	if(temp != *(uint16_t *)&ad5410_config )
	{
	    return 0;
	}
	else
	{
	    return 1;
	}
}

/* Write data to AD5410 */
uint8_t ad5410_write(uint8_t addr, uint16_t data)
{
	/* AD5410 chip select*/
	spi_selectChip( AD5410_SPI, AD5410_SPI_NPCS );

	/* write address byte */
	spi_write( AD5410_SPI, addr );

	/* write highbyte */
	spi_write( AD5410_SPI, ( data >> 8 ) & 0x00FF );

	/* write lowbyte */
	spi_write( AD5410_SPI, data & 0x00FF );

	/* AD5410 chip unselect */
	spi_unselectChip( AD5410_SPI, AD5410_SPI_NPCS );

	return 1;
}

/* Read data from AD5410 */
uint16_t ad5410_read(uint8_t addr)
{
	uint16_t temp = 0;
	uint16_t data = 0;

	/* AD5410 chip select*/
	spi_selectChip( AD5410_SPI, AD5410_SPI_NPCS );

	/* write read command + req. address */
	spi_write( AD5410_SPI, AD5410_ADDR_READ );
	spi_write( AD5410_SPI, 0x00 );
	spi_write( AD5410_SPI, addr );

	/* AD5410 chip unselect */
	spi_unselectChip( AD5410_SPI, AD5410_SPI_NPCS );

	/* AD5410 chip select, read 3 bytes */
	spi_selectChip( AD5410_SPI, AD5410_SPI_NPCS );

	/* Read out Byte 1 */
	spi_write( AD5410_SPI, AD5410_NOP );

	/* Read out Byte 2, payload 1 */
	spi_write( AD5410_SPI, AD5410_NOP );
	spi_read( AD5410_SPI, &temp );
	data = (uint16_t)temp << 8;

	/* Read out Byte 3, payload 2 */
	spi_write( AD5410_SPI, AD5410_NOP );
	spi_read( AD5410_SPI, &temp );
	data |= (uint16_t)temp;

	/* AD5410 chip unselect */
	spi_unselectChip( AD5410_SPI, AD5410_SPI_NPCS );

	return data;
}

/* SET AD5410 Output current */
uint8_t ad5410_set_current(uint16_t current)
{
    uint16_t temp;

    /* write data value to AD5410 if <= 4095 */
	if( current <= 4095 )
	{
	    ad5410_write( AD5410_ADDR_DATA, current << 4);
	}

    /* read back register content */
	ad5410_read_data_reg((uint16_t*)&temp);

    if(temp != current )
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/* Reset AD5410 */
uint8_t ad5410_reset( void )
{
	if( ad5410_write( AD5410_ADDR_RST, 0x0001 ) )
		return 1;
	else
		return 0;
}

/* Read the AD5410 status register */
uint8_t ad5410_read_status_reg( uint16_t * status )
{
	//uint16_t status = 0;

	/* read 16 status bits */
	*status = ad5410_read( 0x00 );

	if( *status & 0x0001 )
	{
		// Overtemp
	}

	if( *status & 0x0002 )
	{
		// Slew active
	}

	if( *status & 0x0004 )
	{
		// I_OUT fault
	}

	return 1;
}

/* Read the AD5410 data register */
uint8_t ad5410_read_data_reg( uint16_t * data )
{
	/* read data register */
	*data = ad5410_read( 0x01 ) >> 4;

	return 1;
}

/* Read the AD5410 control register */
uint8_t ad5410_read_control_reg( uint16_t * data )
{
	/* read control register */
	*data = ad5410_read( 0x02 );

	return 1;
}
