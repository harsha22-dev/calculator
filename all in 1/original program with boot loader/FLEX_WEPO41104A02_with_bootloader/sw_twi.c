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

#include "inc/sw_twi.h"

void sw_TWI_write_single_register(uint8_t mask, uint8_t reg, uint8_t data)
{
#ifdef SW_TWI_BLOCK_INTERRUPT	// see conf_sw_twi.h
	cli();
#endif
	
	// bus idle
	SET_SDA(mask);
	SET_SCL;

	// start
	CLEAR_SDA(mask);
	CLEAR_SCL;

	// Address
	SET_SDA(mask); // Bit 7 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 6 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 5 = "0"
	SET_SCL;
	CLEAR_SCL;
	SET_SDA(mask); // Bit 4 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 3 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 2 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 1 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 0 = "0"	write register
	SET_SCL;
	CLEAR_SCL;

	// Acknowledge
	SET_SDA(mask);	// SDA open Collector
	SET_SCL;	// Acknowledge	
	CLEAR_SCL_SHORT;
	nop_2();
	
	// Register
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop_3();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		nop_2();
		SET_SCL;
	}		
			
	CLEAR_SCL_SHORT;
	
	// Acknowledge
	SET_SDA(mask);	// SDA open Collector
	nop_3();
	SET_SCL;	// Acknowledge	
	CLEAR_SCL_SHORT;
	nop_2();

	// Value
	if ((data & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	data = data << 1;
	CLEAR_SCL_SHORT;
	
	if ((data & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	data = data << 1;
	CLEAR_SCL_SHORT;
	
	if ((data & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	data = data << 1;
	CLEAR_SCL_SHORT;
	
	if ((data & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	data = data << 1;
	CLEAR_SCL_SHORT;
	
	if ((data & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	data = data << 1;
	CLEAR_SCL_SHORT;
	
	if ((data & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	data = data << 1;
	CLEAR_SCL_SHORT;
	
	if ((data & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	data = data << 1;
	CLEAR_SCL_SHORT;
	
	if ((data & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop_3();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		nop_2();
		SET_SCL;
	}		
			
	CLEAR_SCL_SHORT;
	
	// Acknowledge
	SET_SDA(mask);	// SDA open Collector
	nop_3();
	SET_SCL;	// Acknowledge	
	CLEAR_SCL;

	// stop
	CLEAR_SDA(mask);	
	SET_SCL;
	SET_SDA(mask);
	
#ifdef SW_TWI_BLOCK_INTERRUPT	// see conf_sw_twi.h
	sei();
#endif
}

void sw_TWI_read_single_register(uint8_t mask, uint8_t reg, uint8_t* data)
{
	uint8_t x = 0;
	uint8_t received_data[8];
	uint8_t help = 0;
	
#ifdef SW_TWI_BLOCK_INTERRUPT	// see conf_sw_twi.h
	cli();
#endif
	
	// bus idle
	SET_SDA(mask);
	SET_SCL;

	// start
	CLEAR_SDA(mask);
	CLEAR_SCL;

	// Address
	SET_SDA(mask); // Bit 7 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 6 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 5 = "0"
	SET_SCL;
	CLEAR_SCL;
	SET_SDA(mask); // Bit 4 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 3 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 2 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 1 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 0 = "0"	write register
	SET_SCL;
	CLEAR_SCL;

	// Acknowledge
	SET_SDA(mask);	// SDA open Collector
	SET_SCL;	// Acknowledge	
	CLEAR_SCL_SHORT;
	nop_2();
	
	// Register
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		SET_SCL;
	}		
				
	reg = reg << 1;
	CLEAR_SCL_SHORT;
	
	if ((reg & 0b10000000) == 128)
	{
		SET_SDA_SHORT(mask);		// Bit x = "1"
		nop_3();
		SET_SCL_SHORT;
		nop_2();
	}				
	else
	{
		CLEAR_SDA_SHORT(mask);	// Bit x = "0"
		nop_2();
		SET_SCL;
	}		
			
	CLEAR_SCL_SHORT;
	
	// Acknowledge
	SET_SDA(mask);	// SDA open Collector
	nop_3();
	SET_SCL;	// Acknowledge	
	CLEAR_SCL_SHORT;
	nop_3();
	
	// Restart
	SET_SDA(mask);
	SET_SCL_SHORT;
	CLEAR_SDA_SHORT(mask);
	CLEAR_SCL_SHORT;
	nop_3();
	
	// Address
	SET_SDA(mask); // Bit 7 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 6 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 5 = "0"
	SET_SCL;
	CLEAR_SCL;
	SET_SDA(mask); // Bit 4 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 3 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 2 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 1 = "0"
	SET_SCL;
	CLEAR_SCL;
	SET_SDA(mask); // Bit 0 = "1"	read register
	SET_SCL;
	CLEAR_SCL;

	// Acknowledge
	SET_SDA(mask);	// SDA open Collector
	SET_SCL;	// Acknowledge	
	CLEAR_SCL;
	nop_8();
	
	// Register value
	SET_SCL_SHORT;
	received_data[7] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[6] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[5] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[4] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[3] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[2] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[1] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[0] = (SDA_PIN & mask);
	CLEAR_SCL_SHORT;
	
	// Not-Acknowledge 
	SET_SDA(mask);	// SDA open Collector
	nop_3();
	SET_SCL;	// Not-Acknowledge	
	CLEAR_SCL;
	
	// stop
	CLEAR_SDA(mask);	
	SET_SCL;
	SET_SDA(mask);
	
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	
	// transform data
	for( x = 8; x > 0; x-- )
	{
		data[0] = ( data[0] << 1 );
		data[1] = ( data[1] << 1 );
		data[2] = ( data[2] << 1 );
		data[3] = ( data[3] << 1 );
		
		help = ( received_data[x-1] & 0b00000001 );
		help = ( help >> 0 );
		data[3] = ( data [3] | help );
		
		help = ( received_data[x-1] & 0b00000010 );
		help = ( help >> 1 );
		data[2] = ( data[2] | help );
		
		help = ( received_data[x-1] & 0b00000100 );
		help = ( help >> 2 );
		data[1] = ( data [1] | help );
		
		help = ( received_data[x-1] & 0b00001000 );
		help = ( help >> 3 );
		data[0] = ( data[0] | help );
	}

#ifdef SW_TWI_BLOCK_INTERRUPT	// see conf_sw_twi.h
	sei();
#endif
}

void sw_TWI_read_value_registers(uint8_t mask, int16_t* data)
{
	uint8_t x = 0;
	uint8_t received_data[16];
	uint8_t help = 0;
	
#ifdef SW_TWI_BLOCK_INTERRUPT	// see conf_sw_twi.h
	cli();
#endif
	
	// bus idle
	SET_SDA(mask);
	SET_SCL;

	// start
	CLEAR_SDA(mask);
	CLEAR_SCL;

	// Address
	SET_SDA(mask); // Bit 7 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 6 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 5 = "0"
	SET_SCL;
	CLEAR_SCL;
	SET_SDA(mask); // Bit 4 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 3 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 2 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 1 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 0 = "0"	write register
	SET_SCL;
	CLEAR_SCL;

	// Acknowledge
	SET_SDA(mask);	// SDA open Collector
	SET_SCL;	// Acknowledge	
	CLEAR_SCL_SHORT;
	nop_3();
	
	// Register (0x50)
	CLEAR_SDA(mask); // Bit 7 = "0"
	SET_SCL;
	CLEAR_SCL;
	SET_SDA(mask); // Bit 6 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 5 = "0"
	SET_SCL;
	CLEAR_SCL;
	SET_SDA(mask); // Bit 4 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 3 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 2 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 1 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 0 = "0"	
	SET_SCL;
	CLEAR_SCL;		
	
	// Acknowledge
	SET_SDA(mask);	// SDA open Collector
	SET_SCL;	// Acknowledge	
	CLEAR_SCL_SHORT;
	nop_3();
	
	// Restart
	SET_SDA(mask);
	SET_SCL_SHORT;
	CLEAR_SDA_SHORT(mask);
	CLEAR_SCL_SHORT;
	nop_3();
	
	// Address
	SET_SDA(mask); // Bit 7 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 6 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 5 = "0"
	SET_SCL;
	CLEAR_SCL;
	SET_SDA(mask); // Bit 4 = "1"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 3 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 2 = "0"
	SET_SCL;
	CLEAR_SCL;
	CLEAR_SDA(mask); // Bit 1 = "0"
	SET_SCL;
	CLEAR_SCL;
	SET_SDA(mask); // Bit 0 = "1"	read register
	SET_SCL;
	CLEAR_SCL;

	// Acknowledge
	SET_SDA(mask);	// SDA open Collector
	SET_SCL;	// Acknowledge	
	CLEAR_SCL;
	nop_8();
	
	// Register value LSB
	SET_SCL_SHORT;
	received_data[7] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[6] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[5] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[4] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[3] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[2] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[1] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[0] = (SDA_PIN & mask);
	CLEAR_SCL;

	// Acknowledge
	CLEAR_SDA(mask);
	SET_SCL;	// Acknowledge	
	CLEAR_SCL;
	SET_SDA(mask);
	//nop_8();
	
	// Register value MSB
	SET_SCL_SHORT;
	received_data[15] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[14] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[13] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[12] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[11] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[10] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[9] = (SDA_PIN & mask);
	CLEAR_SCL;
	nop_8();
	
	SET_SCL_SHORT;
	received_data[8] = (SDA_PIN & mask);
	CLEAR_SCL_SHORT;
	
	// Not-Acknowledge 
	SET_SDA(mask);	// SDA open Collector
	nop_3();
	SET_SCL;	// Not-Acknowledge	
	CLEAR_SCL;
	
	// stop
	CLEAR_SDA(mask);	
	SET_SCL;
	SET_SDA(mask);
	
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	
	// transform data
	for( x = 16; x > 0; x-- )
	{
		data[0] = ( data[0] << 1 );
		data[1] = ( data[1] << 1 );
		data[2] = ( data[2] << 1 );
		data[3] = ( data[3] << 1 );
		
		help = ( received_data[x-1] & 0b00000001 );
		help = ( help >> 0 );
		data[0] = ( data [0] | help );
		
		help = ( received_data[x-1] & 0b00000010 );
		help = ( help >> 1 );
		data[1] = ( data[1] | help );
		
		help = ( received_data[x-1] & 0b00000100 );
		help = ( help >> 2 );
		data[2] = ( data [2] | help );
		
		help = ( received_data[x-1] & 0b00001000 );
		help = ( help >> 3 );
		data[3] = ( data[3] | help );
	}
	
	// divided by 2 because of 15 bit resolution
	data[0] = data[0] >> 1;
	data[1] = data[1] >> 1;
	data[2] = data[2] >> 1;
	data[3] = data[3] >> 1;
	
#ifdef SW_TWI_BLOCK_INTERRUPT	// see conf_sw_twi.h
	sei();
#endif
}