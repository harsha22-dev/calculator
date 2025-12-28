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

#include "inc/ds620.h"

void ds620_twi_init(void)
{
	DDRC &= ~(1 << PINC0);		// TWI SCL
	PORTC |= (1 << PINC0);		// TWI SCL
	
	DDRC &= ~(1 << PINC1);		// TWI SDA
	PORTC |= (1 << PINC1);		// TWI SDA

	TWBR=0x04;		// TWI Bit Rate Register
	TWCR=0x00;		// TWI Control Register
	TWSR=0xF8;		// TWI Status Register (1 = value of the prescaler bits)
}

unsigned char ds620_twi_start( unsigned char addr )
{
	TWCR = ( 1 << TWINT ) | ( 1 << TWSTA ) | ( 1 << TWEN );
	while( !( TWCR & ( 1 << TWINT ) ) );
	TWDR = addr;
	TWCR = ( 1 << TWINT ) | ( 1 << TWEN );
	while( !( TWCR & ( 1 << TWINT ) ) );
	if( ( TWSR & 0xF8 ) != 0x40 )
	return ( 0 );
	return ( 1 );
}

void ds620_twi_send( unsigned char data )
{
	TWDR = data;
	TWCR = ( 1 << TWINT ) | ( 1 << TWEN );
	while( !( TWCR & ( 1 << TWINT ) ) );
}

uint8_t ds620_twi_read( uint8_t ack )
{
	if( ack )
	TWCR = ( 1 << TWINT ) | ( 1 << TWEA ) | ( 1 << TWEN );
	else
	TWCR = ( 1 << TWINT ) | ( 1 << TWEN );
	while( !( TWCR & ( 1 << TWINT ) ) );
	return ( TWDR );
}

void ds620_twi_stop( void )
{
	TWCR = ( 1 << TWINT ) | ( 1 << TWSTO ) | ( 1 << TWEN );
} 

void ds620_init( void )
{
	ds620_twi_init();						// TWI initialization
	
	ds620_twi_start( 0x90 | ( 0 << 1 ) );	// start TWI connection
	ds620_twi_send( 0xAC );					// Configuration Register MS Byte
	ds620_twi_send( 0x08);					// 12 Bit resolution, Continuous conversion mode
	ds620_twi_start( 0x90 | ( 0 << 1 ) );	// start TWI connection
	ds620_twi_send( 0x51 );					// start convert
	ds620_twi_stop();						// stop TWI connection
}

void ds620_stop_convert( void )
{
	ds620_twi_start( 0x90 | ( 0 << 1 ) );	// start TWI connection
	ds620_twi_send( 0x22 );					// stop convert
	ds620_twi_stop();						// stop TWI connection
}

void ds620_start_convert( void )
{
	ds620_twi_start( 0x90 | ( 0 << 1 ) );	// start TWI connection
	ds620_twi_send( 0x51 );					// start convert
	ds620_twi_stop();						// stop TWI connection
}

int16_t ds620_get_temperature( void )
{
	uint8_t value_LSB = 0;
	uint8_t value_MSB = 0;
	int16_t value_receive = 0;
	
	ds620_twi_start( 0x90 | ( 0 << 1 ) );				// TWI-Verbindung aufbauen
	ds620_twi_send( 0xAA );								// DS620 Adresse auf Temperaturwert
	ds620_twi_start( 0x91 | ( 0 << 1 ) );				// Temperaturwert auslesen starten
	value_MSB = ds620_twi_read( 1 );					// 1.Temperatur-Byte auslesen
	value_LSB = ds620_twi_read( 0 );					// 2.Temperatur-Byte auslesen und
	value_receive = (int16_t) ((value_MSB << 8) | value_LSB);
	ds620_twi_stop();									// TWI-Verbindung beenden
	return value_receive;
}
