// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file usb.h
 *
 * \brief receiving and sending data from/to CDC
 *
 * <b>Target Platforms:</b>	AVR32UC3A1256
 *
 * <b>Editor:</b> Eclipse
 *
 * <b>Compiler:</b> GNU Compiler Collection (GCC) 4.1.4
 *
 * \date 2009-04-03
 *
 * \version V1.00 Exp (experimental)
 * \author 	 				- Florian Merz, pro-micron
 *
 * \bug	...
 * \todo ...
 * \version
 *
 * 		2009-01-12		FM		- Erste Version: - USB-CDC-Task gibt Daten aus
 * 												 - USB-CDC-Task liest Daten ein
 *
 *		2009-02-04	    FM		- Eingabemöglichkeit für set_dac() set_pwm()
 *
 * <b>Copyright &copy;2008 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>*/
//-----------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------//
// Prototypes
//-----------------------------------------------------------------------------------------------//
#include <inttypes.h>

/*! \brief app_init
 *         USB-Tasks erstellen
 *
 */
void usb_start( void );

/*! \brief app_init
 *
 *         Implementierung der printf-Funktin für CDC
 */
int usb_uart_printf( const char *fmt, ... );

/*! \brief app_init
 *
 *         Implementierung der printf-Funktin ohne overhead für CDC
 */
int usb_uart_printf_fast( const char *fmt, ... );

/*
 *\brief usb_uart_get_string
 * 		 read string vom cdc-uart
 */
void usb_uart_get_string( char* buffer, int buffer_len );

/*!
 * \brief Entry point of the device CDC task management ;
 */
void vTaskUSB( void * pvParameters );

/*!
 * \brief write a string to cdc ;
 */
void usb_uart_write_line( const char *string, unsigned char enable_fast );

void usb_cdc_performance_print( int32_t *data, const uint16_t len );


uint32_t usb_get_word(void);