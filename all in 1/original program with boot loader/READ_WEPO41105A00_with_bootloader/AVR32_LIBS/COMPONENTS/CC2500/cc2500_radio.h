// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file cc2500_radio.h
 *
 * \brief Receiving and processing data from CC2500
 *
 * \details Functional description: Receiving and processing data from CC2500
 *
 * <b>Target Platforms:</b> AVR32UC3A1256
 *
 * <b>Editor:</b> Eclipse
 *
 * <b>Compiler:</b> GNU Compiler Collection (GCC) 4.1.4
 *
 * \date 2009-10-15
 *
 * \version V1.00 Exp (experimental)
 * \author        Forian Merz, pro-micron
 *
 *
 *
 * <b>Copyright &copy;2009 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>
 //-----------------------------------------------------------------------------------------------*/
#include "stdint.h"
#include "board.h"
#include "delay.h"
#include "gpio.h"
#include "gpio.h"
#include "spi.h"
#include "portmacro.h"
#include "pdca.h"
#include "spi.h"
#include "gpio.h"

/* Definition of CC2500_DATA_LENTH */
#define CC2500_DATA_LENGTH            59

/* PDCA-SPI definitions */
#define SPI_DMA_RX_CHAN 0
#define SPI_DMA_TX_CHAN 1

/* CC2500 command definitions */
#define SRES    cc2500_send_command(0x30);
#define SFSTXON cc2500_send_command(0x31);
#define SXOFF   cc2500_send_command(0x32);
#define SCAL    cc2500_send_command(0x33);
#define SRX     cc2500_send_command(0x34);
#define STX     cc2500_send_command(0x35);
#define SIDLE   cc2500_send_command(0x36);
#define SAFC    cc2500_send_command(0x37);
#define SWOR    cc2500_send_command(0x38);
#define SPWD    cc2500_send_command(0x39);
#define SFRX    cc2500_send_command(0x3A);
#define SFTX    cc2500_send_command(0x3B);
#define SWORRST cc2500_send_command(0x3C);
#define SNOP    cc2500_send_command(0x3D);
#define IDLE    0;
#define TX      1;
#define RX      2;

/*!
* \brief cc2500_init_chip: Initializes cc2500-chip and setup spi0
 *
 * \return: status: 1=ok   0=error
 *
 */
uint8_t cc2500_init_chip( void );

/*!
 * \brief cc2500_send_command: write command on cc2500
 *
 * \param pvParameters[in] nummer: Channel number
 *
 */
unsigned char cc2500_send_command( unsigned char nummer );

/*!
 * \brief cc2500_write_byte: write data at specified address on cc2500
 */
unsigned char cc2500_write_byte( unsigned short adr, unsigned short data );

/*!
 * \brief cc2500_write_burst: write data burst at specified address on cc2500
 */
unsigned char cc2500_write_burst( unsigned char adr, unsigned char *data, unsigned char size );

/*!
 * \brief cc2500_send_data: write data burst on cc2500
 */
unsigned char cc2500_send_data( unsigned char *tx, unsigned char len );

/*!
 * \brief cc2500_read_byte: read data at specified address on cc2500
 */
unsigned short cc2500_read_byte( unsigned short adr );

/*!
 * \brief cc2500_read_burst: read data burst at specified address on cc2500
 */
unsigned short cc2500_read_burst( unsigned short adr );

/*!
 * \brief cc2500_read_dma start the dma transfer on ch. 3 and 4
 *        receives data from spi0
 */
void cc2500_read_dma( unsigned char addr, unsigned char len );

/*!
 * \brief function to select the CC2500
 *
 */
void cc2500_spi_select( void );

/*!
 * \brief function to unselect the CC2500
 *
 */
void cc2500_spi_unselect( void );

/*!
 * \brief cc2500_write_settings: configuration data for cc2500
 * \return status: 1=ok   0=error
 */
uint8_t cc2500_write_settings( void );

/*!
 * \brief function to change the CC2500 channel number
 *
 * \param number                new channel. 0-15
 *
 */
unsigned char cc2500_change_channel( unsigned char number );

/* !
 * \brief function to change the CC2500 Packet length.
 *
 * \param number                length.
 *
 */
unsigned char cc2500_change_data_length( unsigned char length );

/*!
 * \brief init_cc2500_pdca: Initializes pdca ch3 and ch4 for SPI0
 */
void init_cc2500_pdca( void );

/*!
 * \brief pdca_int_handler ISR; pdca-isr; triggers if data was succesfully received on pdca ch 3.
 */
void pdca_int_handler( void );

/*!
 * \brief wait_us function
 *
 * \param usec number 0 - 2^32 in µs
 *
 */
void wait_us( unsigned int usec );
