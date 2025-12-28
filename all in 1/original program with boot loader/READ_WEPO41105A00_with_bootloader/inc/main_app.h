// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file main_app.h
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
 * \date 2010-03-18
 *
 * \version V1.00 Exp (experimental)
 * \author        Forian Merz, pro-micron
 *
 *
 *
 * <b>Copyright &copy;2009 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>
 //-----------------------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------------------------//
// CC2500 Includes
//------------------------------------------------------------------------------------------------//
#include "portmacro.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "pm.h"
#include "flashc.h"
#include "gpio.h"
#include "pm.h"
#include "compiler.h"
#include <avr32/io.h>
#include "pwm.h"
#include "gpio.h"
#include "flashc.h"
#include "delay.h"
#include "twi.h"
#include "spi.h"
#include "intc.h"
#include "eic.h"
#include "portmacro.h"
#include "pdca.h"
#include "semphr.h"
#include "serial.h"
#include "spi.h"
#include "gpio.h"
#include "cc2500_radio.h"
#include "board.h"
#include "stdint.h"
#include "usb.h"
#include "rtc.h"
#include <math.h>
#ifndef _CC2500_H_
#define _CC2500_H_

/* Define output method for terminal-data */
#define data_printf     usb_uart_printf
#define debug_printf    usb_uart_printf

/* Userpage Definitions */
#define USER_PAGE_END_ADDR            0x80800200

#define SAMPLES_PER_PACKET  7

//-----------------------------------------------------------------------------------------------//
// Typedefs
//-----------------------------------------------------------------------------------------------//

typedef struct
__attribute__((__packed__))
{
    volatile int16_t strain_gauge[SAMPLES_PER_PACKET][4];
    volatile uint16_t counter;
    volatile int16_t temperature;
    volatile uint8_t voltage;
}spike_payload;


typedef struct
__attribute__ ((__packed__))
{
    uint16_t sw_version;
    uint16_t hw_version;
    uint16_t rf231_src_address;
    uint16_t rf231_dest_address;
    uint16_t rf231_pan_id;
    uint8_t rf231_channel;
    uint8_t rf231_channel_page;
    uint8_t rf231_tx_pwr;
    uint16_t pwm_freq;
    uint32_t bootloader_config;
}flash_data;

extern flash_data *ptr_flash_data;

/*! \brief struct cc2500_data_struct
 *        cc2500_data_struct - received information from cc2500
 */
typedef struct
__attribute__((__packed__))
{
    uint32_t error_crc_cnt;
    uint32_t rec_total;
    uint8_t streaming;
    uint8_t crc_error;
    uint32_t timer50m;
    uint8_t set_dac_manual;
    uint32_t frames_missing;
    uint8_t moving_average_size;
}app_control_struct;

/*! \brief struct cc2500_data_struct
 *        cc2500_data_struct - received information from cc2500
 */
typedef struct
{
    int16_t dms_array[7][4];
    int16_t offsets[4];
    int16_t rssi_dbm;
    uint16_t counter;
    uint8_t lqi;
    uint8_t voltage;
}cock_data;

/*! \brief SIS configuration data
 */
typedef struct
{
    uint8_t     cfg_byte;
    uint8_t     channel;
    uint16_t    zmd_offset_1;
    uint16_t    zmd_zero_shift_1;
    uint16_t    zmd_offset_2;
    uint16_t    zmd_zero_shift_2;
    uint8_t     sec_byte;
    uint32_t    serial_number;
    uint8_t     version;
}sis_cfg;

//-----------------------------------------------------------------------------------------------//
// Prototypes
//-----------------------------------------------------------------------------------------------//

/*! \brief function sis_application_init
 *        sis_application_init - create Application Tasks
 *
 *  \param pvParameters   Input. Not Used.
 *
 */
void cock_application_init( void );

/*! \brief TASK vTaskProcessData
 *        vTaskProcessData - application control
 *
 *  \param pvParameters   Input. Not Used.
 *
 */
void vTaskProcessData( void * pvParameters );

/*! \brief handle_config_packet
 *
 *  \param Not Used.
 *
 */
void handle_config_packet( void );

/*! \brief TASK vTaskProcessData
 *        vTaskProcessData - DAC-Output
 *
 *  \param pvParameters   Input. Not Used.
 *
 */
void vTaskTIMING( void * pvParameters );

/*! \brief TASK vTaskProcessData
 *        vTaskProcessData - DAC-Output
 *
 *  \param pvParameters   Input. Not Used.
 *
 */
void vTaskCONTROL( void * pvParameters );

/*! \brief sis_application_init
 *         create cc2500 task
 *
 */
void sis_application_init( void );

/*!
 * \brief pdca_int_handler_ISR_NonNakedBehaviour; pdca-DSR;
 */
long pdca_int_handler_ISR_NonNakedBehaviour( void );

/*!
 * \brief init_eic: Initializes EIC; INT0 = ext.Int CC2500
 */
void init_eic( void );

/*!
 * \brief eic_int_handler1 ISR; Interrupt handler of the External
 *                              interrupt line "1" ==> rx CC2500.
 */
void eic_int_handler1( void );

/*!
 * \brief look if valid data was received and convert it for output ;
 */
void cc2500_parse_data( void );

/*!
 * \brief pwm_init: init and start pwm_init
 */
void pwm_drv_init( unsigned int frequency );

/*!
 * \brief pwm_enable: start pwm channels
 */
void pwm_enable( void );

/*!
 * \brief pwm_disable: stop pwm channels
 */
void pwm_disable( void );

/*!
 * \brief set_dac: write value to dac ( by use of IIC )
 */
void set_dac( unsigned int data );

/*!
 * \brief cc2500_remote_config function
 */
void cc2500_remote_config( void );

/*!
 * \brief zero_strain_gauges function
 */
uint16_t zero_strain_gauges( void );

/*!
 * \brief get_sis_config
 */
uint8_t get_sensor_config( void );

/*!
 * \brief output_mode1
 */
void output_mode1(void);

/*!
 * \brief output_mode2
 */
void output_mode2(void);

/*!
 * \brief output_mode3
 */
void output_mode3(void);

/*!
 * \brief output_mode3
 */
void output_mode4(void);

/*!
 * \brief eeprom_save_config
 */
void eeprom_save_config( void );

/*!
 * \brief eeprom_load_config
 */
void eeprom_load_config( void );

#endif
