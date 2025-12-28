/**\file *******************************************************************************************
 * 
 * \brief PM configuration protocol.
 *
 * \details Functional description:
 *
 * <b>Target Platforms:</b> ATmega324P
 *
 * <b>Editor:</b>           Atmel AVR Studio 5 (Version: 5.1.208)
 *
 * <b>Compiler:</b>			AVRGCC (Version: 3.3.1.27) 
 * 
 * \date 2013-09-10
 * \version WEPO41104A02
 * \author created by flm, mih, job, ank
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 *
 **************************************************************************************************/

#ifndef PM_CONFIG_PROTOCOL_H_
#define PM_CONFIG_PROTOCOL_H_

#include <avr/io.h>
#include <avr/wdt.h>
#include "at86.h"
#include "at86rf231.h"
#include "SPIKE_config.h"
#include "eeprom_abbild.h"
#include "ds620.h"
#include "SX8723c.h"
#include "conf_sw_twi.h"
#include "conf_LED.h"

extern volatile at86rf231_frame at86rf231_rx_frame;
extern volatile at86rf231_frame at86rf231_tx_frame;
extern uint32_t timeout_timer;
extern volatile uint8_t use_acceleration_sensor_flag;
extern volatile bool stop_stream;

extern eeprom_data eeprom_sram_data;
extern eeprom_data *ptr_eeprom_sram_data;

void pm_config_protocol(void);

#endif /* PM_CONFIG_PROTOCOL_H_ */