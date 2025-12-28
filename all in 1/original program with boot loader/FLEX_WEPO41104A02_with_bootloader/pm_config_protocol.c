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

#include "inc/pm_config_protocol.h"
#include "inc/Bootloader.h"

void pm_config_protocol(void)
{
	volatile uint16_t temp = 0;
	volatile uint32_t config_timer = 0;
	tal_trx_status_t tal_trx_status = 0;
	volatile uint8_t cmd_end_flag = 0;

	// Deactivate Timer-Interrupt 
	TIMSK1 = 0;

	// Enable global Interrupts 
	sei();

	// Write data request command 
	//at86rf231_tx_frame.payload[0] = 0x04; // CMD

	// Send data request and wait for data 
	//at86rf231_req_data();

	// Wait for READ-welcome message 
	while (cmd_end_flag == 0)
	{
		// Did we receive a config frame? 
		if ((at86rf231_rx_frame.frame_control_field & 7)== FCF_FRAMETYPE_MAC_CMD)
		{
			// command received, reset timeout 
			config_timer = 0;

			// reset fcf 
			at86rf231_rx_frame.frame_control_field = 0x00;

			// Which command was received? 
			switch (at86rf231_rx_frame.payload[0])
			{

/*
#if defined(FULL_CONFIG_MODE)
			// CMD: REQ_STREAM 
			case 0x00:
			// no new command available 
			break;
#endif
*/
/*
#if defined(FULL_CONFIG_MODE)
			// CMD: REQ_CFG 
			case 0x04:
			// We are already in config mode, do nothing, wait for CMD´s 
			break;
#endif
*/
			// --- CMD: STOP_STREAM ---
			case 0x0F:

				// Write Payload in tx buffer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x0F; // CMD to ACK

				// Send data request and wait for data
				at86rf231_req_data();

				// set spike timeout
				stop_stream = 1;
				
				// leave config mode, start streaming
				cmd_end_flag = 1;
				break;

			// --- CMD: REQ_STREAM ---
			case 0x10:
			
				// Write Payload in tx buffer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x10; // CMD to ACK

				// leave config mode, start streaming
				cmd_end_flag = 1;
				break;
/*
#if defined(FULL_CONFIG_MODE)
				// CMD: READ_EEPROM 
				case 0x11:

				// Read desired memory address 
				temp = ( at86rf231_rx_frame.payload[1] << 8 ) + at86rf231_rx_frame.payload[2];
				temp = eeprom_read_byte( (uint8_t* ) temp);

				// Write Payload in tx buffer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x11; // CMD to ACK
				at86rf231_tx_frame.payload[2] = temp;

				// Send data request and wait for data 
				at86rf231_req_data();
				break;
#endif
*/
/*
#if defined(FULL_CONFIG_MODE)
				// CMD: WRITE_EEPROM 
				case 0x12:

				// write data into eeprom 
				temp = ( at86rf231_rx_frame.payload[1] << 8 ) + at86rf231_rx_frame.payload[2];
				eeprom_write_byte((uint8_t* ) temp, at86rf231_rx_frame.payload[3]);

				// Read written byte back 
				temp = eeprom_read_byte( (uint8_t* ) temp);

				if( temp == at86rf231_rx_frame.payload[3] )
				{
					// Write Payload in tx buffer 
					at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
					at86rf231_tx_frame.payload[1] = 0x12; // CMD to ACK
					at86rf231_tx_frame.payload[2] = temp; // Param2: read back byte
				}
				else
				{
					// Write Payload in tx buffer
					at86rf231_tx_frame.payload[0] = 0xFF; // CMD: ACK_REQ
					at86rf231_tx_frame.payload[1] = 0x12; // CMD to ACK
					at86rf231_tx_frame.payload[2] = temp; // Param2: read back byte
				}
				// Send data request and wait for data
				at86rf231_req_data();
				break;
#endif
*/
/*				
#if defined(FULL_CONFIG_MODE)
				// CMD: WRITE_ZMD_CFG 
				case 0x13:
				// Write Payload in tx buffer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x13; // CMD to ACK

				// copy received zmd config into zmd_config_array 
				for(temp = 0; temp <32; temp++)
				{
					// 4 zmds are available 
					if(at86rf231_rx_frame.payload[1] <= 3 )
					{
						eeprom_sram_data.zmd_config[at86rf231_rx_frame.payload[1]][temp] = ( at86rf231_tx_frame.payload[temp*2+5] << 8 ) + at86rf231_tx_frame.payload[temp*2+6];
					}

				}
				// store data into eeprom
				eeprom_save_config();

				// Send data request and wait for data 
				at86rf231_req_data();
				break;
#endif
*/
/*
#if defined(FULL_CONFIG_MODE)
				// CMD: READ_SX8723C_CFG
				case 0x14:
				// Write Payload in tx buffer
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x14; // CMD to ACK
				
				if (at86rf231_rx_frame.payload[1] >= 4)
				{									
					// Write Payload in tx buffer
					at86rf231_tx_frame.payload[0] = 0xFF; // CMD: ERROR_MSG
					at86rf231_tx_frame.payload[1] = 0x14; // CMD to ACK
				}
				else
				{
					at86rf231_tx_frame.payload[2] = at86rf231_rx_frame.payload[1]; // Number of SX8723c

					// copy requested SX8723c config into payload 
					for(temp = 0; temp <11; temp++)
					{
						at86rf231_tx_frame.payload[temp+3] = (uint8_t)eeprom_sram_data.SX8723c_config[at86rf231_rx_frame.payload[1]][temp];
					}
				}

				// Send data request and wait for data
				at86rf231_req_data();
				
				// leave config mode 
				cmd_end_flag = 1;
				
				break;
#endif
*/
			// --- CMD: READ_VERSION ---
			case 0x15:
				// Write Payload in tx buffer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x15; // CMD to ACK
				at86rf231_tx_frame.payload[2] = ptr_eeprom_sram_data->hw_version; // Param1: HwNum
				at86rf231_tx_frame.payload[3] = ptr_eeprom_sram_data->hw_version >> 8; // Param1: HwNum

				at86rf231_tx_frame.payload[4] = ptr_eeprom_sram_data->sw_version; // Param2: VersionNum
				at86rf231_tx_frame.payload[5] = ptr_eeprom_sram_data->sw_version >> 8; // Param2: VersionNum

				at86rf231_tx_frame.payload[6] = ptr_eeprom_sram_data->serial_number; // Param3: serial number
				at86rf231_tx_frame.payload[7] = ptr_eeprom_sram_data->serial_number >> 8; // Param3: serial number

				// Send data request and wait for data 
				at86rf231_req_data();

				// leave config mode 
				cmd_end_flag = 1;

				break;

			// --- CMD: SET_TIMEOUT ---
			case 0x16:

				// timeout
				temp = (at86rf231_rx_frame.payload[1] << 8)
						+ at86rf231_rx_frame.payload[2];

				if (temp <= 300)
				{
					// store timeout into eeprom 
					eeprom_sram_data.timeout = temp;
					eeprom_save_config(ptr_eeprom_sram_data);

					// Write Payload in tx buffer
					at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
					at86rf231_tx_frame.payload[1] = 0x16; // CMD to ACK

					// if timeout = 0 --> deactivate accelerometer, SPIKE always on 
					if (eeprom_sram_data.timeout == 0)
					{
						use_acceleration_sensor_flag = 0;
					}
					else
					{
						use_acceleration_sensor_flag = 1;
						ACSR = 0x12;
					}

					// reset timeout
					timeout_timer = 0;
				}
				else
				{
					// Write Payload in tx buffer
					at86rf231_tx_frame.payload[0] = 0xFF; // CMD: ERROR_MSG
					at86rf231_tx_frame.payload[1] = 0x16; // CMD to ACK
				}

				// Send data request and wait for data 
				at86rf231_req_data();

				// leave config mode
				cmd_end_flag = 1;

				break;
/*
#if defined(FULL_CONFIG_MODE)
				// CMD: SET_DATARATE 
				case 0x17:

				// store datarate into eeprom 
				temp = ( at86rf231_rx_frame.payload[1] << 8 ) + at86rf231_rx_frame.payload[2];
				if(temp <= 3000 && temp >=350)
				{
					eeprom_sram_data.samplerate = temp;
					eeprom_save_config();

					// Init timer with new samplerate
					timer_init();

					// Write Payload in tx buffer
					at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
					at86rf231_tx_frame.payload[1] = 0x17; // CMD to ACK
				}
				else
				{
					// Write Payload in tx buffer
					at86rf231_tx_frame.payload[0] = 0xFF; // CMD: ERROR_MSG
					at86rf231_tx_frame.payload[1] = 0x17; // CMD to ACK
				}

				// Send data request and wait for data 
				at86rf231_req_data();
				break;
#endif
*/
			// --- CMD: C_RADIO_CH ---
			case 0x20:
				// Set Transceiver State = TRX_OFF 
				pal_trx_reg_write(RG_TRX_STATE, CMD_TRX_OFF);

				// verify that state = TRX_OFF 
				while (tal_trx_status != TRX_OFF)
				{
					tal_trx_status = (tal_trx_status_t) pal_trx_bit_read(
							SR_TRX_STATUS);
				}

				// write new channel 
				temp = at86rf231_change_channel(at86rf231_rx_frame.payload[1]);

				if (temp == at86rf231_rx_frame.payload[1])
				{
					// store channel into eeprom 
					eeprom_sram_data.channel = at86rf231_rx_frame.payload[1];
					eeprom_save_config(ptr_eeprom_sram_data);

					// Write Payload in tx buffer
					at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
					at86rf231_tx_frame.payload[1] = 0x20; // CMD to ACK
					at86rf231_tx_frame.payload[2] = (uint8_t) temp; // channel register
				}
				else
				{
					// Write Payload in tx buffer
					at86rf231_tx_frame.payload[0] = 0xFF; // CMD: ERROR
					at86rf231_tx_frame.payload[1] = 0x20; // CMD to ACK
					at86rf231_tx_frame.payload[2] = (uint8_t) temp; // return channel
				}

				// Send data request and wait for data
				at86rf231_req_data();

				// leave config mode
				cmd_end_flag = 1;

				break;
/*
#if defined(FULL_CONFIG_MODE)
				// CMD: C_DATARATE_MODE 
				case 0x21:

				// Set Transceiver State = TRX_OFF 
				pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

				// verify that state = TRX_OFF 
				while( tal_trx_status != TRX_OFF )
				{
					tal_trx_status = ( tal_trx_status_t )pal_trx_bit_read( SR_TRX_STATUS );
				}

				// write new channel page
				temp = at86rf231_change_channel_page( at86rf231_rx_frame.payload[1] );

				if( temp == 1)
				{
					// store into SRAM and than in EEPROM 
					eeprom_sram_data.channel_page = at86rf231_rx_frame.payload[1];
					eeprom_save_config();

					// Write Payload in tx buffer
					at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
					at86rf231_tx_frame.payload[1] = 0x21; // CMD to ACK
					at86rf231_tx_frame.payload[2] = (uint8_t)temp; // channel page success / failure
				}
				else
				{
					// Write Payload in tx buffer 
					at86rf231_tx_frame.payload[0] = 0xFF; // CMD: ACK_REQ
					at86rf231_tx_frame.payload[1] = 0x21; // CMD to ACK
					at86rf231_tx_frame.payload[2] = (uint8_t)temp; // channel page success / failure
				}

				// Send data request and wait for data 
				at86rf231_req_data();
				break;
#endif
*/
/*
#if defined(FULL_CONFIG_MODE)
				// CMD: NEW_SRC_ADDRESS
				case 0x22:
				// Set Transceiver State = TRX_OFF 
				pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

				// verify that state = TRX_OFF 
				while( tal_trx_status != TRX_OFF )
				{
					tal_trx_status = ( tal_trx_status_t )pal_trx_bit_read( SR_TRX_STATUS );
				}

				// read new address from received frame 
				temp = ( at86rf231_rx_frame.payload[1] << 8 ) + at86rf231_rx_frame.payload[2];

				// write into transceiver filter register 
				pal_trx_reg_write( RG_SHORT_ADDR_0, at86rf231_rx_frame.payload[1]);
				pal_trx_reg_write( RG_SHORT_ADDR_1, at86rf231_rx_frame.payload[2]);

				// store into SRAM and than in EEPROM 
				eeprom_sram_data.src_address = temp;
				eeprom_save_config();

				// Write Payload in tx bufer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x22; // CMD to ACK

				// Send data request and wait for data
				at86rf231_req_data();
				break;
#endif
*/
/*
#if defined(FULL_CONFIG_MODE)
				// CMD: NEW_DEST_ADDRESS
				case 0x23:
				// Set Transceiver State = TRX_OFF 
				pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

				// verify that state = TRX_OFF 
				while( tal_trx_status != TRX_OFF )
				{
					tal_trx_status = ( tal_trx_status_t )pal_trx_bit_read( SR_TRX_STATUS );
				}

				// read new address from received frame 
				temp = ( at86rf231_rx_frame.payload[1] << 8 ) + at86rf231_rx_frame.payload[2];

				// store into SRAM and than in EEPROM 
				eeprom_sram_data.dest_address = temp;
				eeprom_save_config();

				// Write Payload in tx buffer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x23; // CMD to ACK

				// Send data request and wait for data 
				at86rf231_req_data();
				break;
#endif
*/
/*
#if defined(FULL_CONFIG_MODE)
				// CMD: NEW_PANID 
				case 0x24:
				// Set Transceiver State = TRX_OFF 
				pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

				// verify that state = TRX_OFF 
				while( tal_trx_status != TRX_OFF )
				{
					tal_trx_status = ( tal_trx_status_t )pal_trx_bit_read( SR_TRX_STATUS );
				}

				// read new address from received frame 
				temp = ( at86rf231_rx_frame.payload[1] << 8 ) + at86rf231_rx_frame.payload[2];

				// write into transceiver filter register 
				pal_trx_reg_write( RG_PAN_ID_0, at86rf231_rx_frame.payload[1]);
				pal_trx_reg_write( RG_PAN_ID_1, at86rf231_rx_frame.payload[2]);

				// store into SRAM and than in EEPROM
				eeprom_sram_data.pan_id = temp;
				eeprom_save_config();

				// Write Payload in tx buffer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x24; // CMD to ACK

				// Send data request and wait for data 
				at86rf231_req_data();
				break;
#endif
*/
/*			
#if defined(FULL_CONFIG_MODE)
				// CMD: NEW_TX_PWR
				case 0x25:

				// write into transceiver power register 
				temp = at86rf231_change_tx_pwr( at86rf231_rx_frame.payload[1] );

				if( temp != 0xFF )
				{
					// store into SRAM and than in EEPROM 
					eeprom_sram_data.tx_pwr = at86rf231_rx_frame.payload[1];
					eeprom_save_config();

					// Write Payload in tx buffer 
					at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
					at86rf231_tx_frame.payload[1] = 0x25; // CMD to ACK
					at86rf231_tx_frame.payload[2] = (uint8_t)temp; // CMD to ACK
				}
				else
				{
					// Write Payload in tx buffer 
					at86rf231_tx_frame.payload[0] = 0xFF; // CMD: ACK_REQ
					at86rf231_tx_frame.payload[1] = 0x25; // CMD to ACK
					at86rf231_tx_frame.payload[2] = (uint8_t)temp; // CMD to ACK
				}

				// Send data request and wait for data 
				at86rf231_req_data();
				break;
#endif
*/
			// --- CMD: READ_TEMPERATURE ---
			case 0x30:
				
				#if defined(USE_DS620)
				temp = ds620_get_temperature();
				#endif
				
				// Write Payload in tx buffer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x30; // CMD to ACK
				at86rf231_tx_frame.payload[2] = (uint8_t) temp; // CMD to ACK
				at86rf231_tx_frame.payload[3] = (uint8_t)(temp >> 8); // CMD to ACK

				// Send data request and wait for data
				at86rf231_req_data();

				// leave config mode 
				cmd_end_flag = 1;
				
				break;

			// --- CMD: OFFSET ---
			case 0x31:
				
				LED_ON;		// LED on
				
				SX8723c_auto_offset(device_1);	// offset 
				SX8723c_auto_offset(device_2);	// offset
				SX8723c_auto_offset(device_3);	// offset
				SX8723c_auto_offset(device_4);	// offset
				
				// store data into eeprom 
				eeprom_save_config(ptr_eeprom_sram_data);
				
				LED_OFF;	// LED off
				
				// Write Payload in tx buffer
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x31; // CMD to ACK

				// Send data request and wait for data
				at86rf231_req_data();

				// leave config mode 
				cmd_end_flag = 1;
				
				break;

			// --- CMD: WRITE_CALIB_VALUES ---
			case 0x32:
				eeprom_sram_data.calibration_values[0]
						= *((uint32_t*) (at86rf231_rx_frame.payload + 3));
				eeprom_sram_data.calibration_values[1]
						= *((uint32_t*) (at86rf231_rx_frame.payload + 7));
				eeprom_sram_data.calibration_values[2]
						= *((uint32_t*) (at86rf231_rx_frame.payload + 11));
				eeprom_sram_data.calibration_values[3]
						= *((uint32_t*) (at86rf231_rx_frame.payload + 15));

				eeprom_sram_data.calibration_values[4]
						= *((uint32_t*) (at86rf231_rx_frame.payload + 19));
				eeprom_sram_data.calibration_values[5]
						= *((uint32_t*) (at86rf231_rx_frame.payload + 23));
				eeprom_sram_data.calibration_values[6]
						= *((uint32_t*) (at86rf231_rx_frame.payload + 27));
				eeprom_sram_data.calibration_values[7]
						= *((uint32_t*) (at86rf231_rx_frame.payload + 31));
						
				eeprom_save_config(ptr_eeprom_sram_data);

				// Write Payload in tx buffer 
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x32; // CMD to ACK

				// Send data request and wait for data 
				at86rf231_req_data();

				// leave config mode
				cmd_end_flag = 1;
				
				break;

			// --- CMD: READ_CALIB_VALUES ---
			case 0x33:

				// copy requested config into payload
				for (temp = 0; temp < 32; temp++)
				{
					at86rf231_tx_frame.payload[temp + 3]
							= *((uint8_t*) (eeprom_sram_data.calibration_values)
									+ temp);
				}

				// Write Payload in tx buffer
				at86rf231_tx_frame.payload[0] = 0x0B; // CMD: ACK_REQ
				at86rf231_tx_frame.payload[1] = 0x33; // CMD to ACK

				// Send data request and wait for data 
				at86rf231_req_data();

				// leave config mode 
				cmd_end_flag = 1;
				break;
			
			// ---CMD: FIRMWARE_UPDATE ---
			case 0x40:

				// Jump to Bootloader 	
				bootloader();
				
			break;
				
			// --- illegal command --- 
			default:
				// Write Payload 
				at86rf231_tx_frame.payload[0] = 0xFF; // CMD: ERROR_MSG
				at86rf231_tx_frame.payload[1] = 0x01; // ErrorCode

				// Send data request and wait for data
				at86rf231_req_data();

				// leave config mode, start streaming
				cmd_end_flag = 1;
				break;
			}
		}

		// No ACK received?, start streaming 
		if (pal_trx_bit_read(SR_TRAC_STATUS) == TRAC_NO_ACK)
		{
			cmd_end_flag = 1;
		}

		// No command received for more than 5s?, start streaming 
		if (config_timer >= 500000)
		{
			config_timer = 0;
			cmd_end_flag = 1;
		}

		config_timer++;
		_delay_us(10);
		wdt_reset();
	}

	// Reset CMD 
	at86rf231_rx_frame.payload[0] = 0x00;

	cmd_end_flag = 0;

	// Enable Timer 
	TIMSK1 = 1 << OCIE1A; // Timer1 Output Compare A Match
	TIFR1 = 0x02;

	// Set mode = TRX_OFF 
	pal_trx_reg_write(RG_TRX_STATE, CMD_TRX_OFF);

	// verify that state = TRX_OFF 
	while (tal_trx_status != TRX_OFF)
	{
		tal_trx_status = (tal_trx_status_t) pal_trx_bit_read(SR_TRX_STATUS);
	}

	// Set RF231 State = PLL_ON, ready to tx 
	at86rf231_switch_pll_on();
}