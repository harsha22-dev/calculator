// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file usb.c
 *
 * \brief receiving and sending data from/to CDC
 *
 * <b>Target Platforms:</b> AVR32UC3A1256
 *
 * <b>Editor:</b> Eclipse
 *
 * <b>Compiler:</b> GNU Compiler Collection (GCC) 4.1.4
 *
 * \date 2009-04-03
 *
 * \version V1.00 Exp (experimental)
 * \author                  - Florian Merz, pro-micron
 *
 *      2009-01-12      flm      - Erste Version: - USB-CDC-Task gibt Daten aus
 *                                               - USB-CDC-Task liest Daten ein
 *
 *      2010-03-18      flm      - Erweiterung um Terminal Zusatzfunktionen
 *
 * <b>Copyright &copy;2008 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>*/
//-----------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------//
// Header Files
//-----------------------------------------------------------------------------------------------//
/* Standard includes */
#include "main_app.h"
#include <stdarg.h>
#include "queue.h"
#include "partest.h"
#include "flash.h"
#include "wdt.h"
#include "at86.h"
#include "ieee_const.h"
#include "tca6424.h"
#include "spike_fw_update.h"

/* USB */
#include "uart_usb_lib.h"
#include "nlao_cpu.h"
#include "print_funcs.h"
#include "conf_usb.h"
#include "usb_task.h"
#include "device_cdc_task.h"
#include "usb_drv.h"
#include "usb_descriptors.h"
#include "usb_standard_request.h"

//-----------------------------------------------------------------------------------------------//
// Global data
//-----------------------------------------------------------------------------------------------//
char debug_string[2048] =
	{ 0 };
uint16_t sof_cnt;

extern app_control_struct app_control;
extern volatile Bool usb_connected;
extern uint8_t index_average;
extern int32_t avg_values[5][100];
extern int32_t avg_values_result[5];
extern int32_t avg_values_result2[5];
extern volatile frame_info_union rx_frame;
extern volatile frame_info_union tx_frame;
extern volatile uint32_t frequency;

extern volatile bool reset_timer;

extern uint16_t WZH_address;
extern uint8_t empfang;

xSemaphoreHandle xSemaphoreUSB = NULL;
	
extern uint8_t prog_code[128];
char hex_buffer[10][44];	
uint8_t char_to_uint = 0;
extern uint16_t prog_size;

/* prototypes */
void
print_main_menu(void);
void
print_transceiver_config(void);
void
print_tx_command(void);
uint8_t
change_channel_remote(uint8_t channel);

pm_freq_param_t pm_freq_param =
	{ .cpu_f = 24000000, .pba_f = 24000000, .osc0_f = FOSC0, .osc0_startup = OSC0_STARTUP };
xTaskHandle xHandleUSB;
//-----------------------------------------------------------------------------------------------//
// Main Application
//-----------------------------------------------------------------------------------------------//
void
usb_start(void)
{

	/* we need no Parameters to pass */
	unsigned char ucParameterToPass = 0;


	/* Init USB clock */
	pm_configure_usb_clock();


	/* Init USB task */
	usb_task_init();


	/* Init CDC USB Device */
	uart_usb_init();

	Usb_enable_sof_interrupt();

	sof_cnt = 0;


	/* Create the task for USB-CDC (emulate com-port) */
	xTaskCreate(vTaskUSB, // pvTaskCode
	        (signed portCHAR *) "USB_CDC", // pcName
	        2048, // usStackDepth
	        &ucParameterToPass, // pvParameters
	        configTSK_USB_DCDC_PRIORITY, // uxPriority
	        &xHandleUSB); // pvCreatedTask
}

signed portCHAR cChar;
portBASE_TYPE retstatus;
portBASE_TYPE xTaskWokenByTx = pdFALSE;

void
find_channel(void)
{
	uint8_t ch = 0x0b;
//	usb_uart_printf("\r\nSearching channels...");
	for (ch = 0x0b; ch < 0x1b; ch++)
	{
		/* write new channel into transceiver */
		at86rf231_change_channel(ch);

		/* set transceiver mode: receive */
		at86rf231_rx_with_auto_ack();

//		usb_uart_printf(" 0x%x", ch);

		empfang = 0;

		vTaskDelay(20);

		if (empfang)
		{
			/* Kanal in User Page speichern */
			ptr_flash_data->rf231_channel = ch;
			eeprom_save_config();
//			usb_uart_printf("\r\n-> found: channel 0x%x\r\n", ch);
			return;
		}
	}
//	usb_uart_printf("\r\n-> none found! \r\n");

	/* write new channel into transceiver */
	at86rf231_change_channel(ptr_flash_data->rf231_channel);

	/* set transceiver mode: receive */
	at86rf231_rx_with_auto_ack();
}

void
vTaskUSB(void *pvParameters)
{
	int c;
	float fehlerrate = 0;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	char buffer[10] =
		{ 0 };
	int val1;
	uint16_t timeout = 0;
	uint8_t last_streaming = 0;

	uint8_t cca_stat = 0;
	uint16_t i = 0; 
	uint16_t k = 0;
	int16_t energy_lvl = 0;
	double avg_energy_lvl[16] = {0.0};
	uint16_t avrg_nr = 500;
	
	xSemaphoreUSB = xSemaphoreCreateMutex();

	while (1)
	{
		vTaskDelay(50);


		// First, check the device enumeration state
		if (!Is_device_enumerated())
			continue;


		/* Daten von CDC empfangen ? */
		if (uart_usb_test_hit())
		{
			app_control.streaming = 0;
			vTaskDelay(1);


			/* flush tx buffer */
			uart_usb_flush_slow();


			/* Zeichen einlesen */
			c = uart_usb_getchar();

			switch (c)
			{
			/* Task-Ausgabe */
			case 'x':
				/* streaming = off */
				app_control.streaming = 0;
				vTaskDelay(6);

				usb_uart_printf("\f\n\rTask          State  Priority  Stack  #\r\n************************************************\r\n");
				/* list of tasks and their status... */
				taskENTER_CRITICAL();
				vTaskList((signed char *) debug_string);
				usb_uart_printf("%s \n\r", debug_string);
				taskEXIT_CRITICAL();
				break;


				/* Streaming aus/an */
			case 's': 
				app_control.streaming = 0;
				vTaskDelay(6);
				//rtc_set_value(&AVR32_RTC, 0);
				reset_timer = true;
				//data_printf( "\r\n\nStreamingmode = 3 (full data)\r\n" );
				//data_printf( "Fz     Mz     Mxy    TIME / s \r\n" );
				app_control.streaming = 3;
				uart_usb_flush_slow();
				sof_cnt = 0;
				break;


				/*			if (app_control.streaming == 0)
				 {
				 app_control.streaming = 0;
				 vTaskDelay(6);
				 data_printf("\r\n\nStreamingmode = 1 \r\n\r\n");
				 data_printf("Fz     Mz     Mxy    RSSI   BATT. TIME / s \r\n");
				 //  +00000;+00000;+00000;-59dBm;4.30V;281.738375s" );
				 app_control.streaming = 1;
				 }

				 else if (app_control.streaming == 1)
				 {
				 app_control.streaming = 0;
				 vTaskDelay(6);
				 data_printf("\r\n\nStreamingmode = 2 (LogView)\r\n");
				 app_control.streaming = 2;
				 }
				 else if (app_control.streaming == 2)
				 {
				 app_control.streaming = 0;
				 vTaskDelay(6);
				 data_printf("\r\n\nStreamingmode = 3 (full data)\r\n");
				 data_printf("Fz     Mz     Mxy    TIME / s \r\n");
				 //-00022;+00016;+00187;520.049000
				 app_control.streaming = 3;
				 }
				 else if (app_control.streaming == 3)
				 {
				 app_control.streaming = 0;
				 vTaskDelay(6);
				 data_printf("\r\n\nStreamingmode = 4 full raw data\r\n");
				 data_printf("Fz     Mz     Mx     My     TIME / s \r\n");
				 //+00053;+00035;-00022;-00034;2.977338
				 app_control.streaming = 4;
				 }
				 else if (app_control.streaming == 4)
				 {
				 app_control.streaming = 0;
				 vTaskDelay(6);
				 data_printf("\r\n\nStreamingmode = 0 (no streaming)\r\n");
				 app_control.streaming = 0;
				 }
				 app_control.set_dac_manual = 0;
				 uart_usb_flush_slow();
				 sof_cnt = 0;
				 break;*/

				/* Empfangs-Statistik */
			case 'i':
				/* streaming = off */
				app_control.streaming = 0;
				vTaskDelay(6);

				usb_uart_printf("\n\n\rReceive statistics:\r\n");
				usb_uart_printf("Total packets: %u \r\n", app_control.rec_total);
				usb_uart_printf("CRC ERROR: %u \r\n", app_control.error_crc_cnt);
				usb_uart_printf("Frames missing: %u \r\n", app_control.frames_missing);

				if (app_control.rec_total > 0)
				{
					fehlerrate = (((float) app_control.error_crc_cnt * 100) / ((float) app_control.rec_total + (float) app_control.error_crc_cnt));
					usb_uart_printf("CRC_ERR: %.2f %% \n\r", fehlerrate);

					fehlerrate = (((float) app_control.frames_missing * 100) / ((float) app_control.rec_total + (float) app_control.frames_missing));
					usb_uart_printf("MISSING_ERR: %.2f %% \r\n\n\n", fehlerrate);
				}
				else
				{
					usb_uart_printf("CRC_ERR:\r\n");
					usb_uart_printf("MISSING_ERR:\r\n\n\n");
				}
				break;


				/* Empfangsstatistik zurücksetzen */
			case 'r':
				/* streaming = off */
				app_control.streaming = 0;
				vTaskDelay(6);

				app_control.streaming = 0;
				app_control.rec_total = 0;
				app_control.error_crc_cnt = 0;
				app_control.frames_missing = 0;
				usb_uart_printf("\r\n\nStatistik resetted\r\n");
				usb_uart_printf("Time resetted\r\n\n");
				break;


				/* Set moving average size */
			case 'm':
				/* streaming = off */
				app_control.streaming = 0;
				vTaskDelay(6);

				sof_cnt = NB_MS_BEFORE_FLUSH; // force flush
				usb_uart_printf("\r\nFilter does only affect streaming mode 3 and 4!\r\n1 = Disabled (raw data)\r\nEnter size: ");
				usb_uart_get_string(buffer, 10);
				val1 = atoi(buffer);
				if ((val1 < 100) && (val1 != 0))
				{
					app_control.moving_average_size = val1;
					index_average = 0;
					avg_values_result[0] = 0;
					avg_values_result[1] = 0;
					avg_values_result[2] = 0;
					avg_values_result[3] = 0;
					avg_values_result[4] = 0;
				}
				else
					usb_uart_printf("Valid values: 1-99! ");
				break;


				//              /* DAC setzen */
				//              case 'd':
				//                  usb_uart_printf( "Set DAC: Enter value [dec]: " );
				//                  usb_uart_get_string( buffer, 10 );
				//                  val1 = atoi( buffer );
				//                  app_control.set_dac_manual = 1;
				//                  set_dac( ( unsigned int )val1 );
				//                  break;

				//              /* PWM setzen*/
				//              case 'p':
				//                  usb_uart_printf( "Store Value in Userpage: default: 370; akt. Wert: %d [dec]: ", *( unsigned short * )0x808001F8 );
				//                  usb_uart_get_string( buffer, 10 );
				//                  val1 = atoi( buffer );
				//
				//                  taskENTER_CRITICAL();
				//                  /* Frequenz in User Page speichern */
				//                  flashc_memset16( ( unsigned char * )0x808001F8, ( U16 )val1, 2, TRUE );
				//                  taskEXIT_CRITICAL();
				//
				//                  pwm_disable();
				//                  pwm_init( ( unsigned int )val1 );
				//                  pwm_enable();
				//                  pwm_disable();
				//                  break;

			case '0':
				app_control.streaming = 0;
				vTaskDelay(6);
				//data_printf( "\r\n\nStreamingmode = 0 (no streaming)\r\n" );
				app_control.streaming = 0;
				break;

			case '3':
				app_control.streaming = 0;
				vTaskDelay(6);
				//rtc_set_value(&AVR32_RTC, 0);
				reset_timer = true;
				//data_printf( "\r\n\nStreamingmode = 3 (full data)\r\n" );
				//data_printf( "Fz     Mz     Mxy    TIME / s \r\n" );
				app_control.streaming = 3;
				uart_usb_flush_slow();
				sof_cnt = 0;
				break;


				/* logview-start */
			case 17:
				app_control.streaming = 2;
				break;
			case 18:
				app_control.streaming = 2;
				break;
			case 19:
				app_control.streaming = 2;
				break;
			case 20:
				app_control.streaming = 2;
				break;


				// Find channel
			case 'f':
				find_channel();
				break;

			
			case 'a':
			
				usb_uart_printf("Select Intel HEX File \r\n");
				prog_size =  store_prog_code_in_flash();
				usb_uart_printf("\r\nUpload finished !!!\r\n");
				break;
				
			case 'o':
				// Setup CCA 
				
				usb_uart_printf("\r\nEnter ED Threshold lvl = RSSI_BASE + 2*threshold: ");
				usb_uart_get_string(buffer, 3);
				sscanf(buffer, "%i", &val1);
				
				usb_uart_printf("\r\n threshold = %i dBm \r\n",(-90+2*val1));
				
				pal_trx_bit_write(SR_CCA_MODE,1);	// Energy above threshold
				pal_trx_bit_write(SR_CCA_ED_THRES,val1);
				
				pal_trx_bit_write(SR_TRX_CMD,RX_ON);			
				while(pal_trx_bit_read(SR_TRX_STATUS) != RX_ON)	// Wait until TRX is in TX_Ready mode
				{ } 
				
				cca_stat = 1;		// 1 = idle channel 
				for (i = 0; i < 10000; i++)
				{	
					do 
					{		
						if(cca_stat == 0)
							usb_uart_printf("!!! Channel -> BUSY  !!!! \r\n");
		
						pal_trx_bit_write(SR_CCA_REQUEST,1);
	
						while (pal_trx_bit_read(SR_CCA_DONE) == 0)
						{
						}
						cca_stat = pal_trx_bit_read(SR_CCA_STATUS);
					
						usb_uart_printf("CCA STAT = %i, ED LVL = %i dBm  ",cca_stat,(-90+pal_trx_bit_read(SR_ED_LEVEL)));
					
					}while( cca_stat == 0);	// While Busy channel 	
					usb_uart_printf( " Channel -> IDLE\r\n");
				}				
				break;	
			
			case 'e':
			
				pal_trx_bit_write(SR_TRX_CMD,RX_ON);
				while (pal_trx_bit_read(SR_TRX_CMD) != RX_ON)
				{
					vTaskDelay(1);
				}
				for (i = 0xB; i <= 0x1A; i++)
				{
					pal_trx_bit_write(SR_CHANNEL,i);
					while (pal_trx_bit_read(SR_IRQ_0_PLL_LOCK) != 1)
					{
					}
					pal_trx_bit_write(SR_ED_LEVEL,0xFF);
					vTaskDelay(1);
					energy_lvl = pal_trx_bit_read(SR_ED_LEVEL) + RSSI_BASE_VAL - 1;	// RSSI_BASE_VAL = -90 but should be -91
					
					usb_uart_printf("channel %i, energy level = %i\r\n",i,energy_lvl);
				}	
				usb_uart_printf("\r\n");			
				break;
			
			case 'p':	
				
				for (i = 0xB; i <= 0x1A; i++)
				{
					avg_energy_lvl[i-0x0B] = 0;
				}	
				
				usb_uart_printf("\r\n Average over: ");
				usb_uart_get_string(buffer, 5);
				sscanf(buffer, "%i", &val1);
				
				usb_uart_printf("N = %i \r\n",val1);
				avrg_nr = val1;
				
				pal_trx_bit_write(SR_TRX_CMD,RX_ON);
				while (pal_trx_bit_read(SR_TRX_CMD) != RX_ON)
				{
					vTaskDelay(1);
				}
				
				for (k = 0; k < avrg_nr; k++)
				{
					for (i = 0xB; i <= 0x1A; i++)
					{	
						pal_trx_bit_write(SR_CHANNEL,i);
						
						while (pal_trx_bit_read(SR_IRQ_0_PLL_LOCK) != 1)
						{
						}
						
						pal_trx_bit_write(SR_ED_LEVEL,0xFF);
						vTaskDelay(1);
						energy_lvl = pal_trx_bit_read(SR_ED_LEVEL) + RSSI_BASE_VAL - 1;	// RSSI_BASE_VAL = -90 but should be -91
					    
						avg_energy_lvl[i-0x0B] += (double)energy_lvl;
					}
					vTaskDelay(1);
				}
					
				for (i = 0xB; i <= 0x1A; i++)
				{
					avg_energy_lvl[i-0x0B] = avg_energy_lvl[i-0x0B]/avrg_nr;
					usb_uart_printf("channel %i, avg energy %f\r\n",i,avg_energy_lvl[i-0xB]);
				}						
				usb_uart_printf("\r\n");			
				break;

				/* Zero strain gauges */
			case 'z':
				last_streaming = app_control.streaming;
				/* streaming = off */
				app_control.streaming = 0;
				vTaskDelay(6);


				//usb_uart_printf( "\r\nSetting strain gauges zero\r\n" );

				/* send spike-zero command and receive answer */
				val1 = zero_strain_gauges();
				//   app_control.streaming = 1;
				if (val1)
				{
					//usb_uart_printf("ZERO OK, time: %dms\r\n", val1);
					//usb_uart_printf("ZERO OK");
				}
				else
				{
					//usb_uart_printf("ZERO ERROR\r\n");
				}

				if (last_streaming == 0)
				{
					rtc_set_value(&AVR32_RTC, 0);
					app_control.streaming = 3;
				}
				else
				{
					app_control.streaming = last_streaming;
				}
				break;


				/* Tx user data */
			case 't':
				app_control.streaming = 0;
				vTaskDelay(5);


				/* print menu */
				print_tx_command();
				break;


				/* Config Transceiver */
			case 'c':
				/* streaming off */
				app_control.streaming = 0;


				/* print menu  */
				print_transceiver_config();

				break;


				/* change channel lokal and remote */
			case 'k':
				/* streaming off */
				app_control.streaming = 0;

				usb_uart_printf("Enter new channel; Store in Userpage: [hex]: ");
				usb_uart_get_string(buffer, 10);
				sscanf(buffer, "%X", &val1);

				change_channel_remote((uint8_t) val1);
				break;


				/* Write calibration values remote */
			case 'w':
				/* streaming off */
				app_control.streaming = 0;


				/* Build tx frame */
				tx_frame.at86rf231_frame.frame_length = 48;
				tx_frame.at86rf231_frame.frame_control_field = bswap_16(FCF_SET_FRAMETYPE( FCF_FRAMETYPE_MAC_CMD ) | FCF_SET_DEST_ADDR_MODE( FCF_SHORT_ADDR ) | FCF_SET_SOURCE_ADDR_MODE( FCF_SHORT_ADDR ) | FCF_ACK_REQUEST);

				tx_frame.at86rf231_frame.seq_number = 0x77;
				tx_frame.at86rf231_frame.dest_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
				if (ptr_flash_data->rf231_dest_address == 0x0000)
				{
					tx_frame.at86rf231_frame.dest_addr = bswap_16(WZH_address);
				}
				else
				{
					tx_frame.at86rf231_frame.dest_addr = bswap_16(ptr_flash_data->rf231_dest_address);
				}
				tx_frame.at86rf231_frame.src_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
				tx_frame.at86rf231_frame.src_addr = bswap_16(ptr_flash_data->rf231_src_address);

				tx_frame.at86rf231_frame.payload[0] = 0x32; // CMD
				tx_frame.at86rf231_frame.payload[1] = 0x00; // Param1
				tx_frame.at86rf231_frame.payload[2] = 0x00; // Param2

				usb_uart_printf("Tension cal value [hex]: ");
				usb_uart_get_string(buffer, 9);
				sscanf(buffer, "%X", &val1);
				tx_frame.at86rf231_frame.payload[3] = val1;
				tx_frame.at86rf231_frame.payload[4] = val1 >> 8;
				tx_frame.at86rf231_frame.payload[5] = val1 >> 16;
				tx_frame.at86rf231_frame.payload[6] = val1 >> 24;

				usb_uart_printf("Torsion cal value [hex]: ");
				usb_uart_get_string(buffer, 9);
				sscanf(buffer, "%X", &val1);
				tx_frame.at86rf231_frame.payload[7] = val1;
				tx_frame.at86rf231_frame.payload[8] = val1 >> 8;
				tx_frame.at86rf231_frame.payload[9] = val1 >> 16;
				tx_frame.at86rf231_frame.payload[10] = val1 >> 24;

				usb_uart_printf("Bending X cal value [hex]: ");
				usb_uart_get_string(buffer, 9);
				sscanf(buffer, "%X", &val1);
				tx_frame.at86rf231_frame.payload[11] = val1;
				tx_frame.at86rf231_frame.payload[12] = val1 >> 8;
				tx_frame.at86rf231_frame.payload[13] = val1 >> 16;
				tx_frame.at86rf231_frame.payload[14] = val1 >> 24;

				usb_uart_printf("Bending Y cal value [hex]: ");
				usb_uart_get_string(buffer, 9);
				sscanf(buffer, "%X", &val1);
				tx_frame.at86rf231_frame.payload[15] = val1;
				tx_frame.at86rf231_frame.payload[16] = val1 >> 8;
				tx_frame.at86rf231_frame.payload[17] = val1 >> 16;
				tx_frame.at86rf231_frame.payload[18] = val1 >> 24;

				usb_uart_printf("TempCalVal1 [hex]: ");
				usb_uart_get_string(buffer, 9);
				sscanf(buffer, "%X", &val1);
				tx_frame.at86rf231_frame.payload[19] = val1;
				tx_frame.at86rf231_frame.payload[20] = val1 >> 8;
				tx_frame.at86rf231_frame.payload[21] = val1 >> 16;
				tx_frame.at86rf231_frame.payload[22] = val1 >> 24;

				usb_uart_printf("TempCalVal2 [hex]: ");
				usb_uart_get_string(buffer, 9);
				sscanf(buffer, "%X", &val1);
				tx_frame.at86rf231_frame.payload[23] = val1;
				tx_frame.at86rf231_frame.payload[24] = val1 >> 8;
				tx_frame.at86rf231_frame.payload[25] = val1 >> 16;
				tx_frame.at86rf231_frame.payload[26] = val1 >> 24;

				usb_uart_printf("TempCalVal3 [hex]: ");
				usb_uart_get_string(buffer, 9);
				sscanf(buffer, "%X", &val1);
				tx_frame.at86rf231_frame.payload[27] = val1;
				tx_frame.at86rf231_frame.payload[28] = val1 >> 8;
				tx_frame.at86rf231_frame.payload[29] = val1 >> 16;
				tx_frame.at86rf231_frame.payload[30] = val1 >> 24;

				usb_uart_printf("TempCalVal4 [hex]: ");
				usb_uart_get_string(buffer, 9);
				sscanf(buffer, "%X", &val1);
				tx_frame.at86rf231_frame.payload[31] = val1;
				tx_frame.at86rf231_frame.payload[32] = val1 >> 8;
				tx_frame.at86rf231_frame.payload[33] = val1 >> 16;
				tx_frame.at86rf231_frame.payload[34] = val1 >> 24;


				/* Tx frame with channel command */
				vTaskDelay(31);
				usb_uart_printf("sending cmd...\r\n");
				send_frame((uint8_t*) &tx_frame, CSMA_UNSLOTTED, 1);
				at86rf231_rx_with_auto_ack();
				break;


				/* Read calibration values remote */
			case 'l':
				/* streaming off */
				app_control.streaming = 0;
				usb_uart_printf("\r\nRequesting Cal values...\r\n");


				/* Build tx frame */
				tx_frame.at86rf231_frame.frame_length = 32;
				tx_frame.at86rf231_frame.frame_control_field = bswap_16(FCF_SET_FRAMETYPE( FCF_FRAMETYPE_MAC_CMD ) | FCF_SET_DEST_ADDR_MODE( FCF_SHORT_ADDR ) | FCF_SET_SOURCE_ADDR_MODE( FCF_SHORT_ADDR ) | FCF_ACK_REQUEST);

				tx_frame.at86rf231_frame.seq_number = 0x77;
				tx_frame.at86rf231_frame.dest_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
				if (ptr_flash_data->rf231_dest_address == 0x0000)
				{
					tx_frame.at86rf231_frame.dest_addr = bswap_16(WZH_address);
				}
				else
				{
					tx_frame.at86rf231_frame.dest_addr = bswap_16(ptr_flash_data->rf231_dest_address);
				}
				tx_frame.at86rf231_frame.src_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
				tx_frame.at86rf231_frame.src_addr = bswap_16(ptr_flash_data->rf231_src_address);

				tx_frame.at86rf231_frame.payload[0] = 0x33; // CMD
				tx_frame.at86rf231_frame.payload[1] = 0x00; // Param1
				tx_frame.at86rf231_frame.payload[2] = 0x00; // Param2

				/* Tx frame with channel command */
				vTaskDelay(10);
				send_frame((uint8_t*) &tx_frame, CSMA_UNSLOTTED, 1);

				at86rf231_rx_with_auto_ack();


				/* wait 3 seconds for confirmation on new channel*/
				timeout = 0;
				while (timeout < 3000)
				{
					vTaskDelay(1);
					timeout++;
					if (((rx_frame.at86rf231_frame.frame_control_field & 7) == FCF_FRAMETYPE_MAC_CMD) && (rx_frame.at86rf231_frame.payload[0] == 0x0B) && (rx_frame.at86rf231_frame.payload[1] == 0x33))
					{
						val1 = (rx_frame.at86rf231_frame.payload[6] << 24) + (rx_frame.at86rf231_frame.payload[5] << 16) + (rx_frame.at86rf231_frame.payload[4] << 8) + (rx_frame.at86rf231_frame.payload[3]);
						usb_uart_printf("Tension cal value: 0x%x\r\n", val1);

						val1 = (rx_frame.at86rf231_frame.payload[10] << 24) + (rx_frame.at86rf231_frame.payload[9] << 16) + (rx_frame.at86rf231_frame.payload[8] << 8) + (rx_frame.at86rf231_frame.payload[7]);
						usb_uart_printf("Torsion cal value: 0x%x\r\n", val1);

						val1 = (rx_frame.at86rf231_frame.payload[14] << 24) + (rx_frame.at86rf231_frame.payload[13] << 16) + (rx_frame.at86rf231_frame.payload[12] << 8) + (rx_frame.at86rf231_frame.payload[11]);
						usb_uart_printf("Bending X cal value: 0x%x\r\n", val1);

						val1 = (rx_frame.at86rf231_frame.payload[18] << 24) + (rx_frame.at86rf231_frame.payload[17] << 16) + (rx_frame.at86rf231_frame.payload[16] << 8) + (rx_frame.at86rf231_frame.payload[15]);
						usb_uart_printf("Bending Y cal value: 0x%x\r\n", val1);

						val1 = (rx_frame.at86rf231_frame.payload[22] << 24) + (rx_frame.at86rf231_frame.payload[21] << 16) + (rx_frame.at86rf231_frame.payload[20] << 8) + (rx_frame.at86rf231_frame.payload[19]);
						usb_uart_printf("TempCalVal1: 0x%x\r\n", val1);

						val1 = (rx_frame.at86rf231_frame.payload[26] << 24) + (rx_frame.at86rf231_frame.payload[25] << 16) + (rx_frame.at86rf231_frame.payload[24] << 8) + (rx_frame.at86rf231_frame.payload[23]);
						usb_uart_printf("TempCalVal2: 0x%x\r\n", val1);

						val1 = (rx_frame.at86rf231_frame.payload[30] << 24) + (rx_frame.at86rf231_frame.payload[29] << 16) + (rx_frame.at86rf231_frame.payload[28] << 8) + (rx_frame.at86rf231_frame.payload[27]);
						usb_uart_printf("TempCalVal3: 0x%x\r\n", val1);

						val1 = (rx_frame.at86rf231_frame.payload[34] << 24) + (rx_frame.at86rf231_frame.payload[33] << 16) + (rx_frame.at86rf231_frame.payload[32] << 8) + (rx_frame.at86rf231_frame.payload[31]);
						usb_uart_printf("TempCalVal4: 0x%x\r\n", val1);
						//ret_val = timeout;
						break;
					}
				}


				/* No confirmation received in timeout */
				if (timeout >= 1000)
				{
					//ret_val = FALSE;
					usb_uart_printf("timeout\r\n");
				}
				timeout = 0;

				break;


				/* reset */
			case 'n':
				/* streaming = off */
				app_control.streaming = 0;

				usb_uart_printf("reset MCU in 3s\r\n");
				vTaskDelay(500);
				usb_uart_printf("#####\r\n");
				vTaskDelay(500);
				usb_uart_printf("####\r\n");
				vTaskDelay(500);
				usb_uart_printf("###\r\n");
				vTaskDelay(500);
				usb_uart_printf("##\r\n");
				vTaskDelay(500);
				usb_uart_printf("#\r\n");
				vTaskDelay(500);
				usb_uart_printf("reset; wait 2 more seconds \r\n");

				wdt_disable();
				wdt_enable(2000000);

				while (1)
					;

				break;

			case 'd':
                /* streaming = off */
                app_control.streaming = 0;
                vTaskDelay(6);

                sof_cnt = NB_MS_BEFORE_FLUSH; // force flush
                usb_uart_printf( "\r\nTreiberfrequenz ist: %d Hz", frequency);
                usb_uart_printf( "\r\nTreiberfrequenz neu in Hz: " );
                usb_uart_get_string( buffer, 10 );
                val1 = atoi( buffer );
//                if((val1>10000)&&(val1<80000))
//                {
					frequency=val1;
         			ptr_flash_data->pwm_freq = (uint16_t)(66000000.0 / frequency);
         			eeprom_save_config();
                 	pwm_enable();
//                }
//                else
//                {
//                 	usb_uart_printf( "Valid values: 10000-80000! " );
//                }
             	break;

			case 'b':
                /* streaming = off */
				app_control.streaming = 0;
				init_spike_fw_update_task();
				vTaskDelay(5);
             break;
			 
			 
			 
			 
			 
			 
				 
				/* Hilfe Menü */
			default:
				/* streaming = off */
				app_control.streaming = 0;
				vTaskDelay(6);


				/* set_dac_manual = 0; */
				app_control.set_dac_manual = 0;


				/* print menu */
				print_main_menu();
				break;
			}
		}
	}
}

void
usb_uart_write_line(const char *string, unsigned char enable_fast)
{
	while (*string != '\0')
	{
		uart_usb_putchar(*string++, enable_fast);
	}
}

int
usb_uart_printf(const char *fmt, ...)
{
	static va_list ap;
	static int ret;
	sof_cnt = NB_MS_BEFORE_FLUSH;
	if (xSemaphoreUSB != NULL && Is_device_enumerated() && !Is_usb_clock_frozen() && (usb_connected == TRUE))
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// poll to see if it becomes free.
		while (xSemaphoreTake(xSemaphoreUSB , 0xFFFF ) != pdTRUE)
			;

		va_start(ap, fmt);

		taskENTER_CRITICAL();
		ret = vsprintf(debug_string, fmt, ap);
		taskEXIT_CRITICAL();

		va_end(ap);

		usb_uart_write_line(debug_string, 0);

		if (sof_cnt >= NB_MS_BEFORE_FLUSH) //Flush buffer in Timeout
		{
			sof_cnt = 0;
			uart_usb_flush_slow();
		}


		// We have finished accessing the shared resource.  Release
		xSemaphoreGive( xSemaphoreUSB );
		return ret;
	}
	else
		return 0;
}

int
usb_uart_printf_fast(const char *fmt, ...)
{
	static va_list ap;
	static int ret;

	if (Is_device_enumerated() && !Is_usb_clock_frozen() && (usb_connected == TRUE))
	{
		va_start(ap, fmt);
		taskENTER_CRITICAL();
		ret = vsprintf(debug_string, fmt, ap);

		va_end(ap);
		usb_uart_write_line(debug_string, 1);
		taskEXIT_CRITICAL();
		return ret;
	}
	else
		return 0;
}

void
usb_sof_action(void)
{
	sof_cnt++;
}

void
usb_uart_get_string(char* buffer, int buffer_len)
{
	int32_t i = 0;
	uint8_t c;

	while (((c = uart_usb_getchar()) != 0x0d) && (i < buffer_len - 1))
	{
		sof_cnt = NB_MS_BEFORE_FLUSH;


		/* Escape char? */
		if (c == 0x1B)
		{
			break;
		}

		buffer[i++] = c;
		usb_uart_printf("%c", c);
		vTaskDelay(10);
	}
	buffer[i] = '\0';
	usb_uart_printf("\r\n");
}

void
print_main_menu(void)
{
	/* Hilfe-Screen drucken */
	usb_uart_printf("\f\r\n\n\n\n\n\n\n\n\n\n\n\n\n");
	usb_uart_printf("--------------------------------------------------------------- \r\n");
	usb_uart_printf("#####  #####   ####        #    # #  ####  #####   ####  #    # \r\n");
	usb_uart_printf("#    # #    # #    #       ##  ## # #    # #    # #    # ##   # \r\n");
	usb_uart_printf("#    # #    # #    # ##### # ## # # #      #    # #    # # #  # \r\n");
	usb_uart_printf("#####  #####  #    #       #    # # #      #####  #    # #  # # \r\n");
	usb_uart_printf("#      #   #  #    #       #    # # #    # #   #  #    # #   ## \r\n");
	usb_uart_printf("#      #    #  ####        #    # #  ####  #    #  ####  #    # \r\n");
	usb_uart_printf("--------------------------------------------------------------- \r\n");
	usb_uart_printf("   ?: Show help menu                 	\r\n");
	usb_uart_printf("   x: Show tasks                     	\r\n");
	usb_uart_printf("   c: Config radio transceiver        	\r\n");
	usb_uart_printf("   i: Show receive statistics        	\r\n");
	usb_uart_printf("   r: Reset statistics               	\r\n");
	usb_uart_printf("   s: Streaming mode                 	\r\n");
	usb_uart_printf("      0: Disable                     	\r\n");
	usb_uart_printf("      3: Data streaming              	\r\n");
	//usb_uart_printf( "Help:    d: Set DAC                         \r\n" );
	//usb_uart_printf( "Help:    p: Set PWM frequency               \r\n" );
	usb_uart_printf("   m: moving average size: %d, default: 1  \r\n", app_control. moving_average_size);
	usb_uart_printf("   z: Zero strain gauges            	 \r\n");
	usb_uart_printf("   k: Change radio channel remote and local \r\n");
	usb_uart_printf("   w: Write calibration values remote 	\r\n");
	usb_uart_printf("   l: Read calibration values remote 	\r\n");
	usb_uart_printf("   f: Find channel 					\r\n");
	usb_uart_printf("   d: Set Frequency 				 	\r\n");
	usb_uart_printf("   n: Reset System:                 	\r\n");
	usb_uart_printf("   t: Radio tx                      	\r\n");
	usb_uart_printf("   b: Bootloader Command             	\r\n");
	usb_uart_printf("   a: Upload HEX File                  \r\n");
	usb_uart_printf("   e: show energy lvl over channels    \r\n");
	usb_uart_printf("   p: average energy lvls              \r\n");
	usb_uart_printf("--------------------------------------------------------------- \r\n");
}

void
print_transceiver_config(void)
{
	char buffer[10] =
		{ 0 };
	int val1, val2;

	usb_uart_printf("\f--- AT86RF231 RADIO TRANSCEIVER CONFIG PAGE ---\r\n\n");
	usb_uart_printf("ESC: Main menu\r\n");
	usb_uart_printf("r: Read all transceiver registers                    \r\n");
	usb_uart_printf("w: Write one transceiver register                    \r\n");
	usb_uart_printf("c: Change radio channel (0x%x)                       \r\n", ptr_flash_data->rf231_channel);
	usb_uart_printf("p: Change radio channel page (0x%x)                  \r\n", ptr_flash_data->rf231_channel_page);
	usb_uart_printf("e: Set Transceiver mode = receive                    \r\n");
	usb_uart_printf("s: Change Source Address  (0x%x)                     \r\n", ptr_flash_data->rf231_src_address);
	if (ptr_flash_data->rf231_dest_address == 0x0000)
	{
		usb_uart_printf("d: Change Destination Address  (0x%x, auto)           \r\n", WZH_address);
	}
	else
	{
		usb_uart_printf("d: Change Destination Address  (0x%x)                \r\n", ptr_flash_data->rf231_dest_address);
	}

	usb_uart_printf("i: Change PAN ID (0x%x)                              \r\n", ptr_flash_data->rf231_pan_id);
	usb_uart_printf("t: Change TX power (0x%x)                            \r\n", ptr_flash_data->rf231_tx_pwr);
	usb_uart_printf("\r\nEnter CMD: ");


	/* flush tx buffer */
	uart_usb_flush_slow();
	usb_uart_get_string(buffer, 2);
	
	// "ESC" was pushed
	if(buffer[0] == '\0')
	{
		usb_uart_printf("\r\ncanceled\r\n");
		return;
	}	
	
	
	/* Read all transceiver registers */
	if (buffer[0] == 'r')
	{
		usb_uart_printf("\r\n\r\nADR.     VAL. \r\n");

		taskENTER_CRITICAL();
		vTaskDelay(100);
		for (val1 = 0; val1 <= 0x2F; val1++)
		{
			vTaskDelay(1);
			val2 = pal_trx_reg_read((uint8_t) val1);
			usb_uart_printf("0x%x     0x%x \r\n", val1, val2);
		}
		taskEXIT_CRITICAL();
	}


	/* Write one transceiver register */
	else if (buffer[0] == 'w')
	{
		usb_uart_printf("Address [hex]: ");
		usb_uart_get_string(buffer, 10);
		sscanf(buffer, "%X", &val1);

		usb_uart_printf("Value[hex]: ");
		usb_uart_get_string(buffer, 10);
		sscanf(buffer, "%X", &val2);

		taskENTER_CRITICAL();
		pal_trx_reg_write((uint8_t) val1, (uint8_t) val2);
		taskEXIT_CRITICAL();

		usb_uart_printf("Adr: 0x%x     Val: 0x%x \r\n", val1, val2);
	}


	/* Change radio channel */
	else if (buffer[0] == 'c')
	{
		usb_uart_printf("Enter new channel; Store in Userpage: [hex]: ");
		usb_uart_get_string(buffer, 10);
		sscanf(buffer, "%X", &val1);


		/* write new channel into transceiver */
		at86rf231_change_channel((uint8_t) val1);


		/* Kanal in User Page speichern */
		ptr_flash_data->rf231_channel = (uint8_t) val1;
		eeprom_save_config();


		/* set transceiver mode: receive */
		at86rf231_rx_with_auto_ack();
	}


	/* Change radio channel page */
	else if (buffer[0] == 'p')
	{
		usb_uart_printf("0: ALTRATE_250KBPS\r\n2:ALTRATE_500KBPS\r\n16:ALTRATE_1MBPS\r\n17:ALTRATE_2MBPS\r\n : ");
		usb_uart_get_string(buffer, 10);
		sscanf(buffer, "%d", &val1);


		/* write new channel page into transceiver */
		apply_channel_page_configuration((uint8_t) val1);


		/* Kanal in User Page speichern */
		ptr_flash_data->rf231_channel_page = (uint8_t) val1;
		eeprom_save_config();
	}


	/* Set Transceiver mode = receive */
	else if (buffer[0] == 'e')
	{
		at86rf231_rx_with_auto_ack();
		app_control.streaming = 1;
	}


	/* Change Source Address */
	else if (buffer[0] == 's')
	{
		usb_uart_printf("Store address in Userpage: [hex]: ");
		usb_uart_get_string(buffer, 10);
		sscanf(buffer, "%X", &val1);


		/* Kanal in User Page speichern */
		ptr_flash_data->rf231_src_address = (uint16_t) val1;
		eeprom_save_config();
	}


	/* Change Destination Address */
	else if (buffer[0] == 'd')
	{
		usb_uart_printf("Store address in Userpage: [hex]: ");
		usb_uart_get_string(buffer, 10);
		sscanf(buffer, "%X", &val1);


		/* Kanal in User Page speichern */
		ptr_flash_data->rf231_dest_address = (uint16_t) val1;
		eeprom_save_config();
	}


	/* Change PAN ID */
	else if (buffer[0] == 'i')
	{
		usb_uart_printf("Store address in Userpage: [hex]: ");
		usb_uart_get_string(buffer, 10);
		sscanf(buffer, "%X", &val1);


		/* Kanal in User Page speichern */
		ptr_flash_data->rf231_pan_id = (uint16_t) val1;
		eeprom_save_config();
	}


	/* Change TX power */
	else if (buffer[0] == 't')
	{
		usb_uart_printf("Store pwr value in Userpage: [hex]: ");
		usb_uart_get_string(buffer, 10);
		sscanf(buffer, "%X", &val1);


		/* Kanal in User Page speichern */
		ptr_flash_data->rf231_tx_pwr = (uint8_t) val1;
		eeprom_save_config();
	}
}

void
print_tx_command(void)
{
	char buffer[10] =
		{ 0 };
	int val1, val2, val3;
	
	usb_uart_printf( "\fRadio transmit: \r\n\r\n");
	
	usb_uart_printf("COMMANDS:\t\tParam 1\tParam 2\r\n");
	usb_uart_printf("0x0F: STOP_STREAM\t-\t-\r\n");
	usb_uart_printf("0x10: REQ_STREAM\t-\t-\r\n\n");

	//usb_uart_printf("0x11: READ EEPROM\tADDR\t-\r\n");
	//usb_uart_printf("0x12: WRITE EEPROM\tADDR\tVAL\r\n");
	//usb_uart_printf("0x13: WRITE_ZMD_CFG\tZMD\tPAYLOAD \r\n");
	//usb_uart_printf("0x14: READ_ZMD_CFG\tZMD\t-\r\n");
	usb_uart_printf("0x15: READ_VERSION\t-\t-\r\n");
	usb_uart_printf("0x16: SET_TIMEOUT\tTIME1\tTIME2\r\n\n");
	//usb_uart_printf("0x17: SET_SAMPLERATE\tRATE1\tRATE2\r\n\n");

	usb_uart_printf("0x20: C_RADIO_CH\tCH\t-\r\n\n");
	//usb_uart_printf("0x21: C_DATARATE_MODE\tMODE\t-\r\n");
	//usb_uart_printf("0x22: NEW_SRC_ADDRESS\tADDR1\tADDR2\r\n");
	//usb_uart_printf("0x23: NEW_DEST_ADDRESS\tADDR1\tADDR2\r\n");
	//usb_uart_printf("0x24: NEW_PANID\t\tPAN1\tPAN2\r\n");
	//usb_uart_printf("0x25: NEW_TX_PWR\tPWR\t-\r\n\n");

	usb_uart_printf("0x30: READ_TEMPERATURE\t-\t-\r\n");
	
	usb_uart_printf("\r\nEnter CMD: 0x");
	usb_uart_get_string(buffer, 3);
	sscanf(buffer, "%X", &val1);
	
	// check for valid command
	if ( (val1 == 0x0F) |  (val1 == 0x10) | (val1 == 0x15) | (val1 == 0x16) | (val1 == 0x20) | (val1 == 0x30) )
	{
		// check if parameter one is need
		if ( (val1 == 0x16) | (val1 == 0x20) )
		{	
			usb_uart_printf("\r\nEnter Param1: 0x");
			usb_uart_get_string(buffer, 3);
			sscanf(buffer, "%X", &val2);
			
			// check if parameter two is need
			if ( val1 == 0x16 )	
			{
				usb_uart_printf("\r\nEnter Param2: 0x");
				usb_uart_get_string(buffer, 3);
				sscanf(buffer, "%X", &val3);
			}
			else
			{
				val3 = 0;
			}
		}
		else
		{
			val2 = 0;
			val3 = 0;
		}

		tx_frame.at86rf231_frame.frame_length = 25;
		tx_frame.at86rf231_frame.frame_control_field = bswap_16(FCF_SET_FRAMETYPE( FCF_FRAMETYPE_MAC_CMD ) | FCF_SET_DEST_ADDR_MODE( FCF_SHORT_ADDR ) | FCF_SET_SOURCE_ADDR_MODE( FCF_SHORT_ADDR ) | FCF_ACK_REQUEST);

		tx_frame.at86rf231_frame.seq_number = 0x77;
		tx_frame.at86rf231_frame.dest_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
		if (ptr_flash_data->rf231_dest_address == 0x0000)
		{
			tx_frame.at86rf231_frame.dest_addr = bswap_16(WZH_address);
		}
		else
		{
			tx_frame.at86rf231_frame.dest_addr = bswap_16(ptr_flash_data->rf231_dest_address);
		}
		tx_frame.at86rf231_frame.src_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
		tx_frame.at86rf231_frame.src_addr = bswap_16(ptr_flash_data->rf231_src_address);

		tx_frame.at86rf231_frame.payload[0] = val1; // CMD
		tx_frame.at86rf231_frame.payload[1] = val2; // Param1
		tx_frame.at86rf231_frame.payload[2] = val3; // Param2

		vTaskDelay(31);
		send_frame((uint8_t*) &tx_frame, CSMA_UNSLOTTED, 1);

		at86rf231_rx_with_auto_ack();
	}
	else 
	{
		usb_uart_printf("\r\nERROR: Invalid command! \r\n");
	}
}

uint8_t
change_channel_remote(uint8_t channel)
{
	uint8_t ret_val = 0;
	uint16_t timeout = 0;


	/* Build tx frame */
	tx_frame.at86rf231_frame.frame_length = 30;
	tx_frame.at86rf231_frame.frame_control_field = bswap_16(FCF_SET_FRAMETYPE( FCF_FRAMETYPE_MAC_CMD ) | FCF_SET_DEST_ADDR_MODE( FCF_SHORT_ADDR ) | FCF_SET_SOURCE_ADDR_MODE( FCF_SHORT_ADDR ) | FCF_ACK_REQUEST);

	tx_frame.at86rf231_frame.seq_number = 0x77;
	tx_frame.at86rf231_frame.dest_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
	if (ptr_flash_data->rf231_dest_address == 0x0000)
	{
		tx_frame.at86rf231_frame.dest_addr = bswap_16(WZH_address);
	}
	else
	{
		tx_frame.at86rf231_frame.dest_addr = bswap_16(ptr_flash_data->rf231_dest_address);
	}
	tx_frame.at86rf231_frame.src_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
	tx_frame.at86rf231_frame.src_addr = bswap_16(ptr_flash_data->rf231_src_address);

	tx_frame.at86rf231_frame.payload[0] = 0x20; // CMD
	tx_frame.at86rf231_frame.payload[1] = channel; // Param1
	tx_frame.at86rf231_frame.payload[2] = 0x00; // Param2

	/* Tx frame with channel command */
	vTaskDelay(31);
	usb_uart_printf("sending cmd: ch change\r\n");
	send_frame((uint8_t*) &tx_frame, CSMA_UNSLOTTED, 1);


	/* write new channel into transceiver */
	if (at86rf231_change_channel(channel) == channel)
	{
		usb_uart_printf("lokal ch change ok\r\n");
		/* Kanal in User Page speichern */
		ptr_flash_data->rf231_channel = channel;
		eeprom_save_config();
		at86rf231_rx_with_auto_ack();
	}
	else
	{
		usb_uart_printf("lokal ch change ok\r\n");
		ret_val = FALSE;
	}

	/* wait 3 seconds for confirmation on new channel*/
	while (timeout < 3000)
	{
		vTaskDelay(1);
		timeout++;
		if (((rx_frame.at86rf231_frame.frame_control_field & 7) == FCF_FRAMETYPE_MAC_CMD) && (rx_frame.at86rf231_frame.payload[0] == 0x0B) && (rx_frame.at86rf231_frame.payload[1] == 0x20) && (rx_frame.at86rf231_frame.payload[2] == channel))
		{
			ret_val = timeout;
			usb_uart_printf("change ok\r\n");
			break;
		}
	}

	/* No ACK received? */
	if (pal_trx_bit_read(SR_TRAC_STATUS) == TRAC_NO_ACK)
	{
		ret_val = FALSE;
		usb_uart_printf("NO ACK rec\r\n");
	}

	/* No confirmation received in timeout */
	if (timeout >= 3000)
	{
		ret_val = FALSE;
		usb_uart_printf("timeout\r\n");
	}

	return ret_val;
}

uint32_t usb_get_word(void)
{
	register uint32_t data_rx;
	extern U8 dev_rx_cnt;
	
	while( !uart_usb_test_hit() )
	{
	   vTaskDelay( 10 );
	}

	data_rx=Usb_read_endpoint_data(RX_EP, 32);
	dev_rx_cnt--;
	if( dev_rx_cnt==0 ) Usb_ack_out_received_free(RX_EP);

	return data_rx;
}

