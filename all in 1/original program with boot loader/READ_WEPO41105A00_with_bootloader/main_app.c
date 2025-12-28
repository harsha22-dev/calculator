// This file has been prepared for Doxygen automatic documentation generation.-------------------//

/**\file main_app.c
 *
 * \brief Receiving and processing data from AT86RF231
 *
 * \details Functional description: Receiving and processing data from CC2500
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
 * \author      - Michael Hobler, pro-micron
 *              - Frank Timmler, pro-micron
 *              - Florian Merz, pro-micron
 *
 * \bug ...
 * \todo ...
 *
 *
 * \version
 *      2010-03-18     flm     - 1st release
 *
 * <b>Copyright &copy;2008 pro-micron GmbH & Co. KG modular systems, All rights reserved.</b>
 //-----------------------------------------------------------------------------------------------//                                                                                               */

//-----------------------------------------------------------------------------------------------//
// Header Files
//-----------------------------------------------------------------------------------------------//
#include "main_app.h"
#include <math.h>
#include <inttypes.h>
#include "at86.h"
#include "tal_constants.h"
#include "ieee_const.h"
#include "string.h"
#include "tca6424.h"
#include "usb_standard_request.h"


#define NMB_IGNORE_FIRST_FRAMES 30

//-----------------------------------------------------------------------------------------------//
// Global data
//-----------------------------------------------------------------------------------------------//
extern rf231_rec_struct rf231_rec_info;
extern frame_info_union rx_frame;
extern frame_info_union tx_frame;
extern xSemaphoreHandle xSem_radio_receive;

volatile uint32_t frequency = 0;

/* AT86RF231 data */
app_control_struct app_control =
	{ 0 };
cock_data cock_d;

/* Moving average filter data */
uint8_t index_average = 0;
int32_t avg_values[5][100];
int32_t avg_values_result[5] =
	{ 0 };
int32_t avg_values_result2[5] =
	{ 0 };

spike_payload *spike_payload_ptr;
uint32_t fast_usb_string[4096];
uint8_t power_on_cnt = 0;

uint16_t spannung = 0;
float temperature = 0.0;

uint16_t WZH_address = 0xFFFF;
uint8_t empfang = 0;

volatile uint16_t ptr_over_flow = 0;
volatile uint16_t ptr_offset = 0;
volatile bool reset_timer = false;

flash_data struct_flash_data =
	{ 0x0101, // SW-Version
	        0x0202, // HW-Version
	        0x8888, // rf231_src_address
	        0x0000, // rf231_dest_address
	        0xFFFF, // rf231_pan_id
	        0x0B, // rf231_channel
	        0x02, // rf231_channel_page
	        0x00, // rf231_tx pwr
	        0x0190, // pwm freq. for coil supply
	        0x929E1424 // Bootloader configuration
	    };
flash_data *ptr_flash_data = &struct_flash_data;

void find_channel(void);


/* Handle for the Task */
	xTaskHandle xHandleCC2, xHandleTIMING_tsk, xHandleCONTROL_tsk;
	
//-----------------------------------------------------------------------------------------------//
// TASKS: Main Application
//-----------------------------------------------------------------------------------------------//
void
cock_application_init(void)
{
	

	/* we need no Paramteres to pass to the Task */
	uint8_t ucParameterToPass = 0;


	/* Create task for CC2500 radio transceiver data processing */
	xTaskCreate(vTaskProcessData, // pvTaskCode
	        (signed portCHAR *) "CC2500",// pcName
	        1024, // usStackDepth
	        &ucParameterToPass, // pvParameters
	        tskIDLE_PRIORITY + 4, // uxPriority
	        &xHandleCC2); // pvCreatedTask

	/* Create task for timing tasks */
	xTaskCreate(vTaskTIMING, // pvTaskCode
	        (signed portCHAR *) "TIMING",// pcName
	        256, // usStackDepth
	        &ucParameterToPass, // pvParameters
	        tskIDLE_PRIORITY + 3, // uxPriority
	        &xHandleTIMING_tsk); // pvCreatedTask

	/* Create task for control tasks */
	xTaskCreate(vTaskCONTROL, // pvTaskCode
			(signed portCHAR *) "CTRL_TSK",// pcName
			256, // usStackDepth
			&ucParameterToPass, // pvParameters
			tskIDLE_PRIORITY + 1, // uxPriority
			&xHandleCONTROL_tsk); // pvCreatedTask
}

void
Spannung_leds(uint16_t spannung)
{
	uint32_t led_on = 0;
	LED_off(LED5og | LED5or | LED6og | LED6or | LED7og | LED8og);
	if (spannung < 3705)
	{
		led_on = LED5or | LED6or;					// Rot Rot
	}
	else if (spannung < 3768)
	{
		led_on = LED5or | LED6og | LED6or;			// Rot Orange
	}
	else if (spannung < 3810)
	{
		led_on = LED5or | LED5og | LED6or | LED6og;	// Orange Orange
	}
	else if (spannung < 3873)
	{
		led_on = LED5or | LED5og | LED6og;			// Orange Grün
	}
	else if (spannung < 3956)
	{
		led_on = LED5og | LED6og;					// Grün Grün
	}
	else if (spannung < 4063)
	{
		led_on = LED5og | LED6og | LED7og;			// Grün Grün Grün
	}
	else
	{
		led_on = LED5og | LED6og | LED7og | LED8og;	// Grün Grün Grün Grün
	}
	LED_on(led_on);
}

void
rssi_leds(int16_t rssi)
{
  uint32_t led_on = 0;
  LED_off(LED5ug | LED5ur | LED6ug | LED6ur | LED7ug | LED8ug);
  if (rssi < -90)
    {
      led_on = LED5ur | LED6ur;						// Rot Rot
    }
  else if (rssi < -85)
    {
      led_on = LED5ur;								// Rot
    }
  else if (rssi < -83)
    {
      led_on = LED5ur | LED6ug | LED6ur;			// Rot Orange
    }
  else if (rssi < -78)
    {
      led_on = LED5ur | LED5ug | LED6ur | LED6ug;	// Orange Orange
    }
  else if (rssi < -73)
    {
      led_on = LED5ur | LED5ug;						// Orange
    }
  else if (rssi < -60)
    {
      led_on = LED5ug;								// Grün
    }
  else if (rssi < -50)
    {
      led_on = LED5ug | LED6ug;						// Grün Grün
    }
  else if (rssi < -40)
    {
      led_on = LED5ug | LED6ug | LED7ug;			// Grün Grün Grün
    }
  else
    {
      led_on = LED5ug | LED6ug | LED7ug | LED8ug;	// Grün Grün Grün Grün
    }
  LED_on(led_on);
}

void
vTaskProcessData(void * pvParameters)
{
	static uint16_t temp_counter;


	/* disable streaming at startup */
	app_control.streaming = 0;

	
	/* wait until usb is ready */

	//    vTaskDelay( 2000 );  // ?????????????????????????????????????????????????
	//   while(!Is_device_enumerated());

	/* Print welcome message */
	//   debug_printf( ">\r\n>>> pro-micron READ-SPIKE <<<\r\n\n" );
	//   debug_printf( "> Init Usart... ok \r\n" );

	/* Load configuration from eeprom */
	eeprom_load_config();


	/* enable 24V Input Ports IN0 and IN1 */
	ENABLE_IN0;
	//   debug_printf( "> IN0 enabled\r\n" );

	/* Init RTC; 4KHz = 0.000125s = 125us */
	if (!rtc_init(&AVR32_RTC, RTC_OSC_32KHZ, 0)) // PSEL = 2; f=2^-(PSEL+1)*32KHz
	{
		//       debug_printf( "> Error initializing the RTC\r\n" );
	}
	/* Set top value to max value  */
	rtc_set_top_value(&AVR32_RTC, 0x40000000);


	/* Enable the RTC */
	rtc_enable(&AVR32_RTC);


	/* Set default-value for moving average filter */
	app_control.moving_average_size = 1;


	/* Init AT86RF231 Radio Transceiver */
	init_at86rf231();

	vTaskDelay(1000);

	/* LED RED 1 ON */
	LED_GREEN1_OFF;
	LED_RED1_ON;
	LED_RED2_OFF;
	LED_GREEN2_OFF;
	LED_off(0x003FFFFF);

	/* Set payload pointer */
	spike_payload_ptr = (spike_payload*) (rx_frame.at86rf231_frame.payload);


	/* Set Transceiver mode = Receive with auto ack */
	at86rf231_rx_with_auto_ack();


	/* Init LED multiplexer */
	tca6424_init();
	
	/* main application loop */
	while (1)
	{
		/* Wait for radio receive semaphore */
		while (xSemaphoreTake(xSem_radio_receive, 0xFFFF) != pdTRUE);

		/* LED GREEN 1 ON */
		LED_GREEN1_ON;
		LED_RED1_OFF;


		/* config packet received? */
		if ((rx_frame.at86rf231_frame.frame_control_field & 7) == FCF_FRAMETYPE_MAC_CMD)
		{
			handle_config_packet();
		}


		// check if received packet is a data-packet
		else if (((rx_frame.at86rf231_frame.frame_control_field & FCF_FRAMETYPE_DATA) == FCF_FRAMETYPE_DATA))
		{
			/* One more packet received */
			app_control.rec_total++;
			app_control.timer50m = 0;


			/* frame missing? */
			if (((bswap_16(spike_payload_ptr->counter) - temp_counter) > 1) && power_on_cnt > NMB_IGNORE_FIRST_FRAMES)
			{
				LED_GREEN1_OFF;
				LED_RED1_ON;

				app_control.frames_missing = app_control.frames_missing + (bswap_16(spike_payload_ptr->counter) - temp_counter) - 1;
			}
			temp_counter = bswap_16(spike_payload_ptr->counter);

			rssi_leds(rf231_rec_info.rssi_dbm);
			//spannung=(float) (spike_payload_ptr->voltage) / 29.15 * 1000;
			spannung = (float) (spike_payload_ptr->voltage) / 47.5 * 1000;
			Spannung_leds(spannung);

			//temperature = bswap_16(spike_payload_ptr->temperature);

			temperature = (float)((int16_t)bswap_16(spike_payload_ptr->temperature)) / 128.0;
			
			WZH_address = bswap_16(rx_frame.at86rf231_frame.src_addr);
			empfang = 1;


			/* Streaming-Mode 3, all data */
			if (app_control.crc_error == 0 && app_control.streaming == 3)
			{
				output_mode3();
			}


			//			usb_uart_printf( "TEST \r\n" );

			/* Auto zero on power up */
			if (power_on_cnt <= NMB_IGNORE_FIRST_FRAMES)
			{
				//zero_strain_gauges();
				power_on_cnt++;
				app_control.rec_total = 0;
				app_control.error_crc_cnt = 0;
				app_control.frames_missing = 0;


				/* Reset moving average */
				index_average = 0;
				avg_values_result[0] = 0;
				avg_values_result[1] = 0;
				avg_values_result[2] = 0;
				avg_values_result[3] = 0;
				avg_values_result[4] = 0;
			}
		}
		/* unknown packet type */
		/*		else
		 {
		 //debug_printf( "\r\nPaket ist weder Daten noch Konfigurationspaket!!!: %d \r\n", (rx_frame.at86rf231_frame.frame_control_field & FCF_FRAMETYPE_DATA ) );
		 }*/
	}
}

void
handle_config_packet(void)
{
	uint16_t temperature;


	/* Build header for config answer packet */
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

	vTaskDelay(1);


	/* Which command was received? */
	switch (rx_frame.at86rf231_frame.payload[0])
	{
	/* CMD: REQ_CFG */
	case 0x04:
		/* receive with auto ack mode */
		at86rf231_rx_with_auto_ack();
		break;


		/* CMD: ACK_REQ, answer and ack for request received */
	case 0x0B:
		//            usb_uart_printf("\r\n\r\n"
		//                            "CMD: ACK_REQ reveived,"
		//                            "\r\nPayload: CMD: %02x  \r\n"
		//                            "Param1: %02x \r\n"
		//                            "Param2: %02x \r\n"
		//                            "Param3: %02x \r\n"
		//                            "Param4: %02x\r\n\r\n",
		//                            rx_frame.at86rf231_frame.payload[0],
		//                            rx_frame.at86rf231_frame.payload[1],
		//                            rx_frame.at86rf231_frame.payload[2],
		//                            rx_frame.at86rf231_frame.payload[3],
		//                            rx_frame.at86rf231_frame.payload[4]
		//             );

		/* Which command was received? */
		switch (rx_frame.at86rf231_frame.payload[1])
		{
		/* READ EEPROM ANSWER */
		case 0x11:
			usb_uart_printf("EEPROM Value: %02x \r\n", rx_frame.at86rf231_frame.payload[2]);
			break;


			/* CMD: Version ANSWER  */
		case 0x15:
			usb_uart_printf("\r\nHW-Version: 0x%04x \r\n", (rx_frame.at86rf231_frame.payload[3] << 8) + rx_frame.at86rf231_frame.payload[2]);
			usb_uart_printf("SW-Version: 0x%04x \r\n", (rx_frame.at86rf231_frame.payload[5] << 8) + rx_frame.at86rf231_frame.payload[4]);
			usb_uart_printf("Serialnumber: 0x%04x \r\n", (rx_frame.at86rf231_frame.payload[7] << 8) + rx_frame.at86rf231_frame.payload[6]);
			break;
			
		case 0x14:
			usb_uart_printf("\r\nSX8723c %02i register: \r\n", rx_frame.at86rf231_frame.payload[2]);
			usb_uart_printf("0x30: %02x \r\n", rx_frame.at86rf231_frame.payload[3]);
			usb_uart_printf("0x40: %02x \r\n", rx_frame.at86rf231_frame.payload[4]);
			usb_uart_printf("0x42: %02x \r\n", rx_frame.at86rf231_frame.payload[5]);
			usb_uart_printf("0x43: %02x \r\n", rx_frame.at86rf231_frame.payload[6]);
			usb_uart_printf("0x52: %02x \r\n", rx_frame.at86rf231_frame.payload[7]);
			usb_uart_printf("0x53: %02x \r\n", rx_frame.at86rf231_frame.payload[8]);
			usb_uart_printf("0x54: %02x \r\n", rx_frame.at86rf231_frame.payload[9]);
			usb_uart_printf("0x55: %02x \r\n", rx_frame.at86rf231_frame.payload[10]);
			usb_uart_printf("0x56: %02x \r\n", rx_frame.at86rf231_frame.payload[11]);
			usb_uart_printf("0x57: %02x \r\n", rx_frame.at86rf231_frame.payload[12]);
			usb_uart_printf("0x70: %02x \r\n", rx_frame.at86rf231_frame.payload[13]);
			
			break;


			/* CMD: Read temperature */
		case 0x30:
			temperature = (rx_frame.at86rf231_frame.payload[3] << 8) + rx_frame.at86rf231_frame.payload[2];
			usb_uart_printf("\r\ntemperature: %3.3f\r\n", ((float) temperature) / 128.0);
			break;


			/* CMD: ZERO Spike */
		case 0x31:
			// handled by uint16_t zero_strain_gauges( void )
			// usb_uart_printf("ZERO SPIKE OK handle config packet\r\n");
			break;


			/* Answer not defined; output complete array */
		default:
			//                     for(temp = 0; temp <= 127; temp++)
			//                     {
			//                        usb_uart_printf("%02X ",rx_frame.at86rf231_frame.payload[temp]);
			//                        if ( temp == 15 || temp == 31 || temp == 47 || temp == 63 || temp == 79 || temp == 95 || temp == 111)
			//                            usb_uart_printf("\r\n");
			//                     }
			//                    usb_uart_printf("\r\n");

			break;
		}


		/* receive with auto ack mode */
		at86rf231_rx_with_auto_ack();
		break;


		/* CMD: ERROR_MSG */
	case 0xFF:

		/* Print Error Message to usart */
		usb_uart_printf("\r\n\r\nCMD: 0xFF ERROR_MSG:  ERROR_CODE: %02x \r\n", rx_frame.at86rf231_frame.payload[1]);


		/* receive with auto ack mode */
		at86rf231_rx_with_auto_ack();
		break;

	default:
		break;
	}
}

void
vTaskTIMING(void * pvParameters)
{
	static portTickType xLastWakeTime_50m;
	const portTickType xFrequency = 42;


	/* reset stat */
	app_control.rec_total = 0;
	app_control.error_crc_cnt = 0;
	app_control.timer50m = 0;

	vTaskDelay(1000);

	while (1)
	{
		/* Initialise the xLastWakeTime variable with the current time. */
		xLastWakeTime_50m = xTaskGetTickCount();


		/* delay time */
		vTaskDelayUntil(&xLastWakeTime_50m, xFrequency);


		/* Timer1++ 50ms delayed */
		app_control.timer50m++;


		/* Taster gedrückt? */
		if (!GET_IN0)
		{
			app_control.streaming = 0;
			vTaskDelay(6);

			if (!empfang) // ohne Empfang Sender suchen
			{
				find_channel();
				vTaskDelay(200);
			}
			else
			{
				// send spike-zero command and receive answer
				zero_strain_gauges();
			}
			app_control.streaming = 3;

		}

		if (app_control.timer50m > 100)
		{
			/* Reset statistics if new data becomes available after */
			power_on_cnt = 0;
		}
		else if (app_control.timer50m > 2)
		{
			/* LED RED 1 ON */
			LED_GREEN1_OFF;
			LED_RED1_ON;

			LED_off(LED5ug | LED5ur | LED6ug | LED6ur | LED7ug | LED8ug); // RSSI-Anzeige aus
			LED_off(LED5og | LED5or | LED6og | LED6or | LED7og | LED8og); // Spannung-Anzeige aus
			empfang = 0;
		}


		// Ausgabe an LEDs
		tca6424_output(LEDs);

	}

}

void
vTaskCONTROL(void * pvParameters)
{
	static portTickType xLastWakeTime;
	const portTickType xFrequency = 200;

	pwm_disable();
	frequency = (uint32_t)(66000000/ptr_flash_data->pwm_freq);
	pwm_drv_init(ptr_flash_data->pwm_freq);
	pwm_enable();

	while(1)
	{
		// Initialise the xLastWakeTime variable with the current time.
		xLastWakeTime = xTaskGetTickCount();

		// delay time
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		if(gpio_get_gpio_pin_output_value(PWM_TRANSCEIVER_ENABLE))	// wenn Treiber aktiv
		{
			// PWM Ausgabe
			pwm_drv_init(ptr_flash_data->pwm_freq);
		}
	}
}

//-----------------------------------------------------------------------------------------------//
// pwm, dac, eic, pdca functions
//-----------------------------------------------------------------------------------------------//
void
pwm_drv_init(unsigned int frequency)
{
	pwm_opt_t pwm_opt; // PWM option config.
	avr32_pwm_channel_t pwm_channel =
		{
			{ 0 }, 0, 0, 0, 0 }; // One channel config.
	unsigned int channel_id;

	channel_id = COIL_PWM_CHANNEL_ID;
	gpio_enable_module_pin(COIL_PWM_PIN, COIL_PWM_FUNCTION);


	// PWM controller configuration.
	pwm_opt.diva = AVR32_PWM_DIVA_CLK_OFF;
	pwm_opt.divb = AVR32_PWM_DIVB_CLK_OFF;
	pwm_opt.prea = AVR32_PWM_PREA_MCK;
	pwm_opt.preb = AVR32_PWM_PREB_MCK;

	pwm_init(&pwm_opt);

	pwm_channel.CMR.calg = PWM_MODE_LEFT_ALIGNED; // Channel mode.
	pwm_channel.CMR.cpol = PWM_POLARITY_LOW; // Channel polarity.
	pwm_channel.CMR.cpd = PWM_UPDATE_DUTY; // Not used the first time.
	pwm_channel.CMR.cpre = AVR32_PWM_CPRE_MCK; // Channel prescaler.
	pwm_channel.cprd = frequency; // Channel period.
	pwm_channel.cdty = pwm_channel.cprd / 2; // Channel duty cycle, should be < CPRD.
	pwm_channel.cupd = 0; // Channel update is not used here.

	/* With these settings, the output waveform period will be :
	 (115200/256)/20 == 22.5Hz == (MCK/prescaler)/period, with MCK == 115200Hz,
	 prescaler == 256, period == 20. */
	pwm_channel_init(channel_id, &pwm_channel); // Set channel configuration to channel 0.
	//pwm_start_channels( 1 << channel_id );            // Start channel 0.
}

void
pwm_enable(void)
{
	/* Enable Transceiver */
	gpio_set_gpio_pin(PWM_TRANSCEIVER_ENABLE);
	pwm_start_channels(1 << COIL_PWM_CHANNEL_ID);
}

void
pwm_disable(void)
{
	/* Disable Transceiver */
	gpio_clr_gpio_pin(PWM_TRANSCEIVER_ENABLE);
	pwm_stop_channels(1 << COIL_PWM_CHANNEL_ID);
}

void
eeprom_save_config(void)
{
	uint16_t i = 0;
	uint8_t *ptr_flash_data8 = (uint8_t *) &struct_flash_data;
	taskENTER_CRITICAL();
	for (i = 0; i < sizeof(flash_data); i++)
	{
		flashc_memset8((unsigned char *) (USER_PAGE_END_ADDR - sizeof(flash_data) + i), *(ptr_flash_data8 + i), 1, TRUE);
	}
	taskEXIT_CRITICAL();
}

void
eeprom_load_config(void)
{
	uint16_t i = 0;
	uint8_t *ptr_eeprom_data8 = (uint8_t *) &struct_flash_data;

	if (*((uint8_t*) USER_PAGE_END_ADDR - sizeof(flash_data)) != 0xFF)
	{
		for (i = 0; i < sizeof(flash_data); i++)
		{
			*(ptr_eeprom_data8 + i) = *(uint8_t*) (USER_PAGE_END_ADDR - sizeof(flash_data) + i);
		}
	}
}

void
output_mode3(void)
{
	static uint8_t ts = 0;
	static uint16_t string_pos1 = 0, string_pos2 = 0;
	static uint8_t first_output = 0;
	double global_time = 0;
	static double global_time2 = 0;
	//static uint16_t counter, counter_old;

	//static int32_t biege_betrag1, biege_betrag2, biege_betrag3=0;

	/* get time */
	//global_time = rtc_get_value(&AVR32_RTC);
	
	if(reset_timer)
	{
		reset_timer = false;
		ptr_over_flow = 0;
		ptr_offset = bswap_16(spike_payload_ptr->counter);
		global_time2 = 0;
	}

	/* Moving Average Filter */
	for (ts = 0; ts < SAMPLES_PER_PACKET; ts++)
	{
		/* No moving average in use */
		if (app_control.moving_average_size == 1)
		{
			avg_values_result2[0] = (int16_t) bswap_16(spike_payload_ptr->strain_gauge[ts][0]);
			avg_values_result2[1] = (int16_t) bswap_16(spike_payload_ptr->strain_gauge[ts][1]);
			avg_values_result2[2] = (int16_t) bswap_16(spike_payload_ptr->strain_gauge[ts][2]);
			avg_values_result2[3] = (int16_t) bswap_16(spike_payload_ptr->strain_gauge[ts][3]);
		}
		else
		{
			avg_values_result[0] -= avg_values[0][index_average];
			avg_values[0][index_average] = (int16_t) bswap_16(spike_payload_ptr->strain_gauge[ts][0]);
			avg_values_result[0] += avg_values[0][index_average];
			avg_values_result2[0] = avg_values_result[0] / app_control.moving_average_size;
 
			avg_values_result[1] -= avg_values[1][index_average];
			avg_values[1][index_average] = (int16_t) bswap_16(spike_payload_ptr->strain_gauge[ts][1]);
			avg_values_result[1] += avg_values[1][index_average];
			avg_values_result2[1] = avg_values_result[1] / app_control.moving_average_size;

			avg_values_result[2] -= avg_values[2][index_average];
			avg_values[2][index_average] = (int16_t) bswap_16(spike_payload_ptr->strain_gauge[ts][2]);
			avg_values_result[2] += avg_values[2][index_average];
			avg_values_result2[2] = avg_values_result[2] / app_control.moving_average_size;

			avg_values_result[3] -= avg_values[3][index_average];
			avg_values[3][index_average] = (int16_t) bswap_16(spike_payload_ptr->strain_gauge[ts][3]);
			avg_values_result[3] += avg_values[3][index_average];
			avg_values_result2[3] = avg_values_result[3] / app_control.moving_average_size;


			/* Index erhöhen */
			index_average++;
			index_average %= app_control.moving_average_size;
		}

		/* Save system time */
		if (ts == 0)
		{
			/*
			counter = bswap_16(spike_payload_ptr->counter);
			global_time2 = ( counter - counter_old -1 ) * 4375 + 625;
			counter_old = counter;
			*/
			
			global_time = (double)((double)(bswap_16(spike_payload_ptr->counter) - ptr_offset) * 4375.0) + (286720000.0 *  (double)ptr_over_flow);
			
			//usb_uart_printf( "\r\nglobal_time: %3.0f \r\nglobal_time2: %3.0f", global_time, global_time2);
			
			if( global_time < global_time2 )
			{
				ptr_over_flow++;
				global_time = (double)((double)(bswap_16(spike_payload_ptr->counter) - ptr_offset) * 4375.0) + (286720000.0 *  (double)ptr_over_flow);
			}
			
			global_time2 = global_time;
		}
		else
		{
			global_time2 += 625; //ank
			//global_time2 += 571; ank
			//global_time2 = 0;
		}
		
		/* build output string */
		string_pos1 = sprintf((char*) (fast_usb_string) + string_pos2, "\r\n%+06d;%+06d;%+06d;%+06d;%3.0f;%+06d;%04d;%03d;%2.3f", (int16_t) avg_values_result2[0], (int16_t) avg_values_result2[1], (int16_t) avg_values_result2[2], (int16_t) avg_values_result2[3], global_time2, bswap_16(spike_payload_ptr->counter), spannung, rf231_rec_info.rssi_dbm, temperature);
//		string_pos1 = sprintf((char*) (fast_usb_string) + string_pos2, "\r\n%+06d;%+06d;%+06d;%+06d;%3.0f;%+06d;%04d;%03d", (int) avg_values_result2[0], (int) avg_values_result2[1], (int) avg_values_result2[2], (int) avg_values_result2[3], global_time2, bswap_16(spike_payload_ptr->counter), spannung, rf231_rec_info.rssi_dbm);
		/* next position to write */
		string_pos2 += string_pos1;
	}
	/* terminate string */
	*((char*) (fast_usb_string) + string_pos2) = 0x00;
	*((char*) (fast_usb_string) + string_pos2 + 1) = 0x00;
	*((char*) (fast_usb_string) + string_pos2 + 2) = 0x00;
	*((char*) (fast_usb_string) + string_pos2 + 3) = 0x00;
	string_pos1 = 0;
	string_pos2 = 0;

	if (first_output == 0)
	{
		index_average = 0;
		avg_values_result[0] = 0;
		avg_values_result[1] = 0;
		avg_values_result[2] = 0;
		avg_values_result[3] = 0;
		avg_values_result[4] = 0;

		first_output = 1;
	}
	else
	{
//		if(((int)global_time2 % 15876) == 0)
//		{
			//if( strlen((char*)fast_usb_string) > 500 )
			//{
			/* Falls mehr performance benötigt, stringlänge erhöhen, dafür funktion weniger oft aufrufen */
			if (power_on_cnt > NMB_IGNORE_FIRST_FRAMES)
			{
				//if ((spike_payload_ptr->counter % 228) == 0 )
				//{ //ank
					usb_cdc_performance_print((int32_t*) fast_usb_string, strlen((char*) fast_usb_string));
				//}
			}
			//}
//		}
	}
}

uint16_t
zero_strain_gauges(void)
{
	uint16_t timeout = 0;
	uint16_t ret_val = 0;
	uint16_t x, y = 0;


	/* wait for values from radio */
	vTaskDelay(5);

	power_on_cnt = 0;


	/* Reset moving average */
	index_average = 0;
	avg_values_result[0] = 0;
	avg_values_result[1] = 0;
	avg_values_result[2] = 0;
	avg_values_result[3] = 0;
	avg_values_result[4] = 0;

	for (x = 0; x < 5; x++)
	{
		for (y = 0; y < 100; y++)
		{
			avg_values[x][y] = 0;
		}
	}

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

	tx_frame.at86rf231_frame.payload[0] = 0x31; // CMD
	tx_frame.at86rf231_frame.payload[1] = 0x00; // Param1
	tx_frame.at86rf231_frame.payload[2] = 0x00; // Param2

	/* Tx frame with zero command */
	vTaskDelay(31);
	send_frame((uint8_t*) &tx_frame, CSMA_UNSLOTTED, 1);
	at86rf231_rx_with_auto_ack();


	/* wait 3 seconds for confirmation */
	while (timeout < 4000)
	{
		LED_RED1_ON;
		LED_GREEN1_OFF;

		vTaskDelay(1);
		timeout++;

		if (((rx_frame.at86rf231_frame.frame_control_field & 7) == FCF_FRAMETYPE_MAC_CMD) && (rx_frame.at86rf231_frame.payload[0] == 0x0B) && (rx_frame.at86rf231_frame.payload[1] == 0x31))
		{
			ret_val = timeout;
			break;
		}
	}

	/* No ACK received? */
	if (pal_trx_bit_read(SR_TRAC_STATUS) == TRAC_NO_ACK)
	{
		ret_val = FALSE;
	}

	/* No confirmation received in timeout */
	if (timeout >= 4000)
	{
		ret_val = FALSE;
	}

	LED_RED1_OFF;
	LED_GREEN1_ON;

	return ret_val;
}

