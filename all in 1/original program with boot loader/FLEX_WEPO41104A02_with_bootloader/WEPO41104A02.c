//-----------------------------------------------------------------------------------------------//
/**
 * \mainpage Project SPIKE 1.1 - Description
 *
 * -  Project: WEPO
 * -  Custumor:
 *
 * This manual is divided into the following sections:
 * - \subpage intro   
 * - \subpage program_structure
 * - \subpage version_history
 *
 * <b>Target Platform:</b>	ATmega324PV
 *
 * <b>Editor:</b>			Atmel AVR Studio 5 (Version: 5.1.208)
 *
 * <b>Compiler:</b>         AVRGCC - Version: 3.3.1.27
 *
 * \version WEPO41104A02
 * \date 2013-09-11 (last change)
 * \author flm, mih, job, ank
 *
 * \todo edit introduction 
 * \bug 
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 */

/**
\page intro Introduction

<H2>Ein-/Ausschaltverhalten</H2>

- Während des Ladevorgangs: Aus
- Radialbeschleunigung steigt über ca. 2g: Ein
- solange Radialbeschleunigung über ca. 2g: Ein
- Einstellbare Zeit (z.B. 1 min.) nachdem die Radialbeschleunigung unter ca. 2g gefallen ist: Aus

<H2>Funktransceiver AT86RF231</H2>

- Konfigurationen werden zwischen Messwertübertragungen empfangen
- Konfigurationspakete werden bestätigt und bei Bedarf erneut übertragen
- Folgende Kommandos werden im FULL_CONFIG_MODE (MAKRO) unterstützt:
          -REQ_STREAM
          -REQ_CFG
          -STOP_STREAM
          -READ_EEPROM
          -WRITE_EEPROM
          -WRITE_ZMD_CFG
          -READ_ZMD_CFG
          -READ_VERSION
          -SET_TIMEOUT
          -SET_DATARATE
          -C_RADIO_CH
          -C_DATARATE_MODE
          -NEW_SRC_ADDRESS
          -NEW_DEST_ADDRESS
          -NEW_PANID
          -NEW_TX_PWR
          -READ_TEMPERATURE
          -OFFSET
          -Read calibration values
           -Write calibration values

<H2>Konfigurations Daten</H2>

Die Konfigurationsdaten für den SPIKE werden im EEPROM gespeichert. Der Inhalt des EEPROMS ist in der struct_eeprom_data (eeprom_abbild.h) abgelegt.

<H2>Semtech SX8723c</H2>

- [EDIT] Offsetabgleich für DMS Messwerte geschieht über ZMD-Analogoffset, ZMD-Digitaloffset und
  einem zusätzlichen digitalen Offset (digi_offset)

\page program_structure Program Structure
\image html Flex-Programmablauf_FW-Version_0x0003.png "Flex-Programmablauf"

\page version_history Version History

    <TABLE>
    <TR>
     <TD><b>Date </b></TD>
     <TD><b>Change </b></TD>
     <TD><b>Author </b></TD>
    </TR>
    <TR>
     <TD>2010-03-24</TD>
     <TD>Erste stabile getestete Version, FULL_CONFIG_MODE noch nicht komplett getestet</TD>
     <TD>flm</TD>
    </TR>
	<TR>
     <TD>2010-03-26</TD>
     <TD>ADC Prescaler von 2 auf 32 geändert --> Spannungsanzeige deutlich genauer</TD>
     <TD>flm</TD>
    </TR>
	    <TR>
     <TD>2010-04-06</TD>
     <TD>Als sourceaddress wird die Seriennummer verwendet</TD>
     <TD>flm</TD>
    </TR>
	    <TR>
     <TD>2010-04-06</TD>
     <TD> Überprüfung ob ob Ziel und Empfängeradressen stimmen</TD>
     <TD>flm</TD>
    </TR>
	    <TR>
     <TD>2010-04-09</TD>
     <TD>Pullup für Charge_stat aktiviert</TD>
     <TD>flm</TD>
    </TR>
	    <TR>
     <TD>2010-04-12</TD>
     <TD>LED-Ansteuerung hinzugefügt. LED leuchtet während ZMD auslesen</TD>
     <TD>flm</TD>
    </TR>
	    <TR>
     <TD>2012-08-07</TD>
     <TD>Temperaturmessung mit 1 Hz, wenn gerade ZMD nicht auslesen. Auflösung auf 1/16 °C (13 Bit)</TD>
     <TD>job</TD>
    </TR>
	<TR>
     <TD>2013-03-20</TD>
     <TD>
		- Portierung auf AVRStudio 5
 		- eeprom_abbild.h eingebunden
		- Einstellungen in SPIKE_config.h ausgelagert
	 </TD>
     <TD>ank</TD>
    </TR>
    <TR>
     <TD>2013-03-21</TD>
     <TD>
	 - Dokumentation für Doxygen angepasst
	 - Software-TWI Funktionen in eingene Dateien (sw_twi.c, sw_twi.h)
	 - Temperatur Sensor ds620 Funktionen in eingene Dateien (ds620.c, ds620.h)
	 - PM config Protokoll in eingene Dateien (pm_config_protocol.c, pm_config_protocol.h)
	 - WEPO41104A00 header Datei hinzugefügt (WEPO41104A00.h)
	 </TD>
     <TD>ank</TD>
    </TR>
	<TR>
     <TD>2013-03-22</TD>
     <TD>
	 - EEPROM Funktionen und Abbild in eingene Dateien (eeprom_abbild.c, eeprom_abbild.h)
	 - ZMD Funktionen aus Projekt entfernt
	 - EEPROM Abbild auf SX8723C angepasst (eeprom_abbild.c, eeprom_abbild.h)
	 - strukt eeprom_data: digi_offset durch SG_software_offset ersetzt (eeprom_abbild.c, eeprom_abbild.h)
	 - Als scr_address für Funk Protokoll wird jetzt wieder scr_address verwendet anstatt serial_number (eeprom_abbild.c, WEPO41104A00.c, SPIKE_config.h, at86rf231.c, pm_config_protocol.c)
	 </TD>
     <TD>ank</TD>
    </TR>
	<TR>
     <TD>2013-04-10</TD>
     <TD>
	 - Fehler in Funktion ds620_get_temperature() behoben, als Temperaturwert wurde 128°C angezeigt (ds620.c, ds620.h)
	 - TWI Geschwindigkeit des Temperatursensors auf 400 kHz erhöht (ds620.c)
	 - Fehler behoben: Temperatursensor kann keine negativen Werte anzeigen (ds620.c, ds620.h)
	 - Auflösung des Temperatursensors auf 12 Bit verringert (ds620.c)
	</TD>
     <TD>ank</TD>
    </TR>
	<TR>
     <TD>2013-04-11</TD>
     <TD>
	 - LED Funktionen neu definiert (WEPO41104A00.h, WEPO41104A00.c)
	 - Software TWI neu programmiert (sw_twi.c, sw_twi.h, conf_sw_twi.h)
	</TD>
     <TD>ank</TD>
    </TR>
	<TR>
     <TD>2013-04-12</TD>
     <TD>Software TWI neu programmiert (sw_twi.c, sw_twi.h, conf_sw_twi.h)
	</TD>
     <TD>ank</TD>
    </TR>
	<TR>
     <TD>2013-04-15</TD>
     <TD>SX8723c Treiber hinzugefügt und auf Atmega324P angepasst (SX8723c.h, SX8723c.c)</TD>
     <TD>ank</TD>
    </TR>
	<TR>
     <TD>2013-04-16</TD>
     <TD>
	 - samlerate auf 1600 Hz reduziert
	 - cbi() und sbi() entfernt, neue schreibweise verwendet (WEPO41104A00.c, ds620.c, ds620.h)
	 - Zeitpunkt zum Auslesen des Temperatursensors geändert und auf varible samplerate angepasst (WEPO41104A00.c)
	 - "temperature_flag" entfernt da nicht mehr nötig (WEPO41104A00.c)
	 - Auslesen des Temperatursensors auf 5 Hz erhöht
	 </TD>
     <TD>ank</TD>
    </TR>
	
	<TR>
     <TD>2013-04-22</TD>
     <TD>Software TWI timing angepasst (sw_twi.c)</TD>
     <TD>ank</TD>
    </TR>
	
    </TR>
    <TR>
     <TD>2013-07-01</TD>
     <TD>
	 - Neue Version ist WEPO41104A01
	 - Fehler: Beim Offsetabgleich der Brücken kommt es zu einem Sprung eines Messwertes; 
	   Lösung: Befehle werden erst ausgewertet wenn das Paket vollständig geschrieben und verschickt ist
	 </TD>
	 <TD>ank</TD>
	 </TR>
	 
	 <TR>
     <TD>2013-07-08</TD>
     <TD>
	 Hauptprogramm Schleife als Zustandsautomat
	 </TD>
     <TD>ank</TD>
    </TR>
	
	<TR>
     <TD>2013-07-22</TD>
     <TD>
	 Abschaltschwelle für die Akku-Spannung von 3,5V auf 3,6V erhöht
	 </TD>
     <TD>ank</TD>
    </TR>
	
	<TR>
     <TD>2013-09-09</TD>
     <TD>
	 Fehler in Software TWI bei SET_SCL in while Schleife korrigiert 
	 </TD>
     <TD>ank</TD>
    </TR>
	
   </TABLE>
   
	<b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>    
 */
//-----------------------------------------------------------------------------------------------//

#include "inc/WEPO41104A02.h"
#include "inc/Bootloader.h"

int main(void)
{	
	cli();								// clear interrupt
		
	avr_init();							// controller and sensor initialization 
	
	int16_t SG_value[4] = {0};			// value of the strain gauges
	int16_t *p_SG_value = SG_value;		// pointer to value

	current_state = STATE_INIT;			// set current state

	/*--------------- State Machine ----------------------------------------------*/
	while(1)
	{
		switch(current_state)
		{
			/*---------------- INIT ----------------*/
			case STATE_INIT:
			
				TCCR1B = 0x00;	// stop timer; TODO nötig?
				TIFR1 = 0x02;	// Output Compare A Match Flag reset; TODO nötig?
				
				// SX8723c and strain gauges off
				SX8723c_sleep(device_all);

				#if defined(USE_DS620)
					// temperature sensor off
					ds620_stop_convert(); 
				#endif
	
				// ADC off
				ADCSRA = 0x00; 
	
				// Transceiver in Power-down
				at86rf231_sleep(); 

				current_state = STATE_SLEEP;
			break;
			
			/*---------------- SLEEP ----------------*/
			case STATE_SLEEP:
			
				 set_sleep();
				
				current_state = STATE_WAKE_UP;
			break;
			
			/*---------------- WAKE_UP ----------------*/
			case STATE_WAKE_UP:

				// charging?
				if (PIND & 0x20)
				{
					current_state = STATE_SLEEP;
					break;
				}
				
				// power reduction register
				PRR0 = 0x00;	// enable all modules
				
				// battery empty?
				init_adc();
				
				_delay_ms(1);
				
				if (ADCH <= (BATTERY_THRESHOLD_EMPTY+2) ) // +2 hysteresis
				{
					current_state = STATE_SLEEP;
					break;
				}
	
				// Set system clock prescaler 
				CLKPR = 0x80;
				CLKPR = 0x00;	// prescaler = 1
				
				current_state = STATE_MEASUREMENT_PREPARE;
			break;
			
			/*---------------- MEASUREMENT_PREPARE ----------------*/
			case STATE_MEASUREMENT_PREPARE:
			
				cli();

				#if defined(USE_DS620)
					ds620_start_convert(); 
				#endif

				// Start SX8723c an turn strain gauge power on
				SX8723c_start(device_all);
	
				// reset time_slot to avoid sending old data
				timeslot_number = 0; 
				timeout_timer = 0;
	
				// start timer
				TCCR1B = 0x09;	// CTC, no prescaler
				TIFR1 = 0x02;	// clear output compare a match flag
	
				sei();
				
				current_state = STATE_MEASUREMENT;
			break;
			
			/*---------------- MEASUREMENT ----------------*/
			case STATE_MEASUREMENT:
			
				if (timer_flag)
				{
					timer_flag = 0;
			
					// increase timeout_timer
					timeout_timer++;
			
					#if defined(USE_DS620)
					// increase temperature timer
					temperature_timer++;
					#endif
			
					LED_ON;											// turn LED ON 
					SX8723c_send_request(device_all);				// Send request to all bridges
					SX8723c_get_value(device_all, p_SG_value);		// Read data from all bridges 
					LED_OFF;										// turn LED of
					
					
					// store data in payload 
					ptr_payload_spike->strain_gauge[timeslot_number][SG_1] = SG_value[SG_1] - ptr_eeprom_sram_data->SG_software_offset[SG_1];
					ptr_payload_spike->strain_gauge[timeslot_number][SG_2] = SG_value[SG_2] - ptr_eeprom_sram_data->SG_software_offset[SG_2];
					ptr_payload_spike->strain_gauge[timeslot_number][SG_3] = SG_value[SG_3] - ptr_eeprom_sram_data->SG_software_offset[SG_3];
					ptr_payload_spike->strain_gauge[timeslot_number][SG_4] = SG_value[SG_4] - ptr_eeprom_sram_data->SG_software_offset[SG_4];
					
					/*
					ptr_payload_spike->strain_gauge[timeslot_number][SG_1] = 11;
					ptr_payload_spike->strain_gauge[timeslot_number][SG_2] = 22;
					ptr_payload_spike->strain_gauge[timeslot_number][SG_3] = 33;
					ptr_payload_spike->strain_gauge[timeslot_number][SG_4] = 44;
					*/
					
					// Time slot++
					timeslot_number++;
					
					#if defined(USE_DS620)
					// get temperature value if 200ms over and time slot number is 3
					if( (temperature_timer >= (eeprom_sram_data.samplerate / 5)) & (timeslot_number == 3))
					{
						temperature = ds620_get_temperature();
						temperature_timer = 0;
					}
					#endif

					// send data with radio if necessary 
					if (timeslot_number >= SAMPLES_PER_PACKET)
					{
						timeslot_number = 0;
						current_state = STATE_SEND;
						break;
					}				
			
				}
				
				// Acceleration about threshold: reset timeout_timer 
				if ((use_acceleration_sensor_flag == 1) && (!(ACSR & 0x20)))
				{
					timeout_timer = 0;
				}
				
				current_state = STATE_MEASUREMENT;
			break;
			
			/*---------------- MEASUREMENT_CLOSE ----------------*/
			case STATE_MEASUREMENT_CLOSE:
			
				TCCR1B = 0x00;	// stop timer
				TIFR1 = 0x02;	// Output Compare A Match Flag reset
	
				// SX8723c and strain gauge off
				SX8723c_sleep(device_all);

				#if defined(USE_DS620)
					// temperature sensor off
					ds620_stop_convert(); 
				#endif
	
				// ADC off
				ADCSRA = 0x00; 
	
				// Transceiver in Power-down
				at86rf231_sleep(); 

				current_state = STATE_SLEEP;
			break;
			
			/*---------------- SEND ----------------*/
			case STATE_SEND:
			
				// send data 
				send_all_data_rf231();
				
				// Received an configuration packet?
				if ( at86rf231_rx_frame.payload[0] != 0x00)
				{
					current_state = STATE_CONFIG;
					break;
				}
				
				// charging?
				if (PIND & 0x20)
				{
					current_state = STATE_MEASUREMENT_CLOSE;
					break;
				}				
				
				// timeout occurred?
				if ((timeout_timer > (unsigned long) eeprom_sram_data.timeout * eeprom_sram_data.samplerate) && use_acceleration_sensor_flag)
				{
					timeout_timer = 0;
					current_state = STATE_MEASUREMENT_CLOSE;
					
					break;
				}
			
				// battery empty?
				if (ADCH <= BATTERY_THRESHOLD_EMPTY)
				{
					current_state = STATE_MEASUREMENT_CLOSE;
					break;
				}
				
				// stop sending data?
				if (stop_stream)
				{
					stop_stream = 0;
					current_state = STATE_MEASUREMENT_CLOSE;
					break;
					
				}				
								
				current_state = STATE_MEASUREMENT;
			break;
			
			/*---------------- CONFIG ----------------*/
			case STATE_CONFIG:
				
				_delay_ms(40); 

				/* Receive and process data */
				pm_config_protocol();
			
				current_state = STATE_MEASUREMENT;
			break;
			
			/*---------------- default ----------------*/
			default:
			break;	
		}
	} 
}

void timer_init(void) 
{
	// timer/counter1
	
	TCCR1A = 0x00; // Timer/Counter1 Control Register 
	TCCR1B = 0x09; // Clear Timer on Compare match (CTC); No clock source (Timer/Counter stopped)
	
	// Output Compare Register 1 A
	OCR1A = (unsigned short) ((float) F_CPU
			/ (float) eeprom_sram_data.samplerate); // set timer to sample rate
	
	TCNT1 = 0x0000;			// set counter to null
	OCR1B = 0x0000;			//  Output Compare Register 1 B
	ICR1 = 0x0000;			// Input Capture Register 1
	TIMSK1 = 1 << OCIE1A;	// Timer/Counter1 Interrupt Mask Register; Output Compare A Match, interrupt enable
	TIFR1 = 0x02;			// Timer/Counter1 Interrupt Flag Register; clear output compare a match flag
}

void avr_init(void) 
{
	MCUCR &= ~(1 << PUD);	// enable internal pull ups 

	PRR0 = 0x00;			// power manager: enable all modules

	CLKPR = 0x80;			// Set system clock prescaler 
	CLKPR = 0x00;			// prescaler to 1

	DDRA = 0x00;
	PORTA = 0x1F;

	DDRB = 0xB1;
	PORTB = 0x50;

	DDRC = 0xC0;
	PORTC = 0x80;

	DDRD = (1 << PIND4) | (1 << PIND7);		// 1 = output, 0 = input 
	PORTD = (1 << PIND4) | ( 1 << PIND7);	// 1 = high, 0 = low 
	
	INIT_LED;				// initialize LED

	eeprom_read_config(ptr_eeprom_sram_data);	// store EEPROM data to SRAM (faster access)

	// if timeout = 0 --> deactivate acceleration sensor, always on 
	if (eeprom_sram_data.timeout == 0) 
	{
		use_acceleration_sensor_flag = 0;
	}
	else
	{
		use_acceleration_sensor_flag = 1;
	}
	
#if defined(USE_DS620)
	ds620_init();		// initialize temperature sensor
#endif

	ACSR = 0x82;		// analog-comparator off, comparator Interrupt on Falling Output Edge
	
	timer_init();		// timer initialization
	
	init_adc();			// ADC initialization 
	
	at86rf231_init();	// AT86RF231 initialization
	
	_delay_ms(10);		// Wait until SX8723c is ready to communicate (start up time)
	
	SX8723c_init(device_1);		// SX8723c initialization (bridge 1)
	SX8723c_init(device_2);		// SX8723c initialization (bridge 2)
	SX8723c_init(device_3);		// SX8723c initialization (bridge 3)
	SX8723c_init(device_4);		// SX8723c initialization (bridge 4)
}

void init_adc(void)
{
	// ADC Multiplexer Selection Register
	ADMUX = (1 << REFS1) | (1 << ADLAR) | (1 << MUX0) | (1 << MUX1) | (1 << MUX2);		// ADC7, Left Adjust, 1.1V Ref
	
	// ADC Control and Status Register A
	ADCSRA = (1 << ADSC) | (1 << ADATE) | (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0);	// ADC enable, Start conversion, AutoTrigger enable, Prescaler: 32
	
	// ADC Control and Status Register B
	ADCSRB = 0;				// Free Running mode
}

void set_sleep(void)
{
	cli();
		
	// change clock prescaler
	CLKPR = 0x80;	// Clock Prescaler Change Enable
	CLKPR = 0x03;	// Prescaler = 8

	// power reduction register
	PRR0 = 0xFE;

	// analog-comparator on, interrupt on falling edge
	ACSR = 0x1A; 
		
	// set CPU sleep
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	sei();
	sleep_cpu(); 
	sleep_disable();
}

void static inline send_all_data_rf231(void)
{
	static volatile uint16_t frame_counter = 0;

	/* Build frame to transmit */
	at86rf231_tx_frame.frame_length = sizeof(spike_payload) + 13;

	at86rf231_tx_frame.frame_control_field
			= FCF_SET_FRAMETYPE( FCF_FRAMETYPE_DATA )
			        | FCF_SET_DEST_ADDR_MODE( FCF_SHORT_ADDR )
					| FCF_SET_SOURCE_ADDR_MODE( FCF_SHORT_ADDR );
	// |FCF_ACK_REQUEST;

	at86rf231_tx_frame.seq_number = (uint8_t) frame_counter & 0xFF;
	at86rf231_tx_frame.dest_pan_id = ptr_eeprom_sram_data->pan_id;
	at86rf231_tx_frame.dest_addr = ptr_eeprom_sram_data->dest_address;
	at86rf231_tx_frame.src_pan_id = ptr_eeprom_sram_data->pan_id;
	at86rf231_tx_frame.src_addr = ptr_eeprom_sram_data->src_address;

	/* Add counter to payload */
	ptr_payload_spike->counter = frame_counter++;

	/* Add temperature to payload */
	ptr_payload_spike->temperature = temperature;

	/* Add voltage to payload */
	ptr_payload_spike->voltage = ADCH; 

	/* tx data */
	at86rf231_tx();
}



//-----------------------------------------------------------------------------------------------//
// Interrupts
//-----------------------------------------------------------------------------------------------//

ISR( TIMER1_COMPA_vect )
{	
	timer_flag = 1;	// set timer flag 
}

ISR( ANALOG_COMP_vect )
{	
	ACSR = 0x12; // analog-comparator on, interrupt off, clear interrupt
}
