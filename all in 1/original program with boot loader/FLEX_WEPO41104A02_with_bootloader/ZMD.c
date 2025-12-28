/*
 * ZMD.c
 *
 * Created: 21.03.2013 11:08:08
 *  Author: andre.kuhn
 */ 

#include "inc/ZMD.h"

///*! \brief Ein ZMD-Kommando auf die Software-I²C schreiben
 //* \param zmd anzusprechender ZMD [ZMD1, ZMD2, ZMD3, ZMD4, ZMD_ALL]
 //* \param cmd Kommando
 //*/
//void zmd_cmd(unsigned char zmd, unsigned char cmd) // ZMD-Kommando senden
//{
	//// Start
	//TWI_start(zmd);
//
	//// Addrese (ZMD)
	//write_byte(zmd, 0xF0);
//
	//// Kommando
	//write_byte(zmd, cmd);
//
	//// Stop
	//TWI_stop(zmd);
//}
//
//void init_zmd(void) // ZMDs Initialisieren
//{
	//unsigned char x = 0, zmd = ZMD1;
	//// Command-Mode (0x72) für alle ZMDs!!!
	//zmd_cmd(ZMD_ALL, 0x72);
	//_delay_ms(50);
//
	//// Konfiguration laden
	//zmd = ZMD1;
	//for (x = 0; x < 32; x++)
	//{
		//_delay_us(50);
		//TWI_start(zmd);
		//_delay_us(50);
		//write_byte(zmd, 0xF0);
		//_delay_us(50);
		//write_byte(zmd, 0x80 + x);
		//_delay_us(50);
		//write_byte(zmd, (unsigned char) (eeprom_flash_data.zmd_config[0][x]
				//>> 8));
		//_delay_us(50);
		//write_byte(zmd, (unsigned char) (eeprom_flash_data.zmd_config[0][x]
				//& 0x00FF));
		//_delay_us(50);
		//TWI_stop(zmd);
		//_delay_us(50);
	//}
//
	//zmd = ZMD2;
	//for (x = 0; x < 32; x++)
	//{
		//_delay_us(50);
		//TWI_start(zmd);
		//_delay_us(50);
		//write_byte(zmd, 0xF0);
		//_delay_us(50);
		//write_byte(zmd, 0x80 + x);
		//_delay_us(50);
		//write_byte(zmd, (unsigned char) (eeprom_flash_data.zmd_config[1][x]
				//>> 8));
		//_delay_us(50);
		//write_byte(zmd, (unsigned char) (eeprom_flash_data.zmd_config[1][x]
				//& 0x00FF));
		//_delay_us(50);
		//TWI_stop(zmd);
		//_delay_us(50);
	//}
//
	//zmd = ZMD3;
	//for (x = 0; x < 32; x++)
	//{
		//_delay_us(50);
		//TWI_start(zmd);
		//_delay_us(50);
		//write_byte(zmd, 0xF0);
		//_delay_us(50);
		//write_byte(zmd, 0x80 + x);
		//_delay_us(50);
		//write_byte(zmd, (unsigned char) (eeprom_flash_data.zmd_config[2][x]
				//>> 8));
		//_delay_us(50);
		//write_byte(zmd, (unsigned char) (eeprom_flash_data.zmd_config[2][x]
				//& 0x00FF));
		//_delay_us(50);
		//TWI_stop(zmd);
		//_delay_us(50);
	//}
//
	//zmd = ZMD4;
	//for (x = 0; x < 32; x++)
	//{
		//_delay_us(50);
		//TWI_start(zmd);
		//_delay_us(50);
		//write_byte(zmd, 0xF0);
		//_delay_us(50);
		//write_byte(zmd, 0x80 + x);
		//_delay_us(50);
		//write_byte(zmd, (unsigned char) (eeprom_flash_data.zmd_config[3][x]
				//>> 8));
		//_delay_us(50);
		//write_byte(zmd, (unsigned char) (eeprom_flash_data.zmd_config[3][x]
				//& 0x00FF));
		//_delay_us(50);
		//TWI_stop(zmd);
		//_delay_us(50);
	//}
//
	//_delay_ms(5);
	//// Generate RAM-Signature (0xCB)
	//zmd_cmd(ZMD_ALL, 0xCB);
	//_delay_ms(10);
	//// Start Cycle (0x02)
	//zmd_cmd(ZMD_ALL, 0x02);
	//_delay_ms(60);
//}
///*! \brief ZMDs auslesen (Messwerte)
 //* \param *data Array, in dem die Messwerte abgelegt werden
 //*/
//void read_zmd(unsigned char *data)
//{
	//unsigned char x = 0;
	//// Bus Idle
	//SET_SDA
	//SET_SCL;
//
	//// Start
	//CLEAR_SDA;
	//CLEAR_SCL;
//
	//// Addrese
	//SET_SDA; // Bit 7
	//SET_SCL;
	//CLEAR_SCL;
	//SET_SDA; // Bit 6
	//SET_SCL;
	//CLEAR_SCL;
	//SET_SDA; // Bit 5
	//SET_SCL;
	//CLEAR_SCL;
	//SET_SDA; // Bit 4
	//SET_SCL;
	//CLEAR_SCL;
	//CLEAR_SDA; // Bit 3
	//SET_SCL;
	//CLEAR_SCL;
	//CLEAR_SDA; // Bit 2
	//SET_SCL;
	//CLEAR_SCL;
	//CLEAR_SDA; // Bit 1
	//SET_SCL;
	//CLEAR_SCL;
	//SET_SDA; // Bit 0
	//SET_SCL;
	//CLEAR_SCL;
//
	//// Acknowledge
	//SET_SDA; // SDA open Colector
	//SET_SCL;
	//CLEAR_SCL;
//
	//// Byte 1
	//for (x = 16; x > 8; x--)
	//{
		//SET_SCL;
		//data[x - 1] = SDA_PIN;
		//CLEAR_SCL;
	//}
//
	//// Acknowledge
	//CLEAR_SDA; // Acknowledge
	//SET_SCL;
	//CLEAR_SCL;
	//SET_SDA; // SDA open Colector
//
	//// Byte 2
	//for (x = 8; x > 0; x--)
	//{
		//SET_SCL;
		//data[x - 1] = SDA_PIN;
		//CLEAR_SCL;
	//}
//
	//// Acknowledge
	//SET_SDA; // Not Acknowledge
	//SET_SCL;
	//CLEAR_SCL;
	//SET_SDA; // SDA open Colector
//
	//// Stop
	//CLEAR_SDA;
	//SET_SCL;
	//SET_SDA;
//}
//
//
///*! \brief Führt einen Offsetabgleich auf den ZMDs aus
 //*/
//void offsetabgleich(void) // Abgleichen des Offsets der DMS-Brücken (nur Digital)
//{
//#define MITTEL 32             // Anzahl der Messwerte, über die gemittelt wird
//#define DIGI_AVERAGE_SIZE 256 // Anzahl Messwerte für digitalen uC Offsetabgleich
	//// Werte werden im EEPROM gespeichert
//
	//uint8_t tx[255] =
	//{ 0 }; // interner Sendepuffer
	//uint8_t x = 0, zmd = 0, b, db;
	//uint8_t data[16];
	//uint32_t mittelwert[4] =
	//{ 0 };
	//char analogoffset[4] =
	//{ 0 };
	//unsigned long wert = 0;
	//uint32_t z = 0;
//
	//sbi(PORTD,7); // ZMD Power on
//
	//do
	//{
		///* Timer-Interrupt deaktivieren */
		//TIMSK1 = 0;
		//TIFR1 = 0x02;
//
		///* letzten Mittelwert zurücksetzen */
		//for (zmd = 0; zmd < 4; zmd++)
		//{
			//mittelwert[zmd] = 0;
		//}
//
		///* Alle Messwerte aufnehmen und mitteln */
		//for (z = 0; z < MITTEL; z++)
		//{
			//// ZMD auslesen
			//read_zmd(data);
//
			//// Werte zusammensetzen
			//for (x = 0; x < 8; x++)
			//{
				//tx[x] = (data[2 * x] & 0x0F) + (data[2 * x + 1] << 4);
			//}
			//for (zmd = 0; zmd < 4; zmd++) // Sensor
			//{
				//wert = 0;
				//for (b = 0; b < 2; b++)
				//{
					//for (db = 0; db < 8; db++) // Datenbyte
					//{
						//wert += ((tx[db] >> (zmd + b * 4)) & 0x01) << (2 * db + b);
					//}
				//}
				//mittelwert[zmd] += wert;
			//}
			//_delay_ms(1);
		//}
//
		//cbi(PORTD,7); // ZMD Power off
//
		//// Mittelwert berechnen
		//for (zmd = 0; zmd < 4; zmd++)
		//{
			//mittelwert[zmd] = mittelwert[zmd] / MITTEL;
//
			//if (mittelwert[zmd] < 5000)
				//analogoffset[zmd] = -1;
			//else if (mittelwert[zmd] > 25000)
				//analogoffset[zmd] = 1;
			//else
				//analogoffset[zmd] = 0;
//
			//mittelwert[zmd] = (16384 - (signed int) mittelwert[zmd]) / 4 + eeprom_flash_data.zmd_config[zmd][0]; // Offset ermitteln
		//}
		//if ((analogoffset[0] == 0) && (analogoffset[1] == 0)
				//&& (analogoffset[2] == 0) && (analogoffset[3] == 0))
		//{
			//break;
		//}
		//// Analogoffset korrigieren
		//for (zmd = 0; zmd < 4; zmd++)
		//{
			//if (analogoffset[zmd] == 0) // analogoffset richtig eingestellt ?
				//continue; // -> keine Anpassung nötig
//
			//z = eeprom_flash_data.zmd_config[zmd][0x19]; // Konfigurationswert lesen
//
			//if ((z & 0x01F0) == 0) // analogoffset=0 ?
			//{
				//if (z & 0x0200) // analogoffset>0 ?
				//{
					//if (analogoffset[zmd] == 1)
						//z = z & 0xFDFF;
				//}
				//else
				//{
					//if (analogoffset[zmd] == -1)
						//z = z | 0x0200;
				//}
			//}
//
			//x = (z & 0x01F0) >> 4;
			//if (z & 0x0200)
			//{
				//if (analogoffset[zmd] == 1)
					//x--;
				//else
					//x++;
			//}
			//else
			//{
				//if (analogoffset[zmd] == 1)
					//x++;
				//else
					//x--;
			//}
			//z = (z & 0xFE0F) | (x << 4);
			//eeprom_flash_data.zmd_config[zmd][0x19] = z;
			//if (x >= 0x1F) // Offset auf Anschlag?
			//{
				//analogoffset[zmd] = 0;
				//continue;
			//}
		//}
		//sbi(PORTD,7); // ZMD Power on
		//init_zmd();
//
		//// Reset watchdog
		//wdt_reset();
//
	//} while ((analogoffset[0] != 0) || (analogoffset[1] != 0)
			//|| (analogoffset[2] != 0) || (analogoffset[3] != 0));
//
	//cbi(PORTD,7); // ZMD Power off
	////    _delay_ms(10);
	//// Offsetabgleich in die Konfiguration laden
	//eeprom_flash_data.zmd_config[0][0] = mittelwert[0];
	//eeprom_flash_data.zmd_config[1][0] = mittelwert[1];
	//eeprom_flash_data.zmd_config[2][0] = mittelwert[2];
	//eeprom_flash_data.zmd_config[3][0] = mittelwert[3];
//
	//sbi(PORTD,7); // ZMD Power on
//
	//// ZMDs neu initialisieren, um die neue Konfiguration zu übernehmen
	//init_zmd();
//
	//// DIGI-OFFSET ermitteln, wird in EEPROM gespeichert und nach Jeder Messung vom Messwert abgezogen
//
	//// letzten Mittelwert zurücksetzen
	//for (zmd = 0; zmd < 4; zmd++)
	//{
		//mittelwert[zmd] = 0;
	//}
//
	//// Alle Messwerte aufnehmen und mitteln
	//for (z = 0; z < DIGI_AVERAGE_SIZE; z++)
	//{
		//// ZMD auslesen
		//read_zmd(data);
//
		//// Werte zusammensetzen
		//for (x = 0; x < 8; x++)
		//{
			//tx[x] = (data[2 * x] & 0x0F) + (data[2 * x + 1] << 4);
		//}
		//for (zmd = 0; zmd < 4; zmd++) // Sensor
		//{
			//wert = 0;
			//for (b = 0; b < 2; b++)
			//{
				//for (db = 0; db < 8; db++) // Datenbyte
				//{
					//wert += ((tx[db] >> (zmd + b * 4)) & 0x01) << (2 * db + b);
				//}
			//}
			//mittelwert[zmd] += wert;
		//}
		////  _delay_ms(1);
		//wdt_reset();
	//}
//
	//// Mittelwert berechnen
	//for (zmd = 0; zmd < 4; zmd++)
	//{
		//mittelwert[zmd] = mittelwert[zmd] / DIGI_AVERAGE_SIZE;
//
		//offset[zmd] = (16384 - (signed int) mittelwert[zmd]); // Offset ermitteln
		//eeprom_flash_data.digi_offset[zmd] = offset[zmd];
	//}
//
	//eeprom_save_config();
//}
//
//
///*! \brief Liest die ZMDs aus und bringt die Messdaten in die richtige Form für die Weiterverarbeitung
 //*/
//void read_all_zmds(void)
//{
	//unsigned char b;
	//unsigned char data[16];
	//static unsigned short werte[4][14];
	//static unsigned char werte_h, werte_l;
//
	////    sbi(DDRD,1);
	////    sbi(PORTD,1); // TXD auf 1 (für Debugzwecke)
	//timeout_timer++;
	//if (nr < SAMPLES_PER_PACKET)
	//{
		//// ZMDs auslesen
		//sbi(DDRD,4);
//
		//cbi(PORTD,4); // LED an
		//read_zmd(data);
		//sbi(PORTD,4); // LED aus
//
//
		//// Werte zusammensetzten
		//werte_h = 0;
		//werte_l = 0;
		//for (b = 1; b < 8; b++) // LSB unbenutzt
		//{
			//if (data[b] & 0x01)
			//{
				//werte_l += 1 << b;
			//}
		//}
		//for (b = 0; b < 7; b++) // MSB unbenutzt
		//{
			//if (data[b + 8] & 0x01)
			//{
				//werte_h += 1 << b;
			//}
		//}
		//werte[0][nr] = werte_l + ((unsigned short) werte_h << 8);
		//
		//werte_h = 0;
		//werte_l = 0;
		//for (b = 1; b < 8; b++) // LSB unbenutzt
		//{
			//if (data[b] & 0x02)
			//{
				//werte_l += 1 << b;
			//}
		//}
		//for (b = 0; b < 7; b++) // MSB unbenutzt
		//{
			//if (data[b + 8] & 0x02)
			//{
				//werte_h += 1 << b;
			//}
		//}
		//werte[1][nr] = werte_l + ((unsigned short) werte_h << 8);
//
		//werte_h = 0;
		//werte_l = 0;
		//for (b = 1; b < 8; b++) // LSB unbenutzt
		//{
			//if (data[b] & 0x04)
			//{
				//werte_l += 1 << b;
			//}
		//}
		//for (b = 0; b < 7; b++) // MSB unbenutzt
		//{
			//if (data[b + 8] & 0x04)
			//{
				//werte_h += 1 << b;
			//}
		//}
		//werte[2][nr] = werte_l + ((unsigned short) werte_h << 8);
//
		//werte_h = 0;
		//werte_l = 0;
		//for (b = 1; b < 8; b++) // LSB unbenutzt
		//{
			//if (data[b] & 0x08)
			//{
				//werte_l += 1 << b;
			//}
		//}
		//for (b = 0; b < 7; b++) // MSB unbenutzt
		//{
			//if (data[b + 8] & 0x08)
			//{
				//werte_h += 1 << b;
			//}
		//}
		//werte[3][nr] = werte_l + ((unsigned short) werte_h << 8);
//
		///* store data in payload */
		//for (b = 0; b < 4; b++)
		//{
			//ptr_payload_spike->strain_gauge[nr][b] = werte[b][nr] + offset[b];
			////ptr_payload_spike->strain_gauge[nr][b] = 0; //ank
			////at86rf231_tx_frame.payload[ 8 * nr + b * 2 ] = ( unsigned char )( werte[b][nr] >> 8 );
			////at86rf231_tx_frame.payload[ 8 * nr + b * 2 + 1 ] = ( unsigned char )werte[b][nr] & 0x00FF;
		//}
	//}
	////cbi(PORTD,1); // TXD auf 0 (für Debugzwecke)
//}