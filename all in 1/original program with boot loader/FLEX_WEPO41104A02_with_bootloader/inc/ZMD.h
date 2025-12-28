/*
 * ZMD.h
 *
 * Created: 21.03.2013 11:07:57
 *  Author: andre.kuhn
 */ 


#ifndef ZMD_H_
#define ZMD_H_

#include "sw_twi.h"
#include "at86.h"
#include <avr/wdt.h>

extern uint32_t timeout_timer;
extern spike_payload *ptr_payload_spike;
extern signed int offset[4];
extern uint8_t timeslot_number;
//extern volatile eeprom_data eeprom_flash_data;	

#define ZMD1    0x01
#define ZMD2    0x02
#define ZMD3    0x04
#define ZMD4    0x08
#define ZMD_ALL 0x0F

void zmd_cmd(unsigned char zmd, unsigned char cmd);
void init_zmd(void);
void read_zmd(unsigned char *data);
void offsetabgleich(void);
void read_all_zmds(void);

#endif /* ZMD_H_ */