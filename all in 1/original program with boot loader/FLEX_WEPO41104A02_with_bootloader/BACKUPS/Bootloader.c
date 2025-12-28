/*
 * BootLoader.c
 *
 * Created: 06.11.2014 15:26:07
 *  Author: pavt
 */ 
#include "inc/Bootloader.h"
#include "inc/conf_LED.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void bootloader(void)
{
	//static uint8_t flag = 0;
		
	uint8_t *z_ptr_lb = (uint8_t) 0x1E;	// ptr to Z-Reg Low Byte
	uint8_t *z_ptr_hb = (uint8_t)0x1F;	// ptr to Z-Reg High Byte
	
	uint8_t *r0 = (uint8_t)0x00;			// ptr to Register r0
	uint8_t *r1 = (uint8_t)0x01;			// ptr to Register r1
	
	uint8_t i = 0;				// Control variable -> for loop 

/*	Intel HEX File	
	PAGE 89 Start
	:10 2C80 00 28970FB6F894DEBF0FBECDBFF8940E94 10
	                | change |
	:10 2C90 00 7D15 84EF91E0 0E94DD15CE01019628E0 BC
	:10 2CA0 00 DC011D922A95E9F71092160132E0F32E 0D
	:10 2CB0 00 01E014E0C82E792E26E0622E95E0A92E C0
	:10 2CC0 00 B12C8AE4582EB1E08B2EB8E89B2EA5E0 FB
	:10 2CD0 00 EA2EF7E04F2EE0E83E2E73E0272E8091 9B
	:10 2CE0 00 1601833079F1843028F4813071F08230 1C
	:10 2CF0 00 88F4E2C0853009F4DFC08530C8F18630 41		
	PAGE 89 END
*/
	/*uint8_t code[128] = {
							0x28,0x97,0x0F,0xB6,0xF8,0x94,0xDE,0xBF,0x0F,0xBE,0xCD,0xBF,0xF8,0x94,0x0E,0x94, 
							0x7D,0x15,0x84,0xEF,0x91,0xE0,0x0E,0x94,0xDD,0x15,0xCE,0x01,0x01,0x96,0x28,0xE0,
							0xDC,0x01,0x1D,0x92,0x2A,0x95,0xE9,0xF7,0x10,0x92,0x16,0x01,0x32,0xE0,0xF3,0x2E,
							0x01,0xE0,0x14,0xE0,0xC8,0x2E,0x79,0x2E,0x26,0xE0,0x62,0x2E,0x95,0xE0,0xA9,0x2E,
							0xB1,0x2C,0x8A,0xE4,0x58,0x2E,0xB1,0xE0,0x8B,0x2E,0xB8,0xE8,0x9B,0x2E,0xA5,0xE0,
							0xEA,0x2E,0xF7,0xE0,0x4F,0x2E,0xE0,0xE8,0x3E,0x2E,0x73,0xE0,0x27,0x2E,0x80,0x91,
							0x16,0x01,0x83,0x30,0x79,0xF1,0x84,0x30,0x28,0xF4,0x81,0x30,0x71,0xF0,0x82,0x30,
							0x88,0xF4,0xE2,0xC0,0x85,0x30,0x09,0xF4,0xDF,0xC0,0x85,0x30,0xC8,0xF1,0x86,0x30
						};
	
	if(flag == 0)	// Soll LED_blink(100) entsprechen
	{
		code[18] = 0x88;	code[19] = 0xEE;  code[20] = 0x93; code[21] = 0xE0;			
		flag = 1;
	}
	else
	{	
		code[18] = 0x84;	code[19] = 0xEF;  code[20] = 0x91; code[21] = 0xE0;			
		flag = 0;
	}*/
				
	// Write Page into Temp Buffer 
	/*
	*z_ptr_hb = 0;
	
	for(i=0; i < 64; i++)
	{
		while(SPMCSR & 0x01){}; 
		*z_ptr_lb = i;
		*r0 = code[2*i];
		*r1 = code[2*i+1];				
		SPMCSR |= 0x01;	 
		asm("spm");
	}
	
	// Erase PAGE
	while(SPMCSR & 0x01){};
	*z_ptr_lb = 0x80;
	*z_ptr_hb = 0x2C;
	SPMCSR |= 0x03;	 
		asm("spm");
		
	// Write new PAGE
	while(SPMCSR & 0x01){};
	*z_ptr_lb = 0x80;
	*z_ptr_hb = 0x2C;
	SPMCSR |= 0x05;	 
	asm("spm");	
			
	while(SPMCSR & 0x01){};	*/
	
	//==========================================================================================
	
	cli();
	for(i=0; i < 5;i++)
	{
		LED_ON		
		_delay_ms(1000);		
		LED_OFF		
		_delay_ms(1000);				
	}
	
	//*z_ptr_hb = 0;
	
	// Write Page into Temp Buffer 
	
	asm("ldi r16, 0xAA");
	asm("ldi r17, 0x55");
	
	asm("mov r0, r16");
	asm("mov r1, r17");
	
	asm("ldi r16, 0");
	
	asm("st Z, r16");	
		
	for(i=0; i < 64; i++)			// For WORD = 0 to 63
	{										
		
		//*z_ptr_lb = ( i<<1 );		// Set PCWORD in Z-Reg  
		//*r0 = 0xAA;					// Set r0 for Program Data
		//*r1 = 0x55;				// Set r0 for Program Data
				
		SPMCSR |= 0x01;				// Set SPMCSR to Temp Buffer Load
		asm("spm");					// start spm
		
		LED_ON 
		_delay_ms(100); 
		LED_OFF 	
		
		asm("add r30, 2");
	}
	
	for(i=0; i < 5;i++)
	{
		LED_ON		
		_delay_ms(500);		
		LED_OFF		
		_delay_ms(500);				
	}	
	
	// Erase PAGE
	asm("ldi r30, 0x00");				// Set PCWORD ... 
	asm("ldi r31, 0x50");
	SPMCSR |= 0x03;					// Set SPMCSR tp PAGE Erase
	asm("spm");					// start spm
	
	for(i=0; i < 7;i++)
	{
		LED_ON		
		_delay_ms(500);		
		LED_OFF		
		_delay_ms(500);				
	}		
	
	// Write new PAGE
	asm("ldi r30, 0x00");				// Set PCWORD ... 
	asm("ldi r31, 0x50");
	SPMCSR |= 0x05;					// Set SPMCSR tp PAGE Erase
	asm("spm");						// start spm
	
	for(i=0; i < 9;i++)
	{
		LED_ON		
		_delay_ms(2000);		
		LED_OFF		
		_delay_ms(2000);				
	}		
			
	asm("jmp 0x0000"); 	
}