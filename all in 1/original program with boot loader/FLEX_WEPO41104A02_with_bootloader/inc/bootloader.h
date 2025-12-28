/*
 * Boot_Loader.h
 *
 * Created: 06.11.2014 15:28:14
 *  Author: pavt
 */ 



#ifndef BOOT_LOADER_H_
#define BOOT_LOADER_H_

#ifndef PRUSART1		// Should be implemented in iom324p but it is not !!! Used for Power Reduction Register 
	# define PRUSART1 4 
#endif

#define FCF_FRAME_TYPE (at86rf231_rx_frame.frame_control_field & 0x7)  
#define FW_UPD_CMD 0x40
#define FW_UPD_END 0x04

#define MAC_OVERHEAD (FCF_LEN + SEQ_NUM_LEN + 2*PAN_ID_LEN + 2*SHORT_ADDR_LEN + FCS_LEN)

#define TOT_NR_APP_PAGES 224 // Calculated as:	APP_PAGES = (Total Flash - Boot FLASH)/ SPM_PAGESIZE = (32-4)*1024/128

#include <inttypes.h>
#define BOOT_START_ADDR 0x7000

typedef enum{ ACK, NAK, FLASH_ERR, REQ} frame_type;

/**
 *\brief Prototype of Bootloader procedure
 *\param void
 */

/*	Attribute Explanation:	
	
	__attribute__((section(".x"))) void fct()   -->	Defines that the function "fct" has to be placed													has to be placed into 
													into the memory section .x
	To define the memory address (e.g. 0x1000) of section ".x" a Flag has to be set in the Toolchain under:
	AVR/GNU C Linker -> Miscellaneous -> Other Linker Flags: -Wl,--section-start=.x=0x1000	 
	
	Defined Sections:
	
	__attribute__((section(".boot"))) 
		--> This section shall be placed onto the Boot start address defined by the Boot Size FUSES:
			Bootsize (Byte):		4096	2048	1024	512
			Boot Address (Byte):	0x7000	0x7C00	0x7800	0x7E00 		
	==> Toolchain flag: -Wl,--section-start=.boot=0x7000
		
	NOTE:	To get the bootloader running after a Reset (due to watchdog, brown-out, ...) the 
			BOOTRST (Boot reset) FUSE has to be programmed !!!   
		
	__attribute__((section(".bootloader"))) 
		--> This section is used to store the various bootloader functions. 
			Its address MSUT NOT overlap with the .boot section.     
	==> Toolchain flag: -Wl,--section-start=.bootloader=0x7100
	
	NOTE:	The reason to have two sections is that the linker positions the functions within a section   
			on different memory addresses depending on the size of the function and other criteria. 
			Meaning the actual bootloader() function can be set onto an unknown address within 
			the .bootloader section depending on the size of the other boot functions.
			
			To have a defined call to the bootloader (in all Firmwares) the extra section .boot was introduced.
			Here only this boot() function must be present to get the exact address defined in the flag.
			This boot() function is then responsible to call either APP or BOOT depending on the situation.  
*/

__attribute__((section(".boot"))) __attribute__((optimize("O0"))) void boot(void);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader(void);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_write_flash_page(uint16_t page, uint8_t *ptr_data);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_load_flash_page(uint16_t page, uint8_t *ptr_check);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_send_response(frame_type type);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_wait_for_rx_frame(void);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_setup_TRx(void);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_setup_MCU(void);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_pal_trx_reg_write(uint8_t addr, uint8_t data);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) uint8_t bootloader_pal_trx_reg_read(uint8_t addr);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_pal_trx_frame_read(uint8_t *data, uint8_t length);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_pal_trx_frame_write(uint8_t *data, uint8_t length);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) uint8_t bootloader_pal_trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos);

__attribute__((section(".bootloader"))) __attribute__((optimize("O0"))) void bootloader_pal_trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t new_value);

#endif /* BOOT_LOADER_H_ */

