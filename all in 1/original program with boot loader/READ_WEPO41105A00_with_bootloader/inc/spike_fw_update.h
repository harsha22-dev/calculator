/*
 * bootloader.h
 *
 * Created: 02.12.2014 11:16:34
 *  Author: pavt
 */ 


#ifndef SPIKE_FW_UPDATE_H_
#define SPIKE_FW_UPDATE_H_

#include <inttypes.h>

typedef enum {FW_UPD_CMD, FW_END_CMD, PROG_CODE} frame_type;
	
void init_spike_fw_update_task( void );

void vTaskSpikeFwUpdateFlash( void *pvParameters );
void vTaskSpikeFwUpdateSram( void *pvParameters );
void vTaskSpikeFwUpdate( void *pvParameters );

void send(frame_type type);
uint8_t wait_for_response( void );


uint8_t get_prog_code_via_usb( void );
uint16_t store_prog_code_in_flash (void);

char bootloader_usb_getchar(void);
uint8_t bootloader_uart_usb_test_hit(void);



#endif /* SPIKE_FW_UPDATE_H_ */