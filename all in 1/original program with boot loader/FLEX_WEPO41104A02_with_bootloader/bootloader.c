/*
 * BootLoader.c
 *
 * Created: 06.11.2014 15:26:07
 *  Author: pavt
 */ 
#include "inc/bootloader.h"
#include "inc/conf_LED.h"
#include "inc/at86.h"
#include <avr/io.h>
#include <avr/boot.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/iom324p.h>

void boot()	// This function hast to be set onto the BOOT Reset Vector  
{	
	//asm("jmp 0x0000");
		
	bootloader();	
}

void bootloader(void) 
{	
	extern at86rf231_frame at86rf231_rx_frame;
	
	uint8_t		page = 0;							// Index for flash write/read function -> Which page is addressed  
	uint8_t		page_code[SPM_PAGESIZE] = {0};		// Stores Rx data into this array -> used as data_ptr for flash_write()   
	uint8_t		page_code_index = 0;				// Index to control the buffer page_code   	
	uint8_t		loaded_page[SPM_PAGESIZE] = {0};	// Loaded Flash (for verification) is stored in this array and compared  
	uint8_t		frame_index = 0;					// Used as for-loop control variable to store the TRx frame into page_code
	uint8_t		fw_upd_end = 0;						// Flag to signal the end of the firmware update -> All data written  
	uint8_t		length = 0;							// The Frame length info from TRx is stored here 
	uint8_t		verify_ok = 0;						// Used as control variable for Flash verification 
	uint8_t		expected_seq_nr = 0;				// Expected Sequence Nr. within Rx Frame
	uint8_t		i = 0;								// Control variable for for-loops
	uint16_t	wait_cntr = 0;
	
	cli();				// Disable Global Interrupts 	
	bootloader_setup_MCU();	
	bootloader_setup_TRx();	
	
	bootloader_send_response(ACK);	 	// Acknowledge the received Firmware Update command 
	
	do 
	{	
		bootloader_wait_for_rx_frame();	// Waits until Rx Frame is received		
		
		if (bootloader_pal_trx_bit_read(SR_RX_CRC_VALID) == CRC16_VALID)	// Check if CRC check sum is valid
		{	
			switch(FCF_FRAME_TYPE)		// FCF_FRAME_TYPE -> (at86rf231_rx_frame.frame_control_field & 0x3) 
			{
				case FCF_FRAMETYPE_DATA:
				
					length = at86rf231_rx_frame.frame_length - MAC_OVERHEAD;
					
					if( expected_seq_nr != at86rf231_rx_frame.seq_number)	  // If Paket nr. does not match ...  
					{
						bootloader_send_response(ACK);		// ... then an ACK was not received for the previous paket.  
															// The READ will Tx the previously received paket again.  
						break;								// Leave the switch, drop that frame and wait for next Rx message	   
					}
					expected_seq_nr++;
					
					for (frame_index = 0; frame_index < length; frame_index++)
					{
						page_code[page_code_index++] = at86rf231_rx_frame.payload[frame_index];
						
						if(page_code_index == SPM_PAGESIZE) // If page_code full
						{
							// Write to FLASH and verify ----------------------------------------------------				
							verify_ok = 0;
							do 
							{						
								bootloader_write_flash_page(page, page_code);					
								bootloader_load_flash_page(page, loaded_page);	// load PAGE for verification of correct FLASH write	
						
								for(i = 0; i <SPM_PAGESIZE; i++)	// Compare each Byte of the written page with the received program code 
								{
									if( page_code[i] == loaded_page[i])
										verify_ok = 1;
									else
									{
										verify_ok = 0;								
										bootloader_send_response(FLASH_ERR);	// Just for development purposes 
										break;		
									}
								}
					
							}while (verify_ok == 0);
							page++;
							page_code_index = 0;	// Reset buffer index
						}	
					}					
					bootloader_send_response(ACK);
					break;
					
				//---------------------------------------------------------------------------------
				case FCF_FRAMETYPE_MAC_CMD:
					
					if(at86rf231_rx_frame.payload[0] == FW_UPD_END) 
					{	
						bootloader_send_response(ACK);
						
						// If there is still Flash memory left unprogrammed 
						// write the remaining Flash bytes with the remaining bytes or 0xFF for unused Flash
						while(page < TOT_NR_APP_PAGES)
						{
							page_code[page_code_index++] = 0xFF;
							
							if(page_code_index == SPM_PAGESIZE) // If page_code full
							{
								// Write to FLASH and verify ----------------------------------------------------				
								verify_ok = 0;
								do 
								{						
									bootloader_write_flash_page(page, page_code);					
									bootloader_load_flash_page(page, loaded_page);	// load PAGE for verification of correct FLASH write	
						
									for(i = 0; i <SPM_PAGESIZE; i++)	// Compare each Byte of the written page with the received program code 
									{
										if( page_code[i] == loaded_page[i])
											verify_ok = 1;
										else
										{
											verify_ok = 0;								
											bootloader_send_response(FLASH_ERR);	// Just for development purposes 
											break;		
										}
									}
					
								}while (verify_ok == 0);
								page++;
								page_code_index = 0;	// Reset buffer index
							}	
						}						
						fw_upd_end = 1;
					}
					
					if (at86rf231_rx_frame.payload[0] == FW_UPD_CMD)	//
					{
						bootloader_send_response(ACK);
						
						// Reset all control variables   
						page = 0;
						page_code_index = 0;
						expected_seq_nr = 0;
					}
					break;
				
				//-------------------------------------------------------------------------------	
				default:	// Beacon or ACK Frame -> Nothing to do 
					break;	
			}
		}
		else
		{
			//send_nak();
			bootloader_send_response(NAK);
		}	
												
	}while (fw_upd_end == 0);
	
	// Verify that READ received the ACK for End of Session (EoS) 
	// NOTE: It might happen that the ACK for EoS got lost and READ 
	//		 (not knowing that EoS was received on FLEX) will continue to send 
	//		 the FW_UPD_END frame. Therefore check if these frames are still incoming.
		
	do 
	{
		wait_cntr = 0;
		// Set TRx into Receive mode
		bootloader_pal_trx_bit_write(SR_TRX_CMD,RX_ON);
		while(bootloader_pal_trx_bit_read(SR_TRX_STATUS) != RX_ON)
		{ }	
		
		// Wait until an incoming Frame is received or until
		// the Counter reaches its limit 
		while(bootloader_pal_trx_bit_read(SR_IRQ_3_TRX_END) == 0 )
		{	
			wait_cntr++;		
			if (wait_cntr >= 10000)
				break;
		}
		
		// If counter didn't reach its limit -> Incoming frame received -> ACK 
		if(wait_cntr < 10000)	
			bootloader_send_response(ACK);
		
	}while(wait_cntr < 10000);	// If no frame received within timeout-> Session finished 
			
	asm("jmp 0x0000");	// Jump to App Flash section 
}	

void bootloader_setup_MCU(void)
{
	// Setup the MCU for the Firmware Update: 
	// Deactivate Interrupt sources (since the bootloader uses only polling)
	// and initialize the SPI communication to the Transceiver 				
					
	EIMSK = (0 << INT2) | (0 << INT1) | (0 << INT0); 					// Disable External Interrupts 
	PCICR = (0 << PCIE3) | (0 << PCIE2) | (0 << PCIE1) | (0 << PCIE0);	// Disable Pin Change Interrupt 
	
	// Disable all Modules except SPI, which is needed for the µC-TRx communication 
	// This also deactivates the various internal Interrupt sources that might jump into the RWW section    
	PRR0 = (1 << PRTWI) | (1 << PRTIM2) | (1 << PRTIM0) | (1 << PRUSART1)| 
		   (1 << PRTIM1) | (0 << PRSPI) | (1 << PRUSART0) | (1 << PRADC);	   
	
	// Initialize SPI communication just in case that:
	// a) MCU is coming from a Reset State
	// b) FLEX Firmware does not (correctly) initialize it 
	MCUCR &= ~(1 << PUD);		// Enable internal Pull Ups for IO
	
	// Define Data Direction (1 = Output) of SPI Output Pins
	TRX_PORT2_DDR |= (1 << SEL );
    TRX_PORT1_DDR |= (1 << SCK );
    TRX_PORT2_DDR |= (1 << RST );
    TRX_PORT1_DDR |= (1 << MOSI );
    TRX_PORT1_DDR |= (1 << SLP_TR );
	
	// Define Data Direction (0 = Input) of SPI Input Pins
	TRX_PORT1_DDR &= ~(1 << MISO );
	
	// Enable SPI and make MCU the Master 
	// NOTE: In the current Hardware Layout the Slave Select (SS) is on PORT PC7 ...   
	//		 ... BUT to be able to define MCU as SPI Master the original SS Pin on PB4
	//		 has to be set HIGH !!! If PB4 = Low the MCU expects a write access from a 
	//		 different SPI Master and sets its Mode to Slave. --> Set PB4 as Output + HIGH 
	   
	DDRB |= (1 << DDRB4);		// DataDirectionReg. = 1 -> output 
	PORTB |= (1 << DDRB4);		// Set PB4 = HIGH
	TRX_INIT();					// Enable SPI Module and set Master	 
} 

void bootloader_setup_TRx(void)
{	
	// Setup the TRx for the Firmware Update: 
	// Initialize the necessary Registers for Basic Operation just in Case that:
	// a) TRx is coming from a Reset State 
	// b) FLEX Firmware uses different settings for Measurement Data Transmission    
	
	// After Power-On it is necessary to set teh TRx Pins to default operation 
	// SLP_TR = L, /RST = H (and /SEL = H but this is already done by the SPI module).
	PAL_RST_HIGH();
    PAL_SLP_TR_LOW();
	
	bootloader_pal_trx_bit_write(SR_TRX_CMD, TRX_OFF);						// Set TRx into "Standby" (no Rx no Tx) ...
	while(bootloader_pal_trx_bit_read(SR_TRX_STATUS) != TRX_OFF)			// .. and wait till state change finished 
	{	}
	
	bootloader_pal_trx_bit_write(SR_IRQ_MASK, 0x00);						// Disable Interrupts but ...
    bootloader_pal_trx_bit_write(SR_IRQ_MASK_MODE,IRQ_MASK_MODE_ON);		// ... enable poll mode (to read IRQ status) 
	
	bootloader_pal_trx_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_250KBPS);		// Change data rate to 250 kb/s mode  
	bootloader_pal_trx_bit_write(SR_RX_PDT_LEVEL, 0x00);					// Use full sensitivity (-101 dBm)
	bootloader_pal_trx_bit_write(SR_RX_PDT_DIS, RX_ENABLE);					// Enable Rx Frame Detection
	
	bootloader_pal_trx_bit_write(SR_RX_SAFE_MODE, RX_SAFE_MODE_ENABLE );	// Enable buffer protection mode
	bootloader_pal_trx_bit_write(SR_ANT_DIV_EN, ANT_DIV_DISABLE);			// No Antenna Diversity 
	
	bootloader_pal_trx_bit_write(SR_TX_AUTO_CRC_ON,TX_AUTO_CRC_ENABLE);		// Enable Auto CRC Generation 
	
	pal_trx_bit_write( SR_CLKM_CTRL, CLKM_1MHZ );					// Config clock = 1 MHz
	
	bootloader_pal_trx_bit_write(SR_SPI_CMD_MODE, SPI_CMD_MODE_DEFAULT);	// No monitoring byte in SPI comm.
	
	// Write device addresses used for Automatic Address Matching 
	bootloader_pal_trx_reg_write( RG_PAN_ID_0,    0xFF);	// Set PAN ID Low Byte 				
    bootloader_pal_trx_reg_write( RG_PAN_ID_1,    0xFF);	// Set PAN ID High Byte 
	
    bootloader_pal_trx_reg_write( RG_SHORT_ADDR_0, 0x0F);	// Set Source Address Low Byte 
    bootloader_pal_trx_reg_write( RG_SHORT_ADDR_1, 0xA0);	// Set Source Address High Byte
	
	bootloader_pal_trx_bit_write( SR_CHANNEL, 0x12);	// Set channel (carrier freq) 0xB (11) ... 0x1A (26) 
}

void bootloader_wait_for_rx_frame(void)
{	
	extern at86rf231_frame at86rf231_rx_frame;
	uint8_t length = 0; 
	
	volatile uint8_t reg = 0;
	volatile uint8_t trx = 0;
	volatile uint8_t ami = 0;
	
	uint8_t rx_match = 0;
	 
	bootloader_pal_trx_bit_write(SR_TRX_CMD,RX_ON);	// Set TRx into Receive Mode (RX_ON)
	bootloader_pal_trx_reg_read(RG_IRQ_STATUS);		// Reset Interrupt Status Reg.
	while(bootloader_pal_trx_bit_read(SR_TRX_STATUS) != RX_ON)
	{ }
	
	// Wait for packet to be received:
	// If packed received the Interrupt Flag TRX_END -> 1
	// If this packet is dedicated for this node (address matching) INT Flag AMI -> 1
		
	/*	
	while( bootloader_pal_trx_bit_read(SR_IRQ_3_TRX_END) == 0)
	{	
	}*/		
		
	/*do 
	{	
		// The Rx of an incoming Frame is signaled via Interrupt Flag Polling: 
		// The order of the interrupts should be:
		// Start of Rx Frame -> Address Match -> Rx Frame finished  
		// IRQ_2 (RX_START)  -> IRQ_5 (AMI)   -> IRQ_3 (TRX_END) 
		
		rx_match |= bootloader_pal_trx_reg_read(RG_IRQ_STATUS);
		
		if((rx_match & 0x2C) == 0x0C)	// If Rx_START and Rx_END but NO Address Match  
			rx_match = 0;
			
		if(rx_match & 0x40)		// If Frame Buffer Violation 
			rx_match = 0;		// NOTE: This should not happen since the Frame Buffer is not read during Rx
		
	} while ( (rx_match & 0x2C) != 0x2C);*/
	
	do 
	{	
		// The Rx of an incoming Frame is signaled via Interrupt Flag Polling: 
		// The order of the interrupts should be:
		// Start of Rx Frame -> Address Match -> Rx Frame finished  
		// IRQ_2 (RX_START)  -> IRQ_5 (AMI)   -> IRQ_3 (TRX_END) 
		
		rx_match |= bootloader_pal_trx_reg_read(RG_IRQ_STATUS);
		
		// If Rx_START and Rx_END but NO Address Match  
		if( (rx_match & (TRX_IRQ_RX_START | TRX_IRQ_AMI | TRX_IRQ_TRX_END) ) == (TRX_IRQ_RX_START | TRX_IRQ_TRX_END) )	
			rx_match = 0;
			
		if( rx_match & TRX_IRQ_TRX_UR )		// If Frame Buffer Violation 
			rx_match = 0;		// NOTE: This should not happen since the Frame Buffer is not read during Rx
		
	}while ( (rx_match & (TRX_IRQ_RX_START | TRX_IRQ_AMI | TRX_IRQ_TRX_END) ) != (TRX_IRQ_RX_START | TRX_IRQ_AMI | TRX_IRQ_TRX_END) );
				
	// Read Frame Length from TRx frame buffer 
	bootloader_pal_trx_frame_read(&length,LENGTH_FIELD_LEN);
	
	// Read packet from TRx Frame buffer and store it into at86rf231_rx_frame 			
	bootloader_pal_trx_frame_read((uint8_t*)&at86rf231_rx_frame,length);
}

void bootloader_send_response(frame_type type)
{
	extern at86rf231_frame at86rf231_tx_frame;
	extern at86rf231_frame at86rf231_rx_frame;
	// Setup the Tx frame: 
	 
	// These parts of the frame are identical for all frame types
	at86rf231_tx_frame.frame_length = FCF_LEN+SEQ_NUM_LEN+FCS_LEN;
	
	at86rf231_tx_frame.frame_control_field = FCF_SET_FRAMETYPE(FCF_FRAMETYPE_ACK)|
											 FCF_SET_DEST_ADDR_MODE(FCF_SHORT_ADDR)| 
											 FCF_SET_SOURCE_ADDR_MODE(FCF_SHORT_ADDR);
	
	at86rf231_tx_frame.seq_number = at86rf231_rx_frame.seq_number;	// Set ACK Sequence nr. equal to received seq nr.   
	
	switch (type)	// Set frame bits that depend on the type of response 
	{
		case ACK:
			// Do nothing since the FCF is already correct
			break;
		
		case NAK:
			at86rf231_tx_frame.frame_control_field |= FCF_ACK_REQUEST;
			break;
		
		case FLASH_ERR:
			at86rf231_tx_frame.frame_control_field |= FCF_FRAME_PENDING;
			break;
		
		/*case REQ:
			at86rf231_tx_frame.frame_control_field |= FCF_ACK_REQUEST;
			at86rf231_tx_frame.frame_control_field |= FCF_FRAME_PENDING;
			break;
		*/
		default:
			
			break;				
	}
	
	// Write frame to the TRx frame buffer
	bootloader_pal_trx_frame_write((uint8_t*) &at86rf231_tx_frame, at86rf231_tx_frame.frame_length);
	
	// Set TRx into Transmission mode (PLL_ON)
	bootloader_pal_trx_bit_write(SR_TRX_CMD,CMD_PLL_ON);
	while(bootloader_pal_trx_bit_read(SR_TRX_STATUS) != PLL_ON)
	{ 			
	}		
	
	// Toogle the SLP_TR pin to trigger transmission	
	PAL_SLP_TR_HIGH();
	asm volatile("nop\n\t" ::); // wait 65ns
	asm volatile("nop\n\t" ::);
	asm volatile("nop\n\t" ::);
	asm volatile("nop\n\t" ::);
	asm volatile("nop\n\t" ::);
	PAL_SLP_TR_LOW();	
	
	// Wait for Frame to be transmitted		
	while(bootloader_pal_trx_bit_read(SR_IRQ_3_TRX_END) == 0)
	{		
	}	
}

void bootloader_write_flash_page(uint16_t page, uint8_t *ptr_data)
{
	uint16_t i;
	
	eeprom_busy_wait ();
    boot_page_erase (page<<7);	// page<<7, because these MAKROS (boot_page_erase, fill, write)
								// refer to the complete Z-ptr for the address. But "PCPAGE" is allocated 
								// from Bit 14:7 within the Z-Reg.   
    boot_spm_busy_wait ();      // Wait until the memory is erased.

    for (i=0; i<SPM_PAGESIZE; i+=2)
	{					
		// Set up little-endian word.
        uint16_t word = *ptr_data++;
        word += (*ptr_data++) << 8;
        
        boot_page_fill ((page<<7) + i, word);	// Store Page from SRAM to temp. buffer 
    }
				
    boot_page_write (page<<7);		// Store buffer in flash page.
    boot_spm_busy_wait();			// Wait until the memory is written.
	
	// Reenable RWW-section 
	// -> Reenables access to the App Flash for the Programm Counter
	// This is needed to be able to jump back to the Application Flash section
	// and to read the App Flash section (for verification)     
	boot_rww_enable ();	
}

void bootloader_load_flash_page(uint16_t page, uint8_t *ptr_check)
{
	/* To load FLASH BYTEWISE the LPM instruction is used:
	   - The address of the Byte that has to be read is stored in the Z-pointer (R31:R30)   
	   - "lpm" (load propgram memory) is called 
	   -> Flash byte is stored in General Purpose Register (GPR) R0*/ 
	
	uint16_t i;
	uint16_t address = 0; 
		
	for (i=0; i< SPM_PAGESIZE; i++)		// Pagesize = 128 Bytes per PAGE (for ATMega324p)
	{									
										// Z-Pointer (R31:R30) for ATMega324:
		address = (page << 7) | i;		// Bit: 15(MSB)	14			7 6		 0(LSB)	 
										//				|    PAGE   | | Byte |
		 __asm__ 
		(                                            
			"mov r30, %1\n\t"			// Move Low Byte of address to r30 (Low Byte of z-pointer)                          
			"mov r31, %2\n\t"			// Move High Byte of address to r31 (High Byte of z-pointer)
			"lpm \n\t"					// Load Program Memory: Loads the Flash Byte addressed by the value 
										// in z-pointer and stores it in GPR r0
			"mov %0, r0 \n\t"			// Move r0 (the Program Byte) into the array field pointed to by ptr_check  
			
			// Outputs:
			:   "=r" ( *ptr_check )					// Pointer to the array which stores the PAGE	                    
			// Inputs:
			:	"r" ( (uint8_t) (0xFF&address)),	// Loads Low Byte of "address" into r30 (ZL)         
				"r" ( (uint8_t)(address >> 8))		// Loads High Byte of "address" into r31 (ZH)
		); 
		
		ptr_check++;	// Increment pointer to set the address on the next array field (for the following Program Byte) 
	}
}

void bootloader_pal_trx_reg_write(uint8_t addr, uint8_t data)
{  
#ifdef NON_BLOCKING_SPI
    while (spi_state != SPI_IDLE)
    {
        /* wait until SPI gets available */
    }
#endif

    /* Prepare the command byte */
    addr |= WRITE_ACCESS_COMMAND;

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the Read command byte */
    SPDR0 = addr;
    SPI_WAIT();

    /* Write the byte in the transceiver data register */
    SPDR0 = data;
    SPI_WAIT();

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();
}

uint8_t bootloader_pal_trx_reg_read(uint8_t addr)
{
    uint8_t register_value;

#ifdef NON_BLOCKING_SPI
    while (spi_state != SPI_IDLE)
    {
        /* wait until SPI gets available */
    }
#endif

    /* Prepare the command byte */
    addr |= READ_ACCESS_COMMAND;

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the Read command byte */
    SPDR0 = addr;
    SPI_WAIT();

    /* Do dummy read for initiating SPI read */
    SPDR0 = SPI_DUMMY_VALUE;
    SPI_WAIT();

    /* Read the byte received */
    register_value = SPDR0;

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();
	   
    return register_value;
}

void bootloader_pal_trx_frame_read(uint8_t *data, uint8_t length)
{  
    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPDR0 = TRX_CMD_FR;
    SPI_WAIT();
    SPDR0 = SPI_DUMMY_VALUE;

    if (length != 1)
    {
        do
        {
            uint8_t temp;
			
            SPI_WAIT();
            /* Upload the received byte in the user provided location */
            temp = SPDR0;
            SPDR0 = SPI_DUMMY_VALUE; /* Do dummy write for initiating SPI read */
            *data = temp;
            data++;
            length--;
        } while (length > 1);
    }

    SPI_WAIT(); /* Wait until last bytes is transmitted. */
    *data = SPDR0;

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();
}

void bootloader_pal_trx_frame_write(uint8_t *data, uint8_t length)
{
#ifndef NON_BLOCKING_SPI
    cli();
    /* Assumption: The TAL has already disabled the trx interrupt. */

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Start SPI transfer by sending the command byte */
    SPDR0 = TRX_CMD_FW;

    SPI_WAIT();

    do
    {
        uint8_t temp = *data;
        data++;
        length--;
        SPI_WAIT();
        SPDR0 = temp;
    } while (length > 0);

    SPI_WAIT(); /* Wait until last bytes is transmitted. */

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();
   
#else

    spi_state = SPI_WRITE;
    spi_remaining_bytes = length;
    spi_data_ptr = data;
    SPI_IRQ_ENABLE();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Start SPI transfer by sending the command byte */
    SPDR0 = TRX_CMD_FW;
#endif
}

uint8_t bootloader_pal_trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos)
{
    uint8_t ret;

    ret = bootloader_pal_trx_reg_read(addr);
    ret &= mask;
    ret >>= pos;

    return ret;
}

void bootloader_pal_trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t new_value)
{
    uint8_t current_reg_value;

    current_reg_value = bootloader_pal_trx_reg_read(reg_addr);
    current_reg_value &= ~mask;
    new_value <<= pos;
    new_value &= mask;
    new_value |= current_reg_value;

    bootloader_pal_trx_reg_write(reg_addr, new_value);
} 



