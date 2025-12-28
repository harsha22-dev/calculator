/**\file *******************************************************************************************
 * 
 * \brief EEPROM data structure and EEPROM I/O-functions
 *
 * \details Functional description:		
 *
 * <b>Target Platforms:</b> ATmega324P
 *
 * <b>Editor:</b>           Atmel AVR Studio 5 (Version: 5.1.208)
 *
 * <b>Compiler:</b>			AVRGCC (Version: 3.3.1.27)  
 * 
 * \date 2013-09-05
 * \version WEPO41104A02
 * \author created by Andre Kuhn
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 *
 **************************************************************************************************/

#include "inc/eeprom_abbild.h"
#include "inc/SX8723c.h"

/**
 *  This data is stored in the EEPROM, see *.eep file.                 
 */
eeprom_data EEMEM struct_eeprom_data =
{ 
	SERIAL_NUMMBER, // serial number
	{				// SX8723C_CONFIG
		{			// SX8723c_1	
			RC_ENABLE,												// RegCen[0x30]
			D0_DIR_OUTPUT | D1_DIR_INPUT | REG_OUT_RESERVED_PIN,	// RegOut[0x40]
			I2C_TIMEOUT_DISABLE | REG_TIMEOUT_RESERVED_PIN,			// RegTimeout[0x42]
			INTERNAL_I2C_ADDRESS,									// RegExtAdd[0x43]
			REG_ACCFG0_RESERVED_PIN | OSR_128 | NELCONV_2,			// RegACCfg0[0x52]
			ADC_ENABLE | PGA1_ENABLE | PGA2_ENABLE | PGA3_ENABLE | IB_AMP_PGA_100 | IB_AMP_ADC_100,	// RegACCfg1[0x53]
			PGA2_GAIN_10 | OSF_500KHZ,								// RegACCfg2[0x54]
			PGA1_GAIN_10 | 0x7F,									// RegACCfg3[0x55]
			REG_ACCFG4_RESERVED_PIN,								// RegACCfg4[0x56]
			VMUX_VBATT | AMUX_AC2_AC3 | AMUX_SIGN_CROSS | AMUX_MODE_DIFFERENTIAL_INPUT,	// RegACCfg5[0x57]
			VREF_D1_IN_ENABLE | MULTI_FORCE_OFF | CHOP_0 | REG_MODE_RESERVED_PIN	// RegMode[0x70]
		},
		{			// SX8723c_2
			RC_ENABLE,												// RegCen[0x30]
			D0_DIR_OUTPUT | D1_DIR_INPUT | REG_OUT_RESERVED_PIN,	// RegOut[0x40]
			I2C_TIMEOUT_DISABLE | REG_TIMEOUT_RESERVED_PIN,			// RegTimeout[0x42]
			INTERNAL_I2C_ADDRESS,									// RegExtAdd[0x43]
			REG_ACCFG0_RESERVED_PIN | OSR_128 | NELCONV_2,			// RegACCfg0[0x52]
			ADC_ENABLE | PGA1_ENABLE | PGA2_ENABLE | PGA3_ENABLE | IB_AMP_PGA_100 | IB_AMP_ADC_100,	// RegACCfg1[0x53]
			PGA2_GAIN_10 | OSF_500KHZ,								// RegACCfg2[0x54]
			PGA1_GAIN_10 | 0x7F,									// RegACCfg3[0x55]
			REG_ACCFG4_RESERVED_PIN,								// RegACCfg4[0x56]
			VMUX_VBATT | AMUX_AC2_AC3 | AMUX_SIGN_CROSS | AMUX_MODE_DIFFERENTIAL_INPUT,	// RegACCfg5[0x57]
			VREF_D1_IN_ENABLE | MULTI_FORCE_OFF | CHOP_0 | REG_MODE_RESERVED_PIN	// RegMode[0x70]
		},
		{			// SX8723c_3
			RC_ENABLE,												// RegCen[0x30]
			D0_DIR_OUTPUT | D1_DIR_INPUT | REG_OUT_RESERVED_PIN,	// RegOut[0x40]
			I2C_TIMEOUT_DISABLE | REG_TIMEOUT_RESERVED_PIN,			// RegTimeout[0x42]
			INTERNAL_I2C_ADDRESS,									// RegExtAdd[0x43]
			REG_ACCFG0_RESERVED_PIN | OSR_128 | NELCONV_2,			// RegACCfg0[0x52]
			ADC_ENABLE | PGA1_ENABLE | PGA2_ENABLE | PGA3_ENABLE | IB_AMP_PGA_100 | IB_AMP_ADC_100,	// RegACCfg1[0x53]
			PGA2_GAIN_10 | OSF_500KHZ,								// RegACCfg2[0x54]
			PGA1_GAIN_10 | 0x7F,									// RegACCfg3[0x55]
			REG_ACCFG4_RESERVED_PIN,								// RegACCfg4[0x56]
			VMUX_VBATT | AMUX_AC2_AC3 | AMUX_SIGN_CROSS | AMUX_MODE_DIFFERENTIAL_INPUT,	// RegACCfg5[0x57]
			VREF_D1_IN_ENABLE | MULTI_FORCE_OFF | CHOP_0 | REG_MODE_RESERVED_PIN	// RegMode[0x70]
		},
		{			// SX8723c_4	
			RC_ENABLE,												// RegCen[0x30]
			D0_DIR_OUTPUT | D1_DIR_INPUT | REG_OUT_RESERVED_PIN,	// RegOut[0x40]
			I2C_TIMEOUT_DISABLE | REG_TIMEOUT_RESERVED_PIN,			// RegTimeout[0x42]
			INTERNAL_I2C_ADDRESS,									// RegExtAdd[0x43]
			REG_ACCFG0_RESERVED_PIN | OSR_128 | NELCONV_2,			// RegACCfg0[0x52]
			ADC_ENABLE | PGA1_ENABLE | PGA2_ENABLE | PGA3_ENABLE | IB_AMP_PGA_100 | IB_AMP_ADC_100,	// RegACCfg1[0x53]
			PGA2_GAIN_10 | OSF_500KHZ,								// RegACCfg2[0x54]
			PGA1_GAIN_10 | 0x7F,									// RegACCfg3[0x55]
			REG_ACCFG4_RESERVED_PIN,								// RegACCfg4[0x56]
			VMUX_VBATT | AMUX_AC2_AC3 | AMUX_SIGN_CROSS | AMUX_MODE_DIFFERENTIAL_INPUT,	// RegACCfg5[0x57]
			VREF_D1_IN_ENABLE | MULTI_FORCE_OFF | CHOP_0 | REG_MODE_RESERVED_PIN	// RegMode[0x70]
		}
	}, 
	SW_VERSION,		// SW-Version 
	HW_VERSION,		// HW-Version 
	SRC_ADDRESS,	// src_address
	0x8888,			// dest_address
	0xFFFF,			// pan_id, broadcast 
	CHANNEL,		// channel 
	2,				// channel_page
	0x0F,			// tx pwr TODO
	TIMEOUT,		// timeout [s]
	1600,			// samplerate [Hz]
	{				// software_offset of strain gauge		
		0x0000,		// SG_1
		0x0000,		// SG_2
		0x0000,		// SG_3
		0x0000		// SG_4
	}, 
	{				// calibration values
		0x000F4240, 
		0x000F4240,  
		0x000F4240,  
		0x000F4240,  
		0x000F4240, 
		0x000F4240,  
		0x000F4240,  
		0x000F4240
	} 
};


void eeprom_read_config(eeprom_data* ptr_sram_data)
{
	uint16_t i = 0;
	uint8_t temp = 0;
	uint8_t *ptr_temp_sram_data = (uint8_t *) ptr_sram_data;

	if (eeprom_read_byte((uint8_t*) 0x00) != 0xFF)
	{
		for (i = 0; i < sizeof(eeprom_data); i++)
		{
			temp = eeprom_read_byte((uint8_t *) i);
			*(ptr_temp_sram_data + i) = temp;
		}
	}
}

void eeprom_save_config(eeprom_data* ptr_sram_data)
{
	uint16_t i = 0;
	uint8_t *ptr_temp_sram_data = (uint8_t *) ptr_sram_data;

	for (i = 0; i < sizeof(eeprom_data); i++)
	{
		eeprom_write_byte((uint8_t *) i, *(ptr_temp_sram_data + i));
	}
}