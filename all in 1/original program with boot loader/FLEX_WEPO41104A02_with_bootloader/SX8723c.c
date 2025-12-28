/**\file *******************************************************************************************
 * 
 * \brief Semtech SX8723c Service
 *
 * \details Functional description:
 *
 *          The Semtech SX8723c module includes project-specific definitions
 *			and function prototypes, such as the SX8723c initialization function.
 *
 * <b>Target Platforms:</b> AT32UC3A3256 (other Controller/independent)
 *
 * <b>Editor:</b>           Atmel AVR Studio 5 (Version: 5.1.208)
 *
 * <b>Compiler:</b>         AVRGCC (3.3.1.27)
 * 
 * \date 2013-09-09
 * \version WEPO41104A02
 * \author created by Andre Kuhn
 * 
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 *
 **************************************************************************************************/

#include "inc/SX8723c.h"

void SX8723c_init(uint8_t device)
{	
	uint8_t device_number = 0;
	
	// device_number = address in EEPROM
	switch (device)
	{
		case device_1:
			device_number = EEPROM_SG_1;
		break;
		
		case device_2:
			device_number = EEPROM_SG_2;
		break;
		
		case device_3:
			device_number = EEPROM_SG_3;
		break;
		
		case device_4:
			device_number = EEPROM_SG_4;
		break;
		
		default:
			return;
		break;
	}
	
	//RegRCen register (0x30)
	sw_TWI_write_single_register(device, REG_RC_EN, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGCEN]);
	
	//RegOut register (0x40)
	sw_TWI_write_single_register(device, REG_OUT, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGOUT]);
	
	//RegTimeout register (0x42)
	sw_TWI_write_single_register(device, REG_TIMEOUT, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGTIMEOUT]);
								
	//RegExtAdd register (0x43)						
	sw_TWI_write_single_register(device, REG_EXT_ADD, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGEXTADD]);
	
	//AC_CFG0 register (0x52)
	sw_TWI_write_single_register(device, REG_AC_CFG0, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG0]);

	//AC_CFG1 register (0x53)													
	sw_TWI_write_single_register(device, REG_AC_CFG1, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG1]);
	
	//AC_CFG2 register (0x54)														
	sw_TWI_write_single_register(device, REG_AC_CFG2, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG2]);
	
	//AC_CFG3 register (0x55)
	sw_TWI_write_single_register(device, REG_AC_CFG3, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG3]);
	
	//AC_CFG4 register (0x56)
	sw_TWI_write_single_register(device,REG_AC_CFG4, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG4]);
	
	//AC_CFG5 register (0x57)
	sw_TWI_write_single_register(device, REG_AC_CFG5, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG5]);
	
	//RegMode register (0x70)
	sw_TWI_write_single_register(device, REG_MODE, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGMODE]);
}

void SX8723c_get_value(uint8_t device, int16_t* data)
{
	sw_TWI_read_value_registers(device, data);
}

void SX8723c_auto_offset(uint8_t device)
{
	int16_t SG_value[4] = {0};			// measurement value of the strain gauge sensor
	int16_t *p_SG_value = SG_value;		// pointer to measurement value
	int32_t temp = 0;					// temporary value for average
	uint8_t offset = 0;					// offset step size
	uint8_t device_number = 0;			// device_number = address in EEPROM
	
	switch (device)
	{
		case device_1:
			device_number = EEPROM_SG_1;
		break;
		
		case device_2:
			device_number = EEPROM_SG_2;
		break;
		
		case device_3:
			device_number = EEPROM_SG_3;
		break;
		
		case device_4:
			device_number = EEPROM_SG_4;
		break;
		
		default:
			return;
		break;
	}
	
	// reset offset
	sw_TWI_write_single_register(device, REG_AC_CFG4, 0x00);
	eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG4] = 0x00;
	_delay_ms(4);	//wait until new offset is set
	
	// get value before offset compensation (average) 
	for (uint16_t i = 0; i < SX8723c_DATA_AVERAGE; i++)
	{
		SX8723c_send_request(device);
		_delay_us(600);
		SX8723c_get_value(device, p_SG_value);	// get value	
		temp += SG_value[device_number];
		SG_value[device_number] = 0; 
	}
	SG_value[device_number] = temp / SX8723c_DATA_AVERAGE;
	
	//*********************** PGA3 offset **********************
	// check if offset compensation with PGA3 offset is possible 
	// the smallest step size is 5460 [digit] in both direction
	if ((SG_value[device_number] > TRESHOLD_PGA3_OFFSET)||(SG_value[device_number] < -TRESHOLD_PGA3_OFFSET))
	{
		// value is positive and offset must be negative 
		if (SG_value[device_number] > 0)	
		{
			// do compensation until value < threshold 
			while (SG_value[device_number] > TRESHOLD_PGA3_OFFSET)
			{
				// increment offset PGA3
				sw_TWI_write_single_register(device, REG_AC_CFG4, (0x40 | ++offset) );
				_delay_ms(4);	//wait until new offset is set
			
				// get new value after offset compensation (average)
				temp = 0;
				SG_value[device_number] = 0;
				for (uint16_t i = 0; i < SX8723c_DATA_AVERAGE; i++)
				{
					SX8723c_send_request(device);
					_delay_us(600);
					SX8723c_get_value(device, p_SG_value);	//get value
					temp += SG_value[device_number];
					SG_value[device_number] = 0; 
				}
				SG_value[device_number] = temp / SX8723c_DATA_AVERAGE;
				
				// maximal offset is 63 
				// if 63 is reached it is not possible to compensate that huge offset 
				// with PGA3 offset
				if (offset == 63)
				{
					return;
				}
			}
			eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG4] = (0x40 | offset);
		}
		
		// value is negative and offset must be positive 
		else
		{
			// do compensation until value < threshold 
			while (SG_value[device_number] < -TRESHOLD_PGA3_OFFSET)
			{
				// increment offset PGA3 
				sw_TWI_write_single_register(device, REG_AC_CFG4,  ++offset );
				_delay_ms(4);	//wait until new offset is set
				
				// get new value after offset compensation (average) 
				temp = 0;
				SG_value[device_number] = 0;
				for (uint16_t i = 0; i < SX8723c_DATA_AVERAGE; i++)
				{
					SX8723c_send_request(device);
					_delay_us(600);
					SX8723c_get_value(device, p_SG_value);	//get value
					temp += SG_value[device_number];
					SG_value[device_number] = 0; 
				}
				
				SG_value[device_number] = temp / SX8723c_DATA_AVERAGE;
				
				
				// maximal offset is 63 
				// if 63 is reached it is not possible to compensate that huge offset 
				// with PGA3 offset
				if (offset == 63)
				{
					return;
				}
			}	
			eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG4] = offset;
		}
	}	

	eeprom_sram_data.SG_software_offset[device_number] = SG_value[device_number];
}
				
void SX8723c_send_request(uint8_t device)
{
	uint8_t value = eeprom_sram_data.SX8723c_config[EEPROM_SG_1][EEPROM_SX8723C_REGACCFG0];	
	
	value &= 0xFE;
	value |= (1 << 7);
	
	sw_TWI_write_single_register(device, REG_AC_CFG0, value);
}

void SX8723c_sleep(uint8_t device)
{
	STRAIN_GAUGE_OFF;	// turn voltage of strain gauges off
	
	//RegRCen register (0x30)
	sw_TWI_write_single_register(device, REG_RC_EN, 0x00);
	
	//AC_CFG1 register (0x53)	
	sw_TWI_write_single_register(device, REG_AC_CFG1, 0x00);
}

void SX8723c_start(uint8_t device)
{
	uint8_t device_number = 0;
	
	// device_number = address in EEPROM
	switch (device)
	{
		case device_1:
			device_number = EEPROM_SG_1;
		break;
		
		case device_2:
			device_number = EEPROM_SG_2;
		break;
		
		case device_3:
			device_number = EEPROM_SG_3;
		break;
		
		case device_4:
			device_number = EEPROM_SG_4;
		break;
		
		case device_all:
			device_number = EEPROM_SG_1;
		break;
		
		default:
			return;
		break;
	}
	
	//RegRCen register (0x30)
	sw_TWI_write_single_register(device, REG_RC_EN, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGCEN]);
	
	//AC_CFG1 register (0x53)													
	sw_TWI_write_single_register(device, REG_AC_CFG1, eeprom_sram_data.SX8723c_config[device_number][EEPROM_SX8723C_REGACCFG1]);
	
	STRAIN_GAUGE_ON;	// turn voltage of strain gauges on
}

void SX8723c_set_PGA3_offset(uint8_t device, sign_t sign, uint8_t offset)
{	
	uint8_t value = 0;
	
	sw_TWI_read_single_register(device, REG_AC_CFG4, &value);
		
	value &= 0x80;
	// sign of the offset
	value |= sign << 6;
	//value of the offset
	value |= offset << 0;
	
	sw_TWI_write_single_register(device, REG_AC_CFG4, value);
}