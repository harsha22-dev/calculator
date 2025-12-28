/**\file *******************************************************************************************
 * 
 * \brief LED configuration.
 *
 * \details Functional description:
 *
 * <b>Target Platforms:</b> ATmega324P
 *
 * <b>Editor:</b>           Atmel AVR Studio 5 (Version: 5.1.208)
 *
 * <b>Compiler:</b>			AVRGCC (Version: 3.3.1.27) 
 * 
 * \date 2013-07-12
 * \version WEPO41104A02
 * \author created by flm, mih, job, ank
 *
 * <b>Copyright &copy;2013 pro-micron GmbH & Co. KG, All rights reserved.</b>
 *
 **************************************************************************************************/


#ifndef CONF_LED_H_
#define CONF_LED_H_

#define INIT_LED	{DDRD |= (1 << PIND4); PORTD |= (1 << PIND4);}
#define	LED_OFF		PORTD |= (1 << PIND4);
#define LED_ON		PORTD &= ~(1 << PIND4);

#endif /* CONF_LED_H_ */