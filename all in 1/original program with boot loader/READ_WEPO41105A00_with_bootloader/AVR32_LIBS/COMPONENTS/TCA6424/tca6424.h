#include "main_app.h"

extern volatile uint32_t LEDs;

uint8_t tca6424_init(void);
uint8_t tca6424_write_data(uint8_t reg_addr, uint8_t data);
uint8_t tca6424_output(uint32_t output);
void LED_on(uint32_t led);
void LED_off(uint32_t led);


//LEDs

#define LED2or	0x00000001
#define LED2ur	0x00000002
#define LED2og	0x00000004
#define LED2ug	0x00000008

#define LED3or	0x00000010
#define LED3ur	0x00000020

#define LED4or	0x00000040
#define LED4ur	0x00000080

#define LED5or	0x00000100
#define LED5ur	0x00000200
#define LED5og	0x00000400
#define LED5ug	0x00000800

#define LED6or	0x00001000
#define LED6ur	0x00002000
#define LED6og	0x00004000
#define LED6ug	0x00008000

#define LED7og	0x00010000
#define LED7ug	0x00020000

#define LED8og	0x00040000
#define LED8ug	0x00080000

#define LED9ob	0x00100000
#define LED9ub	0x00200000
