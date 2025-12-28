/* This header file is part of the ATMEL AVR32-SoftwareFramework-1.3.0-AT32UC3A Release */

/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Standard board header file.
 *
 * This file includes the appropriate board header file according to the
 * defined board.
 *
 * - Compiler:           GCC
 * - Supported devices:  All AVR32 devices can be used.
 * - AppNote:
 *
 * \author               flm
 *
 ******************************************************************************/

#ifndef _read_board_H_
#define _read_board_H_

#include <avr32/io.h>
#include "compiler.h"

// RS485, MAX3491 definitons
#define RS485_USART                         (&AVR32_USART0)
#define RS485_USART_RX_PIN                  AVR32_USART0_RXD_0_0_PIN
#define RS485_USART_RX_FUNCTION             AVR32_USART0_RXD_0_0_FUNCTION
#define RS485_USART_TX_PIN                  AVR32_USART0_TXD_0_0_PIN
#define RS485_USART_TX_FUNCTION             AVR32_USART0_TXD_0_0_FUNCTION
#define RS485_USART_RTS_PIN                 AVR32_USART0_RTS_0_0_PIN
#define RS485_USART_RTS_FUNCTION            AVR32_USART0_RTS_0_0_FUNCTION

#define RS485_USART_RX_EN_ON                {};//gpio_clr_gpio_pin(AVR32_PIN_PB00);
#define RS485_USART_RX_EN_OFF               {};//gpio_set_gpio_pin(AVR32_PIN_PB00);

#define RS485_USART_TX_EN_ON                gpio_set_gpio_pin(AVR32_PIN_PA03);
#define RS485_USART_TX_EN_OFF               gpio_clr_gpio_pin(AVR32_PIN_PA03);

#define RS485_USART_IRQ                     AVR32_USART0_IRQ
#define RS485_USART_BAUDRATE                4000000

#define AVR32_PDCA_PID_USART_TX  		   	AVR32_PDCA_PID_USART0_TX
#define PDCA_CHANNEL_USART_TX				0

#define AVR32_PDCA_PID_USART_RX  		   	AVR32_PDCA_PID_USART0_RX
#define PDCA_CHANNEL_USART_RX 				1


// SPI Connections: CC2500 radio chip
#define CC2500_SPI                      (&AVR32_SPI0)
#define CC2500_SPI_NPCS                 0
#define CC2500_SPI_SCK_PIN              AVR32_SPI0_SCK_0_0_PIN
#define CC2500_SPI_SCK_FUNCTION         AVR32_SPI0_SCK_0_0_FUNCTION
#define CC2500_SPI_MISO_PIN             AVR32_SPI0_MISO_0_0_PIN
#define CC2500_SPI_MISO_FUNCTION        AVR32_SPI0_MISO_0_0_FUNCTION
#define CC2500_SPI_MOSI_PIN             AVR32_SPI0_MOSI_0_0_PIN
#define CC2500_SPI_MOSI_FUNCTION        AVR32_SPI0_MOSI_0_0_FUNCTION
#define CC2500_SPI_NPCS_PIN             AVR32_SPI0_NPCS_0_0_PIN
#define CC2500_SPI_NPCS_FUNCTION        AVR32_SPI0_NPCS_0_0_FUNCTION

// SPI Connections: AT86RF231 radio chip 1
#define RF231_SPI                       (&AVR32_SPI1)
#define RF231_SPI_NPCS                  2
#define RF231_SPI_SCK_PIN               AVR32_SPI1_SCK_0_0_PIN
#define RF231_SPI_SCK_FUNCTION          AVR32_SPI1_SCK_0_0_FUNCTION
#define RF231_SPI_MISO_PIN              AVR32_SPI1_MISO_0_0_PIN
#define RF231_SPI_MISO_FUNCTION         AVR32_SPI1_MISO_0_0_FUNCTION
#define RF231_SPI_MOSI_PIN              AVR32_SPI1_MOSI_0_0_PIN
#define RF231_SPI_MOSI_FUNCTION         AVR32_SPI1_MOSI_0_0_FUNCTION
#define RF231_SPI_NPCS_PIN              AVR32_PIN_PA19
#define RF231_SPI_NPCS_FUNCTION         AVR32_SPI1_NPCS_2_0_FUNCTION

#define SLP_TR                          AVR32_PIN_PA06
#define RST                             AVR32_PIN_PB17

#define EXT_INT_PIN_LINE1               AVR32_EIC_EXTINT_1_PIN
#define EXT_INT_FUNCTION_LINE1          AVR32_EIC_EXTINT_1_FUNCTION
#define EXT_INT_LINE1                   EXT_INT1
#define EXT_INT_NB_LINES                1
#define EXT_INT_IRQ_LINE1               AVR32_EIC_IRQ_1

// SPI Connections: AT86RF231 radio chip 2
#define RF231_SPI_2                     (&AVR32_SPI0)
#define RF231_SPI_NPCS_2                2
#define RF231_SPI_SCK_PIN_2             AVR32_SPI0_SCK_0_0_PIN
#define RF231_SPI_SCK_FUNCTION_2        AVR32_SPI0_SCK_0_0_FUNCTION
#define RF231_SPI_MISO_PIN_2            AVR32_SPI0_MISO_0_0_PIN
#define RF231_SPI_MISO_FUNCTION_2       AVR32_SPI0_MISO_0_0_FUNCTION
#define RF231_SPI_MOSI_PIN_2            AVR32_SPI0_MOSI_0_0_PIN
#define RF231_SPI_MOSI_FUNCTION_2       AVR32_SPI0_MOSI_0_0_FUNCTION
#define RF231_SPI_NPCS_PIN_2            AVR32_SPI0_NPCS_2_0_PIN
#define RF231_SPI_NPCS_FUNCTION_2       AVR32_SPI0_NPCS_2_0_FUNCTION

#define SLP_TR_2                        AVR32_PIN_PA05
#define RST_2                           AVR32_PIN_PB16

#define EXT_INT_PIN_LINE1_2             AVR32_EIC_EXTINT_0_PIN
#define EXT_INT_FUNCTION_LINE1_2        AVR32_EIC_EXTINT_0_FUNCTION
#define EXT_INT_LINE1_2                 EXT_INT0
#define EXT_INT_NB_LINES_2              1
#define EXT_INT_IRQ_LINE1_2             AVR32_EIC_IRQ_0

// SPI Connections of the AD5410 DAC-chip
#define AD5410_SPI                      (&AVR32_SPI1)
#define AD5410_SPI_NPCS                 0
#define AD5410_SPI_SCK_PIN              AVR32_SPI1_SCK_0_0_PIN
#define AD5410_SPI_SCK_FUNCTION         AVR32_SPI1_SCK_0_0_FUNCTION
#define AD5410_SPI_MISO_PIN             AVR32_SPI1_MISO_0_0_PIN
#define AD5410_SPI_MISO_FUNCTION        AVR32_SPI1_MISO_0_0_FUNCTION
#define AD5410_SPI_MOSI_PIN             AVR32_SPI1_MOSI_0_0_PIN
#define AD5410_SPI_MOSI_FUNCTION        AVR32_SPI1_MOSI_0_0_FUNCTION
#define AD5410_SPI_NPCS0_PIN            AVR32_SPI1_NPCS_0_0_PIN
#define AD5410_SPI_NPCS0_FUNCTION       AVR32_SPI1_NPCS_0_0_FUNCTION

/* PWM, Coil supply definitions */
#define COIL_PWM_PIN                    AVR32_PWM_5_1_PIN
#define COIL_PWM_FUNCTION               AVR32_PWM_5_1_FUNCTION
#define COIL_PWM_CHANNEL_ID             5
#define COIL_PWM_2_PIN                  AVR32_PWM_2_PIN
#define COIL_PWM_2_FUNCTION             AVR32_PWM_2_FUNCTION
#define COIL_PWM_2_CHANNEL_ID           2

#define PWM_TRANSCEIVER_ENABLE			AVR32_PIN_PB20

/* DAC definitions */
#define DAC_ADDRESS           0x4c        // EEPROM's TWI address
#define DAC_ADDR_LGT          1           // Address length of the EEPROM memory
#define VIRTUALMEM_ADDR_START 0x000000    // Address of the virtual mem in the EEPROM
#define TWI_SPEED             100000      // Speed of TWI
#define PATTERN_TEST_LENGTH   (sizeof(test_pattern)/sizeof(U8))

/* LED definitions */
#define LED_GREEN1_ON   gpio_clr_gpio_pin( AVR32_PIN_PB12 );    // green led on
#define LED_GREEN1_OFF  gpio_set_gpio_pin( AVR32_PIN_PB12 );    // green led off
#define LED_RED1_ON     gpio_clr_gpio_pin( AVR32_PIN_PB11 );    // red LED on
#define LED_RED1_OFF    gpio_set_gpio_pin( AVR32_PIN_PB11 );    // red LED on
#define LED_GREEN2_ON   gpio_clr_gpio_pin( AVR32_PIN_PB27 );    // green led on
#define LED_GREEN2_OFF  gpio_set_gpio_pin( AVR32_PIN_PB27 );    // green led off
#define LED_RED2_ON     gpio_clr_gpio_pin( AVR32_PIN_PB26 );    // red LED on
#define LED_RED2_OFF    gpio_set_gpio_pin( AVR32_PIN_PB26 );    // red LED on

/* TCA6424 IO expander */
#define TCA6424_TWI_SDA_PIN              AVR32_TWI_SDA_0_0_PIN
#define TCA6424_TWI_SDA_FUNCTION         AVR32_TWI_SDA_0_0_FUNCTION

#define TCA6424_TWI_SCL_PIN              AVR32_TWI_SCL_0_0_PIN
#define TCA6424_TWI_SCL_FUNCTION         AVR32_TWI_SCL_0_0_FUNCTION

#define TCA6424_TWI_RST_PIN              AVR32_SPI1_SCK_0_0_PIN

/* Digital Inputs */
#define IN0         AVR32_PIN_PA04
#define ENABLE_IN0 	gpio_enable_pin_pull_up(IN0); gpio_enable_pin_glitch_filter(IN0)
#define GET_IN0 	gpio_get_pin_value(IN0)


// USB
#define USB_ID                      AVR32_USBB_USB_ID_0_0
#define USB_VBOF                    AVR32_USBB_USB_VBOF_0_1
#define USB_VBOF_ACTIVE_LEVEL       LOW
#define USB_OVERCURRENT_DETECT_PIN  AVR32_PIN_PX33

// SDRAM
#define SDRAM_PART_HDR  "MT48LC16M16A2TG7E/mt48lc16m16a2tg7e.h"
#define SDRAM_DBW       16

// Oscillator
#define FOSC32          32768                                 //!< Osc32 frequency: Hz.
#define OSC32_STARTUP   AVR32_PM_OSCCTRL32_STARTUP_8192_RCOSC //!< Osc32 startup time: RCOsc periods.
#define FOSC0           12000000                              //!< Osc0 frequency: Hz.
#define OSC0_STARTUP    AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.
#ifndef FRCOSC
  #define FRCOSC    AVR32_PM_RCOSC_FREQUENCY  //!< Default RCOsc frequency.
#endif

#endif  // _BOARD_H_
