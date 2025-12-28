/**\file *******************************************************************************************
 * 
 * \brief Semtech SX8723c Service
 *
 * \details Functional description:
 *
 *          The Semtech SX8723c module includes project-specific definitions
 *			and function prototypes, such as the SX8723c initialization function.
 *
 * <b>Target Platforms:</b> ATmega324PV (other Controller/independent)
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

#ifndef SX8723c_H_
#define SX8723c_H_

/**
 * \defgroup group_SX8723c_service Semtech SX8723c
 *
 * The Semtech SX8723c module includes project-specific definitions
 * and function prototypes, such as the SX8723c initialization function.
 *
 * \{
 */

//-----------------------------------------------------------------------------------------------//
// Header Files
//-----------------------------------------------------------------------------------------------//
#include <inttypes.h>           

// CPU clock
#ifndef F_CPU
#define F_CPU                   (10000000UL)
#endif
#include <util/delay.h>         // Delays

#include "sw_twi.h"
#include "eeprom_abbild.h"

//-----------------------------------------------------------------------------------------------//
// Symbolic Constants, Defines and Macros
//-----------------------------------------------------------------------------------------------//

#define STRAIN_GAUGE_ON		PORTD &= ~(1 << PIND7)	//!< Pin to turn the supply of the strain gauges on
#define STRAIN_GAUGE_OFF	PORTD |= (1 << PIND7)	//!< Pin to turn the supply of the strain gauges off

#define TRESHOLD_PGA3_OFFSET		2800	//!< this is the high level of the hardware offset
#define TRESHOLD_PGA3_OFFSET_HIGH	2800	//!< this is the high level of the hardware offset
#define TRESHOLD_PGA3_OFFSET_LOW	-2800	//!< this is the low level of the hardware offset
#define SX8723c_DATA_AVERAGE		100		//!< maximal 65536, average in auto-offset function

/*! \name RC Register
 */
//! @{
#define REG_RC_EN		0x30	//!< RC oscillator control
//! @}

/*! \name GPIO Registers
 */
//! @{
#define REG_OUT			0x40	//!< D0 and D1 pads data output and direction control
#define REG_IN			0x41	//!< D0 and D1 pads input data
#define REG_TIMEOUT		0x42	//!< Enable/Disable I2C timeout
#define REG_EXT_ADD		0x43	//!< Set address by external pin
//! @}

/*! \name ADC Registers
 */
//! @{
#define REG_AC_OUT_LSB	0x50	//!< LSB of ADC result
#define REG_AC_OUT_MSB	0x51	//!< MSB of ADC result
#define REG_AC_CFG0		0x52	//!< ADC conversion control
#define REG_AC_CFG1		0x53	//!< ADC conversion control
#define REG_AC_CFG2		0x54	//!< ADC conversion control
#define REG_AC_CFG3		0x55	//!< ADC conversion control
#define REG_AC_CFG4		0x56	//!< ADC conversion control
#define REG_AC_CFG5		0x57	//!< ADC conversion control
//! @}

/*! \name Mode Register
 */
//! @{
#define REG_MODE		0x70	//!< Chip operating mode register
//! @}

/*! \name Defines for RegRCen register (0x30)
 */
//! @{
#define RC_DISABLE	(0 << 0) //!< Disable RC oscillator. Set 0 for low power mode.
#define RC_ENABLE	(1 << 0) //!< Enable RC oscillator.
//! @}

/*! \name Defines for RegQut register (0x40)
 */
//! @{
#define D0_DIR_INPUT			(0 << 4) //!< D0 pad direction as input.
#define D0_DIR_OUTPUT			(1 << 4) //!< D0 pad direction as output.
#define D1_DIR_INPUT			(0 << 5) //!< D1 pad direction as input.
#define D1_DIR_OUTPUT			(1 << 5) //!< D1 pad direction as output.
#define REG_OUT_RESERVED_PIN	(0 << 0)|(0 << 1)|(1 << 6)|(1 << 7) //!< Please write this reset values every time. 
//! @}

/*! \name Defines for RegTimeout register (0x42)
 */
//! @{
#define I2C_TIMEOUT_DISABLE	(0 << 5) //!< D0 pad direction as input.
#define I2C_TIMEOUT_ENABLE	(1 << 5) //!< D0 pad direction as output.
#define REG_TIMEOUT_RESERVED_PIN	(0 << 0)|(0 << 1)|(0 << 2)|(0 << 3)|(0 << 4)|(0 << 6)|(0 << 7) //!< Please write this reset values every time. 
//! @}

/*! \name Defines for RegExtAdd register (0x43)
 */
//! @{
#define INTERNAL_I2C_ADDRESS	0x00 //!< Use standard I2C address.
#define EXTERNAL_I2C_ADDRESS	0x96 //!< Set two LSbits of the I2C address by external (D0 and D1).
//! @}

/*! \name Defines for RegACCfg0 register (0x52)
 */
//! @{
#define REG_ACCFG0_RESERVED_PIN	(0 << 0) //!< Please write this reset values every time.
#define SET_CONTINUOUS_MODE		(1 << 1) //!< Sets the continuous ADC conversion mode.
#define OSR_8					(0 << 2) //!< Sets the ADC over-sampling rate = 8.
#define OSR_16					(1 << 2) //!< Sets the ADC over-sampling rate = 16.
#define OSR_32					(2 << 2) //!< Sets the ADC over-sampling rate = 32.
#define OSR_64					(3 << 2) //!< Sets the ADC over-sampling rate = 64.
#define OSR_128					(4 << 2) //!< Sets the ADC over-sampling rate = 128.
#define OSR_256					(5 << 2) //!< Sets the ADC over-sampling rate = 256.
#define OSR_512					(6 << 2) //!< Sets the ADC over-sampling rate = 512.
#define OSR_1024				(7 << 2) //!< Sets the ADC over-sampling rate = 1024.
#define	NELCONV_1				(0 << 5) //!< Nummer of elementary conversation = 1.
#define NELCONV_2				(1 << 5) //!< Nummer of elementary conversation = 2.
#define NELCONV_4				(2 << 5) //!< Nummer of elementary conversation = 4.
#define NELCONV_8				(3 << 5) //!< Nummer of elementary conversation = 8.
#define START_CONVERSION		(1 << 7) //!< Starts an ADC conversion.
//! @}

/*! \name Defines for RegACCfg1 register (0x53)
 */
//! @{
#define ADC_DISABLE		(0 << 0) //!< Disable ADC.
#define ADC_ENABLE		(1 << 0) //!< Enable ADC.
#define PGA1_DISABLE	(0 << 1) //!< Disable PGA1.
#define PGA1_ENABLE		(1 << 1) //!< Enable PGA1.
#define PGA2_DISABLE	(0 << 2) //!< Disable PGA2.
#define PGA2_ENABLE		(1 << 2) //!< Enable PGA2.
#define PGA3_DISABLE	(0 << 3) //!< Disable PGA3.
#define PGA3_ENABLE		(1 << 3) //!< Enable PGA3.
#define IB_AMP_PGA_25	(0 << 4) //!< Bias current for the PGA = 25%.
#define IB_AMP_PGA_50	(1 << 4) //!< Bias current for the PGA = 50%.
#define IB_AMP_PGA_75	(2 << 4) //!< Bias current for the PGA = 75%.
#define IB_AMP_PGA_100	(3 << 4) //!< Bias current for the PGA = 100%.
#define IB_AMP_ADC_25	(0 << 6) //!< Bias current for the ADC = 25%.
#define IB_AMP_ADC_50	(1 << 6) //!< Bias current for the ADC = 50%.
#define IB_AMP_ADC_75	(2 << 6) //!< Bias current for the ADC = 75%.
#define IB_AMP_ADC_100	(3 << 6) //!< Bias current for the ADC = 100%.
//! @}

/*! \name Defines for RegACCfg2 register (0x54)
 */
//! @{
#define PGA2_GAIN_1		(0 << 4) //!< PGA2 gain = 1.
#define PGA2_GAIN_2		(1 << 4) //!< PGA2 gain = 2.
#define PGA2_GAIN_5		(2 << 4) //!< PGA2 gain = 5.
#define PGA2_GAIN_10	(3 << 4) //!< PGA2 gain = 10.
#define OSF_62K5HZ		(0 << 6) //!< ADC Sampling Frequency = 62.5 kHz.
#define OSF_125KHZ		(1 << 6) //!< ADC Sampling Frequency = 125 kHz.
#define OSF_250KHZ		(2 << 6) //!< ADC Sampling Frequency = 250 kHz.
#define OSF_500KHZ		(3 << 6) //!< ADC Sampling Frequency = 500 kHz.
//! @}

/*! \name Defines for RegACCfg3 register (0x55)
 */
//! @{
#define PGA1_GAIN_1		(0 << 7) //!< PGA1 gain = 1.
#define PGA1_GAIN_10	(1 << 7) //!< PGA1 gain = 10.
//! @}

/*! \name Defines for RegACCfg4 register (0x56)
 */
//! @{
#define REG_ACCFG4_RESERVED_PIN	(0 << 7) //!< Please write this reset values every time.
//! @}

/*! \name Defines for RegACCfg5 register (0x57)
 */
//! @{
#define VMUX_VBATT		(0 << 0) //!< Reference is Vbatt.
#define VMUX_VREF		(1 << 0) //!< Reference is Vref.
#define AMUX_AC0_AC1	(0 << 1) //!< Channel: AC0 (VSS) - AC1 (VREF).
#define AMUX_AC1_AC0	(0 << 1) //!< Channel: AC1 (VREF) - AC0 (VSS).
#define AMUX_AC2_AC3	(1 << 1) //!< Channel: AC2 - AC2.
#define AMUX_AC3_AC2	(1 << 1) //!< Channel: AC3 - AC2.
#define AMUX_AC5_AC4	(2 << 1) //!< Channel: AC5 - AC4. 
#define AMUX_AC4_AC5	(2 << 1) //!< Channel: AC4 - AC5. 
#define AMUX_AC0		(0 << 1) //!< Channel: AC0 (VSS) - AC0 (VSS). 
#define AMUX_AC1		(1 << 1) //!< Channel: AC1 (VREF) - AC0 (VSS). 
#define AMUX_AC2		(2 << 1) //!< Channel: AC2 - AC0 (VSS). 
#define AMUX_AC3		(3 << 1) //!< Channel: AC3 - AC0 (VSS).
#define AMUX_AC4		(4 << 1) //!< Channel: AC4 - AC0 (VSS). 
#define AMUX_AC5		(5 << 1) //!< Channel: AC5 - AC0 (VSS). 
#define AMUX_SIGN_STRAIGHT		(0 << 4) //!< Sign is straight.
#define AMUX_SIGN_CROSS			(1 << 4) //!< Sign is cross.
#define AMUX_MODE_DIFFERENTIAL_INPUT	(0 << 5) //!< Mode: Differential input.
#define AMUS_MODE_SINGLE_ENDED_INPUT	(1 << 5) //!< Mode: Single ended input.
#define DEF_CONFIG						(1 << 6) //!< Selects ADC and PGA default configuration
//! @}

/*! \name Defines for RegMode register (0x70)
 */
//! @{
#define VREF_D1_IN_ENABLE		(1 << 0) //!< Enable external Vref on D1 pin.
#define VREF_D0_OUT_ENABLE		(1 << 1) //!< Enable Vref output on D0 pin.
#define MULTI_FORCE_OFF			(1 << 2) //!< Force charge pump Off.
#define MULTI_FORCE_ON			(1 << 3) //!< Force charge pump On. Takes priority.
#define CHOP_0					(0 << 4) //!< Chopping control state = 0. 
#define CHOP_1					(1 << 4) //!< Chopping control state = 1. 
#define CHOP_NELCONV			(2 << 4) //!< Chopping control at Nelconv rate. 
#define CHOP_NELCONV_0_5		(3 << 4) //!< Chopping control at Nelconv/2 rate. 
#define REG_MODE_RESERVED_PIN	(0 << 6)|(1 << 7) //!< Please write this reset values every time.
//! @}

//-----------------------------------------------------------------------------------------------//
// Global variables
//-----------------------------------------------------------------------------------------------//

extern eeprom_data eeprom_sram_data;
extern eeprom_data *ptr_eeprom_sram_data;

//-----------------------------------------------------------------------------------------------//
// Typedefs, enums and structs
//-----------------------------------------------------------------------------------------------//

/**
* Sign of a value.
*/
typedef enum {
  SIGN_POSITIV			=		0,	/**< Sign positive (+). */
  SIGN_NEGATIVE			=		1	/**< Sign negative (-). */
} sign_t;

//-----------------------------------------------------------------------------------------------//
// Prototypes
//-----------------------------------------------------------------------------------------------//

/*! \brief This function initialize the SX8723c.
 * \param device Strain gauge device.
 */
void SX8723c_init(uint8_t device);

/*! \brief This function returns the actually measurement value.
 * \param device Strain gauge device.
 * \param data Pointer to a data field as return value.
 */
void SX8723c_get_value(uint8_t device, int16_t* data);

/*! \brief This function starts an conversation on the SX8723c.
 * \param device Strain gauge device.
 */
void SX8723c_send_request(uint8_t device);

/*! \brief This function compensate an offset automatically.
 * \details Use only PGA3 offset. It is only possible to adjust one bridge at the time. 
 *          Save offset values in the EEPROM. Preforms a hardware and software offset.
 * \param device Strain gauge device.
 */
void SX8723c_auto_offset(uint8_t device);

/*! \brief This function sets the SX8723c in sleep mode.
 * \param device Strain gauge device.
 */
void SX8723c_sleep(uint8_t device);

/*! \brief This function sets the SX8723c in normal mode (back from sleep mode).
 * \param device Strain gauge device.
 */
void SX8723c_start(uint8_t device);

/*! \brief This function sets an new PGA3 offset.
 * \param device Strain gauge device.
 * \param sign Sign can be positive or negative.
 * \param offset New offset value.
 */
void SX8723c_set_PGA3_offset(uint8_t device, sign_t sign, uint8_t offset);

/**
 * \}
 */

#endif /* SX8723c_H_ */