/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief This file implements the API for the AT86RF231 radio
 *        transceiver.
 *
 *
 ******************************************************************************/
#define TAL_TYPE  AT86RF231
#define SHORTENUM           __attribute__((packed))
#ifndef AT86_H_
#define AT86_H_
#define RF_BAND  BAND_2400
#define ENABLE_TRX_SRAM

/*============================ INCLUDE =======================================*/
#include <stdint.h>             // std typedefs
#include <stdbool.h>            // std bool types
#include "at86rf231.h"          // AT86RF231 Registermap
#include "tal_constants.h"      // Tal definitions
#include "ieee_const.h"         // IEEE definitions
#include "pal_config.h"         // Platfrom abstraction layer config
#include <avr/io.h>             // Registerdefinitionen
#include <inttypes.h>           // Datatypes
#include <avr/Interrupt.h>      // Interrupts
#include <util/delay.h>         // Delays

#define SAMPLES_PER_PACKET  7

/* === Typedefs =============================================================== */

/*!
 * \brief AT86RF231 Frame struct including frame header
 */
typedef struct
{
    uint8_t  frame_length;
    uint16_t frame_control_field;
    uint8_t  seq_number;
    uint16_t dest_pan_id;
    uint16_t dest_addr;
    uint16_t src_pan_id;
    uint16_t src_addr;
    volatile uint8_t  payload[aMaxPHYPacketSize-13];
}at86rf231_frame;

/*!
 * \brief Spike tx payload data struct
 */
typedef struct
{
    volatile int16_t strain_gauge[SAMPLES_PER_PACKET][4];
    volatile uint16_t counter;
    volatile int16_t temperature;
    volatile uint8_t voltage;
}spike_payload;

/*!
 * \brief AT86RF231 Transceiver states
 */
typedef enum tal_state_tag
{
    TAL_IDLE           = 0,
    TAL_TX_AUTO        = 1,
    TAL_TX_DONE        = 2
} SHORTENUM tal_state_t;

/**
 * Transceiver interrupt handler index
 */
typedef enum trx_irq_hdlr_idx_tag
{
    TRX_MAIN_IRQ_HDLR_IDX = 0,
    TRX_TSTAMP_IRQ_HDLR_IDX,
    TRX_ALTERNATE_0_IRQ_HDLR_IDX, /* Spare IRQ for devices with multiple irqs */
    TRX_ALTERNATE_1_IRQ_HDLR_IDX, /* Spare IRQ for devices with multiple irqs */
    TRX_ALTERNATE_2_IRQ_HDLR_IDX, /* Spare IRQ for devices with multiple irqs */
    TRX_ALTERNATE_3_IRQ_HDLR_IDX, /* Spare IRQ for devices with multiple irqs */
    TRX_ALTERNATE_4_IRQ_HDLR_IDX, /* Spare IRQ for devices with multiple irqs */
    TRX_ALTERNATE_5_IRQ_HDLR_IDX, /* Spare IRQ for devices with multiple irqs */
    TRX_ALTERNATE_6_IRQ_HDLR_IDX, /* Spare IRQ for devices with multiple irqs */
    TRX_ALTERNATE_7_IRQ_HDLR_IDX  /* Spare IRQ for devices with multiple irqs */
} trx_irq_hdlr_idx_t;

/**
 * Transceiver csma_mode_tag
 */
typedef enum csma_mode_tag
{
    NO_CSMA_NO_IFS,
    NO_CSMA_WITH_IFS,
    CSMA_UNSLOTTED,
    CSMA_SLOTTED
} csma_mode_t;

/**
 * This is a typedef of the function which is called from the transceiver ISR
 */
typedef void (*irq_handler_t)(void);

/* === Definitions =============================================================== */

/*
 *  Macro to set Reset pin to high
 */
#define PAL_RST_HIGH()                  RST_HIGH()

/*
 * Macro to set Reset pin to low
 */
#define PAL_RST_LOW()                   RST_LOW()

/*
 *  Macro to set SLP_TR pin to high
 */
#define PAL_SLP_TR_HIGH()               SLP_TR_HIGH()

/*
 *  Macro to set SLP_TR pin to low
 */
#define PAL_SLP_TR_LOW()                SLP_TR_LOW()

/*
 * Write access command of the transceiver
 */
#define WRITE_ACCESS_COMMAND            (0xC0)

/*
 * Read access command to the tranceiver
 */
#define READ_ACCESS_COMMAND             (0x80)

/*
 * Frame write command of transceiver
 */
#define TRX_CMD_FW                      (0x60)

/*
 * Frame read command of transceiver
 */
#define TRX_CMD_FR                      (0x20)

/*
 * SRAM write command of transceiver
 */
#define TRX_CMD_SW                      (0x40)

/*
 * SRAM read command of transceiver
 */
#define TRX_CMD_SR                      (0x00)

/*========================= PROTOTYPES =======================================*/

/**
 * \fn void delay_us(unsigned int usec);
 *
 * \brief delay for specified time in us
 * \details uses "nop" instruction
 * \param uint16_t usec
 */
void delay_us( uint16_t usec);

/**
 * \fn void trx_interface_init(void);
 *
 * \brief init spi interace
 */
void trx_interface_init(void);

/**
 * \fn void at86rf231_init( void );
 *
 * \brief init AT86RF231 radio cip
 */
void at86rf231_init( void );

/**
 * \fn void pal_trx_irq_init(trx_irq_hdlr_idx_t trx_irq_num, void *trx_irq_cb);
 *
 * \brief init external interrupt
 * \param Interrupt number
 * \param Interrupt callback function
 */
void pal_trx_irq_init(trx_irq_hdlr_idx_t trx_irq_num, void *trx_irq_cb);

/**
 * \fn void trx_irq_handler_cb(void);
 *
 * \brief external interrupt handler
 */
void trx_irq_handler_cb(void);

/**
 * \fn void handle_received_frame_irq(void);
 *
 * \brief Interrupt handler for received frames
 */
void handle_received_frame_irq(void);

/**
 * \fn uint8_t pal_trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos);
 *
 * \brief read specified bit from transceiver
 */
uint8_t pal_trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos);

/**
 * \fn void pal_trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t new_value);
 *
 * \brief write specified bit to transceiver
 */
void pal_trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t new_value);

/**
 * \fn uint8_t pal_trx_reg_read(uint8_t addr);
 *
 * \brief read register from transceiver
 */
uint8_t pal_trx_reg_read(uint8_t addr);

/**
 * \fn void pal_trx_reg_write(uint8_t addr, uint8_t data);
 *
 * \brief write register to transceiver
 */
void pal_trx_reg_write(uint8_t addr, uint8_t data);

/**
 * \fn void pal_trx_frame_read(uint8_t *data, uint8_t length);
 *
 * \brief Upload frame from transceiver
 */
void pal_trx_frame_read(uint8_t *data, uint8_t length);

/**
 * \fn void pal_trx_frame_write(uint8_t *data, uint8_t length);
 *
 * \brief write frame to transceiver
 */
void pal_trx_frame_write(uint8_t *data, uint8_t length);

/**
 * \fn void at86rf231_rx_with_auto_ack( void );
 *
 * \brief receive frames with auto acknowledge
 */
void at86rf231_rx_with_auto_ack( void );

/**
 * \fn void at86rf231_tx(void);
 *
 * \brief transmit frames fast, without retry
 * \details manual state transition  necessary
 */
void at86rf231_tx(void);

/**
 * \fn void at86rf231_tx_with_retry( csma_mode_t csma_mode, bool tx_retries );
 *
 * \brief tx frames with retry
 * \details csma/cd not working yet
 */
void at86rf231_tx_with_retry( csma_mode_t csma_mode, bool tx_retries );

/**
 * \fn void at86rf231_req_data( void );
 *
 * \brief send data request command and wait for data
 * \details auto retransmit and autoack used
 */
void at86rf231_req_data( void );

/**
 * \fn void at86rf231_sleep( void );
 *
 * \brief set sleep mode
 */
void at86rf231_sleep( void );

/**
 * \fn void at86rf231_switch_pll_on(void);
 *
 * \brief set transceiver state = pll on
 * \details blocks if pll does not lock
 */
uint8_t at86rf231_switch_pll_on(void);

/**
 * \fn uint8_t at86rf231_change_channel( uint8_t new_channel );
 *
 * \brief change transceiver channel
 */
uint8_t at86rf231_change_channel( uint8_t new_channel );

/**
 * \fn bool at86rf231_change_channel_page(uint8_t ch_page);
 *
 * \brief change transceiver channel page
 */
bool at86rf231_change_channel_page(uint8_t ch_page);

/**
 * \fn uint8_t at86rf231_change_tx_pwr( uint8_t pwr );
 *
 * \brief change transceiver tx power
 */
uint8_t at86rf231_change_tx_pwr( uint8_t pwr );

/**
 * \fn void at86rf231_ftn_pll_calibration(void);
 *
 * \brief filter and pll calibration
 * \details should be performed every 5 minutes in 2MBit-mode
 */
void at86rf231_ftn_pll_calibration(void);

/**
 * \brief Enables the transceiver interrupt
 *
 * \param trx_irq_num One of several interrupt lines provided by the transceiver
 * \ingroup apiPalApi
 */
static inline void pal_trx_irq_enable(trx_irq_hdlr_idx_t trx_irq_num)
{
    ENABLE_TRX_IRQ(trx_irq_num);
}

/**
 * \brief Disables the transceiver interrupt
 *
 * \param trx_irq_num One of several interrupt lines provided by the transceiver
 * \ingroup apiPalApi
 */
static inline void pal_trx_irq_disable(trx_irq_hdlr_idx_t trx_irq_num)
{
    DISABLE_TRX_IRQ(trx_irq_num);
}

/**
 * \brief Clears the transceiver interrupt
 *
 * \param trx_irq_num One of several interrupt lines provided by the transceiver
 * \ingroup apiPalApi
 */
static inline void pal_trx_irq_flag_clr(trx_irq_hdlr_idx_t trx_irq_num)
{
    CLEAR_TRX_IRQ(trx_irq_num);
}

/**
 * \brief Enables the global interrupt
 * \ingroup apiPalApi
 */
static inline void pal_global_irq_enable(void)
{
    ENABLE_GLOBAL_IRQ();
}

/**
 * \brief Disables the global interrupt
 * \ingroup apiPalApi
 */
static inline void pal_global_irq_disable(void)
{
    DISABLE_GLOBAL_IRQ();
}

#endif
/*EOF*/
