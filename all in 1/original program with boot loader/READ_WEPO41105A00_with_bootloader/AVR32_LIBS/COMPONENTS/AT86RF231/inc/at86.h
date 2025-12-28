/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief This file implements the HAL API for the AT86RF230 radio
 *        transceiver.
 *
 * \defgroup hal_avr_api Hardware Abstraction Layer API Functions
 *      This set of functions (Some defined as macros) is the API for the
 *      Hardware Abstraction Layer. These functions gives complete access to
 *      all of the low level functionality of the radio transceiver
 *      (IO, SPI and ISR).
 *
 ******************************************************************************/
#define TAL_TYPE  AT86RF231
#define SHORTENUM           __attribute__((packed))
#ifndef AT86_H_
#define AT86_H_
#define RF_BAND  BAND_2400

/*============================ INCLUDE =======================================*/
#include <stdbool.h>
#include "at86rf231.h"
#include "tal_internal.h"
#include "tal_constants.h"
#include "ieee_const.h"
#include "board.h"

/* === Macros =============================================================== */

typedef union
__attribute__((__packed__))
{
    /* frame structure */
    struct {
        /* frame length */
        uint8_t  frame_length;
        /* fcf*/
        uint16_t frame_control_field;
        /* sequence number */
        uint8_t  seq_number;
        /* Destination Pan id of frame */
        uint16_t dest_pan_id;
        /* Destination address of frame */
        uint16_t dest_addr;
        /* Src pan id */
        uint16_t src_pan_id;
        /* Source address of frame */
        uint16_t src_addr;
        /* Payload Array */
        uint8_t  payload[aMaxPHYPacketSize - 13];
    }__attribute__((__packed__)) at86rf231_frame;

    /* data in bytes */
    uint8_t rf231_array[135];
}frame_info_union;

typedef struct
__attribute__((__packed__))
{
    /* RSSI of received frame */
    int16_t rssi_dbm;
    /* Packet lenght of received frame */
    uint8_t packet_length;
    /* time of received frame */
    uint32_t time;
    /* Pointer to received frame */
    frame_info_union *frame_ptr;
}rf231_rec_struct;





static inline unsigned short bswap_16(unsigned short x) {
  return (x>>8) | (x<<8);
}
static inline unsigned int bswap_32(unsigned int v)
{
         return ((v & 0xff) << 24) | ((v & 0xff00) << 8) |
                  ((v & 0xff0000) >> 8) | (v >> 24);
}

#define ENTER_CRITICAL_REGION()         taskENTER_CRITICAL();
#define LEAVE_CRITICAL_REGION()         taskEXIT_CRITICAL();

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

/**
 * This is a typedef of the function which is called from the transceiver ISR
 */
typedef void (*irq_handler_t)(void);

typedef enum csma_mode_tag
{
    NO_CSMA_NO_IFS,
    NO_CSMA_WITH_IFS,
    CSMA_UNSLOTTED,
    CSMA_SLOTTED
} csma_mode_t;

/**
 * These are the return values of the PAL API.
 */
typedef enum
#if !defined(DOXYGEN)
retval_tag
#endif
{
    MAC_SUCCESS                 = 0x00,
    TAL_TRX_ASLEEP              = 0x81,
    TAL_TRX_AWAKE               = 0x82,
    TAL_CRC_CORRECT             = 0x83,
    TAL_CRC_INCORRECT           = 0x84,
    FAILURE                     = 0x85,
    TAL_BUSY                    = 0x86,
    TAL_FRAME_PENDING           = 0x87,
    /** Return code, if the timer that should be started is already running */
    PAL_TMR_ALREADY_RUNNING     = 0x88,
    /** Return code, if the timer that should be stopped is not running */
    PAL_TMR_NOT_RUNNING         = 0x89,
    /** Return code, if the requested timer ID is invalid */
    PAL_TMR_INVALID_ID          = 0x8A,
    /** Return code, if the specified timeout is out of range or too short */
    PAL_TMR_INVALID_TIMEOUT     = 0x8B,
    QUEUE_FULL                  = 0x8C,
    TAL_CSMA_CA_IN_PROGRESS     = 0x8D,
    MAC_BEACON_LOSS             = 0xE0,
    MAC_CHANNEL_ACCESS_FAILURE  = 0xE1,
    MAC_DISABLE_TRX_FAILURE     = 0xE3,
    MAC_FRAME_TOO_LONG          = 0xE5,
    MAC_INVALID_GTS             = 0xE6,
    MAC_INVALID_HANDLE          = 0xE7,
    MAC_INVALID_PARAMETER       = 0xE8,
    MAC_NO_ACK                  = 0xE9,
    MAC_NO_BEACON               = 0xEA,
    MAC_NO_DATA                 = 0xEB,
    MAC_NO_SHORT_ADDRESS        = 0xEC,
    MAC_OUT_OF_CAP              = 0xED,
    MAC_PAN_ID_CONFLICT         = 0xEE,
    MAC_REALIGNMENT             = 0xEF,
    MAC_TRANSACTION_EXPIRED     = 0xF0,
    MAC_TRANSACTION_OVERFLOW    = 0xF1,
    MAC_TX_ACTIVE               = 0xF2,
    MAC_UNSUPPORTED_ATTRIBUTE   = 0xF4,
    MAC_INVALID_ADDRESS         = 0xF5
} retval_t;

/*
 * Macros defined for the radio transceiver's access modes.
 *
 * These functions are implemented as macros since they are used very often and
 * we want to remove the function call overhead.
 */
#define SPI_DUMMY_VALUE        ( 0x00 ) //!< Dummy value for the SPI.

/*! \brief This macro pulls the SLP_TR pin high.
 *
 *  \ingroup hal_avr_api
 */
#define PAL_SLP_TR_HIGH( )              gpio_set_gpio_pin(SLP_TR);

/*! \brief This macro pulls the SLP_TR pin low.
 *
 *  \ingroup hal_avr_api
 */
#define PAL_SLP_TR_LOW( )               gpio_clr_gpio_pin(SLP_TR);

/*! \brief  Read current state of the SLP_TR pin (High/Low).
 *
 *  \retval 0 if the pin is low, 1 is the pin is high.
 *
 *  \ingroup hal_avr_api
 */
#define hal_get_slptr( )                gpio_get_pin_value(SLP_TR)

/*! \brief This macro pulls the RST pin high.
 *
 *  \ingroup hal_avr_api
 */
#define PAL_RST_HIGH( )                 gpio_set_gpio_pin(RST);

/*! \brief This macro pulls the RST pin low.
 *
 *  \ingroup hal_avr_api
 */
#define PAL_RST_LOW( )                  gpio_clr_gpio_pin(RST);

/*! \brief  Read current state of the RST pin (High/Low).
 *
 *  \retval 0 if the pin is low, 1 if the pin is high.
 *
 *  \ingroup hal_avr_api
 */
#define hal_get_rst( )                  gpio_get_pin_value(RST);

/*Chip Select Makros*/
#define HAL_SS_HIGH( ) spi_unselectChip(RF231_SPI, RF231_SPI_NPCS); //!< MACRO for pulling SS high.
#define HAL_SS_LOW( )  spi_selectChip(RF231_SPI, RF231_SPI_NPCS); //!< MACRO for pulling SS low.

/*========================= PROTOTYPES =======================================*/
void init_at86rf231( void );
void delay_us(unsigned int usec);
void eic_rf230_isr(void);
void trx_irq_handler_cb(void);
void pal_trx_irq_init(uint8_t dummy, void *trx_irq_cb);
void trx_interface_init(void);
void handle_tx_end_irq(bool underrun_occured);
void handle_received_frame_irq(void);
void delay_us(unsigned int usec);
uint8_t tal_rx_enable(uint8_t state);
void pal_trx_irq_init(uint8_t dummy, void *trx_irq_cb);
uint8_t pal_trx_reg_read(uint8_t addr);
void pal_trx_reg_write(uint8_t addr, uint8_t data);
void pal_trx_frame_read(uint8_t *data, uint8_t length);
void pal_trx_frame_write(uint8_t *data, uint8_t length);
uint8_t pal_trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos);
void pal_trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t new_value);
void write_all_tal_pib_to_trx(void);
void switch_pll_on(void);
long eic_rf230_isr_NonNakedBehaviour( void );
void handle_tx_end_irq(bool underrun_occured);
void send_frame(uint8_t *frame_tx, csma_mode_t csma_mode, bool tx_retries);
void tx_done_handling(void);
bool apply_channel_page_configuration(uint8_t ch_page);
void at86rf231_rx_with_auto_ack( void );
uint8_t at86rf231_change_channel( uint8_t new_channel );
void trx_config_csma( void );
#endif
/*EOF*/
