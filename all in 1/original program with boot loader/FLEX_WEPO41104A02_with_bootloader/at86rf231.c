/*
 * at86rf231.c
 *
 *  Created on: 30.10.2009
 *      Author: florian.merz
 */

//-----------------------------------------------------------------------------------------------//
// Header Files
//-----------------------------------------------------------------------------------------------//

#include "inc/at86.h"  // Transceiver definitions
#include "inc/eeprom_abbild.h"
#include <avr/wdt.h>            // Watchdog
#include "inc/Bootloader.h"

//-----------------------------------------------------------------------------------------------//
// Global data
//-----------------------------------------------------------------------------------------------//

/* Global Receive and Transmit buffer */
volatile at86rf231_frame at86rf231_tx_frame;
volatile at86rf231_frame at86rf231_rx_frame;

tal_state_t tal_state;
volatile uint8_t tx_end_irq_flag = 1;

extern eeprom_data *ptr_eeprom_sram_data;

//-----------------------------------------------------------------------------------------------//
// Implementation
//-----------------------------------------------------------------------------------------------//

/**
 * @brief Init AT86RF231 Transceiver
 *
 */
void at86rf231_init( void )
{
    /* TRX status variale */
    tal_trx_status_t tal_trx_status = 0;

    /* Init SPI, Interrupt, IO */
    /* The following pins are output pins.  */
    TRX_PORT2_DDR |= _BV( SEL );
    TRX_PORT1_DDR |= _BV( SCK );
    TRX_PORT2_DDR |= _BV( RST );
    TRX_PORT1_DDR |= _BV( MOSI );
    TRX_PORT1_DDR |= _BV( SLP_TR );

    /* The following pins are input pins.  */
    TRX_PORT1_DDR &= ~_BV( MISO );
    trx_interface_init();

    /* Install a handler for the transceiver interrupt. */
    pal_trx_irq_init( TRX_MAIN_IRQ_HDLR_IDX, ( void * )trx_irq_handler_cb );
    pal_trx_irq_enable( TRX_MAIN_IRQ_HDLR_IDX );

    /* Set RST + SLP PIN */
    PAL_RST_HIGH();
    PAL_SLP_TR_LOW();
    _delay_us( P_ON_TO_CLKM_AVAILABLE);

    /* Apply reset pulse */
    PAL_RST_LOW();
    _delay_us( RST_PULSE_WIDTH_US );
    PAL_RST_HIGH();

    /* Verify chip version */
    while( ( pal_trx_reg_read( RG_VERSION_NUM ) != AT86RF231_VERSION_NUM ) ||
           ( pal_trx_reg_read( RG_PART_NUM ) != AT86RF231_PART_NUM ) );

    /* Set Transceiver State = TRX_OFF */
    pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

    /* verify that state = TRX_OFF */
    while( tal_trx_status != TRX_OFF )
    {
        tal_trx_status = ( tal_trx_status_t )pal_trx_bit_read( SR_TRX_STATUS );
    }

    /* write status variable */
    tal_trx_status = TRX_OFF;

    /* IO Driver strength */
    pal_trx_bit_write( SR_PAD_IO_CLKM, PAD_CLKM_2_MA );

    /* Set CLKM_SHA_DISABLE */
    pal_trx_bit_write( SR_CLKM_SHA_SEL, CLKM_SHA_DISABLE );

    /* Config clock */
    pal_trx_bit_write( SR_CLKM_CTRL, CLKM_1MHZ );

    /* ACKs for data requests, indicate pending data */
    pal_trx_bit_write( SR_AACK_SET_PD, PD_ACK_BIT_SET_ENABLE );

    /* Enable buffer protection mode */
    pal_trx_bit_write( SR_RX_SAFE_MODE, RX_SAFE_MODE_ENABLE );

    /* Enable poll mode */
    pal_trx_bit_write( SR_IRQ_MASK_MODE, IRQ_MASK_MODE_ON );

    /* The TRX_END interrupt of the transceiver is enabled. */
    pal_trx_reg_write( RG_IRQ_MASK, TRX_IRQ_TRX_END );

    /* Enable auto time stamping */
    pal_trx_bit_write( SR_IRQ_2_EXT_EN, TIMESTAMPING_ENABLE );

    /* write device addresses */
    pal_trx_reg_write( RG_PAN_ID_0,    ptr_eeprom_sram_data->pan_id); //( uint8_t )( TAL_PANID_BC_DEFAULT ) );
    pal_trx_reg_write( RG_PAN_ID_1,    ptr_eeprom_sram_data->pan_id>>8); //( uint8_t )( TAL_PANID_BC_DEFAULT >> 8 ) );

    pal_trx_reg_write( RG_SHORT_ADDR_0, ptr_eeprom_sram_data->src_address);//( uint8_t )( TAL_SHORT_ADDRESS_DEFAULT ) );
    pal_trx_reg_write( RG_SHORT_ADDR_1, ptr_eeprom_sram_data->src_address>>8);//( uint8_t )( TAL_SHORT_ADDRESS_DEFAULT >> 8 ) );

    /* configure TX_ARET; CSMA and CCA */
    pal_trx_bit_write( SR_CCA_MODE, TAL_CCA_MODE_DEFAULT );
    pal_trx_bit_write( SR_MIN_BE, TAL_MINBE_DEFAULT );
    pal_trx_bit_write( SR_AACK_I_AM_COORD, TAL_PAN_COORDINATOR_DEFAULT );

    /* set phy parameter */
    pal_trx_bit_write( SR_MAX_BE, TAL_MAXBE_DEFAULT );

    /* write channel page configuration 0, 2, 16, 17, see function for more details */
    at86rf231_change_channel_page(ptr_eeprom_sram_data->channel_page );

    /* write channel */
    pal_trx_bit_write( SR_CHANNEL, ptr_eeprom_sram_data->channel );	// B (11) - 1A (26)  

    /* set power level */
    pal_trx_bit_write( SR_TX_PWR, ptr_eeprom_sram_data->tx_pwr );
    // 0x0   3.0 dBm
    // 0x1   2.8 dBm
    // 0x2   2.3 dBm
    // 0x3   1.8 dBm
    // 0x4   1.3 dBm
    // 0x5   0.7 dBm
    // 0x6   0.0 dBm
    // 0x7   -1 dBm
    // 0x8   -2 dBm
    // 0x9   -3 dBm
    // 0xA   -4 dBm
    // 0xB   -5 dBm
    // 0xC   -7 dBm
    // 0xD   -9 dBm
    // 0xE   -12 dBm
    // 0xF   -17 dBm

    /* config csma */
    //trx_config_csma();

    /* write status variable */
    tal_state = TAL_IDLE;

//    volatile uint8_t temp = 0x00;
//
//    temp  = pal_trx_reg_read( RG_XOSCii_CTRL );
//    pal_trx_reg_write( RG_XOSC_CTRL, 0xFF );
//    temp  = pal_trx_reg_read( RG_XOSC_CTRL );


}

/**
 * @brief Switch transceiver pll on and wait until it is locked
 *
 */
uint8_t at86rf231_switch_pll_on(void)
{
    trx_irq_reason_t irq_status;

    /* Check if trx is in TRX_OFF; only from PLL_ON the following procedure is applicable */
    if (pal_trx_bit_read(SR_TRX_STATUS) != TRX_OFF)
    {
        return 0;
    }

    pal_trx_reg_read(RG_IRQ_STATUS);    /* clear PLL lock bit */
    /* Switch PLL on */
    pal_trx_reg_write(RG_TRX_STATE, CMD_PLL_ON);

    /* Check if PLL has been locked. */
    //pal_get_current_time(&start_time);
    while (1)
    {
        irq_status = (trx_irq_reason_t)pal_trx_reg_read(RG_IRQ_STATUS);
        if (irq_status & TRX_IRQ_PLL_LOCK)
        {
            break;  // PLL is locked now
        }

    }

    return 1;
}

/**
 * @brief Set Transceiver mode = sleep
 */
void at86rf231_sleep( void )
{
    pal_trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF);
    _delay_us(1);
    PAL_SLP_TR_HIGH();
}

/**
 * @brief Set Transceiver State = RX_AACK_ON
 * send ACK-Frame automatically if requested
 *
 */
void at86rf231_rx_with_auto_ack( void )
{
    /* Set Transceiver State = TRX_OFF */
    pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

    /* Set Transceiver State = PLL_ON and until PLL locks */
    at86rf231_switch_pll_on(); //only allowed from state = TRX_OFF

    /* tal_state = receive mode */
    tal_state = RX_ON;

    /* Set Transceiver State = RX_AACK_ON  */
    pal_trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);
}

/**
 * @brief Send data without ACK and Retry
 */
void at86rf231_tx( void )
{
    /* Set state PLL on */
    pal_trx_reg_write(RG_TRX_STATE, CMD_PLL_ON);

    /* pal_trx_irq_disable(TRX_MAIN_IRQ_HDLR_IDX) */
    tal_state = TAL_TX_AUTO;

    /* Reset TX_END Interrupt Flag */
    tx_end_irq_flag = 0;

    /* Toggle the SLP_TR pin triggering transmission. */
    PAL_SLP_TR_HIGH();
    asm volatile("nop\n\t" ::); // wait 65ns
    asm volatile("nop\n\t" ::);
    asm volatile("nop\n\t" ::);
    asm volatile("nop\n\t" ::);
    asm volatile("nop\n\t" ::);
    PAL_SLP_TR_LOW();

    /* Send the frame to the transceiver. */
    pal_trx_frame_write( (uint8_t *)&at86rf231_tx_frame, at86rf231_tx_frame.frame_length);
}

/**
 * @brief Sends frame
 *
 * @param use_csma Flag indicating if CSMA is requested
 * @param tx_retries Flag indicating if transmission retries are requested
 *                   by the MAC layer
 */
void at86rf231_tx_with_retry( csma_mode_t csma_mode, bool tx_retries)
{
    /* TRX status variale */
    tal_trx_status_t tal_trx_status = 0;

    /* configure tx according to tx_retries */
    if (tx_retries)
    {
        pal_trx_bit_write(SR_MAX_FRAME_RETRIES, TAL_MAXFRAMERETRIES_DEFAULT);
        pal_trx_bit_write(SR_MAX_FRAME_RETRIES, 8);
    }
    else
    {
        pal_trx_bit_write(SR_MAX_FRAME_RETRIES, 0);
    }

    /* configure tx according to csma usage */
    if ((csma_mode == NO_CSMA_NO_IFS) || (csma_mode == NO_CSMA_WITH_IFS))
    {
        if (tx_retries)
        {
            pal_trx_bit_write(SR_MAX_CSMA_RETRIES, TAL_MAX_CSMA_BACKOFFS_DEFAULT);
            pal_trx_reg_write(RG_CSMA_BE, 0x00);
        }
        else
        {
            pal_trx_bit_write(SR_MAX_CSMA_RETRIES, 7);
        }
    }
    else
    {
        pal_trx_reg_write(RG_CSMA_BE, ((TAL_MAXBE_DEFAULT << 4) | TAL_MINBE_DEFAULT));
        pal_trx_bit_write(SR_MAX_CSMA_RETRIES, TAL_MAX_CSMA_BACKOFFS_DEFAULT);
    }

    /* state change from TRX_OFF to TX_ARET_ON can be done directly, too */
    at86rf231_switch_pll_on();

    do
    {
        pal_trx_reg_write(RG_TRX_STATE, CMD_TX_ARET_ON);
        _delay_us(1);
        tal_trx_status = ( tal_trx_status_t )pal_trx_bit_read( SR_TRX_STATUS );

    } while (tal_trx_status != TX_ARET_ON);

    tx_end_irq_flag = 0;
    //pal_trx_irq_disable(TRX_MAIN_IRQ_HDLR_IDX);
    tal_state = TAL_TX_AUTO;

    /* Toggle the SLP_TR pin triggering transmission. */
    PAL_SLP_TR_HIGH();
    asm volatile("nop\n\t" ::); // wait 65ns
    asm volatile("nop\n\t" ::);
    asm volatile("nop\n\t" ::);
    asm volatile("nop\n\t" ::);
    asm volatile("nop\n\t" ::);
    PAL_SLP_TR_LOW();

    /* Send the frame to the transceiver. */
    pal_trx_frame_write( (uint8_t *)&at86rf231_tx_frame, at86rf231_tx_frame.frame_length);

    while(tx_end_irq_flag == 0)
    {

    }
}

/**
 * @brief Send data request and receive data with ACK and retry
 */
void at86rf231_req_data( void )
{
     uint8_t i = 0;

     /* Build frame to transmit */
     at86rf231_tx_frame.frame_length = 67;
     at86rf231_tx_frame.frame_control_field = FCF_SET_FRAMETYPE(FCF_FRAMETYPE_MAC_CMD) |
                                              FCF_SET_DEST_ADDR_MODE(FCF_SHORT_ADDR) |
                                              FCF_SET_SOURCE_ADDR_MODE(FCF_SHORT_ADDR)|
                                              FCF_ACK_REQUEST;

     at86rf231_tx_frame.seq_number = 0xFF;
     at86rf231_tx_frame.dest_pan_id = ptr_eeprom_sram_data->pan_id;
     at86rf231_tx_frame.dest_addr = ptr_eeprom_sram_data->dest_address;
     at86rf231_tx_frame.src_pan_id = ptr_eeprom_sram_data->pan_id;
     at86rf231_tx_frame.src_addr = ptr_eeprom_sram_data->src_address;

     /* Ensure that global Interrupts are enabled */
     sei();

     /* Send frame with ACK */
     at86rf231_tx_with_retry( NO_CSMA_NO_IFS, 1 );

     /* Reset CMD */
     for(i=0;i < (aMaxPHYPacketSize - 12) ;i++)
         at86rf231_rx_frame.payload[i] = 0x00;

     /* Now wait for a config data packet */
     at86rf231_rx_with_auto_ack();
}

/**
 * @brief Apply channel to transceiver
 *
 * @param uint8_t new_channel
 *
 * @return content of channel register
 */
uint8_t at86rf231_change_channel( uint8_t new_channel )
{
    /* TRX status variale */
    tal_trx_status_t tal_trx_status = 0;

    if ((uint32_t)TRX_SUPPORTED_CHANNELS & ((uint32_t)0x01 << new_channel ))
    {
        /* Set Transceiver State = TRX_OFF */
        pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

        /* verify that state = TRX_OFF */
        while( tal_trx_status != TRX_OFF )
        {
            tal_trx_status = ( tal_trx_status_t )pal_trx_bit_read( SR_TRX_STATUS );
        }

        /* write new channel */
        pal_trx_bit_write(SR_CHANNEL, new_channel);
    }
    return pal_trx_bit_read(SR_CHANNEL);
}

/**
 * @brief Apply channel page configuartion to transceiver
 *
 * @param ch_page Channel page
 *
 * @return true if changes could be applied else false
 */
bool at86rf231_change_channel_page(uint8_t ch_page)
{
    /* TRX status variale */
    tal_trx_status_t tal_trx_status = 0;

    /* Set Transceiver State = TRX_OFF */
    pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

    /* verify that state = TRX_OFF */
    while( tal_trx_status != TRX_OFF )
    {
        tal_trx_status = ( tal_trx_status_t )pal_trx_bit_read( SR_TRX_STATUS );
    }

    switch (ch_page)
    {
        case 0: /* compliant O-QPSK */
            pal_trx_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_250KBPS);
            // Apply compliant ACK timing
            pal_trx_bit_write(SR_AACK_ACK_TIME, AACK_ACK_TIME_12_SYMBOLS);
            // Use full sensitivity
            pal_trx_bit_write(SR_RX_PDT_LEVEL, 0x00);
            break;

        case 2: /* non-compliant OQPSK mode 1 */
            pal_trx_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_500KBPS);
            // Apply reduced ACK timing
            pal_trx_bit_write(SR_AACK_ACK_TIME, AACK_ACK_TIME_2_SYMBOLS);
            // Use full sensitivity
            pal_trx_bit_write(SR_RX_PDT_LEVEL, 0x00);
            break;

        case 16:    /* non-compliant OQPSK mode 2 */
            pal_trx_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_1MBPS);
            // Apply reduced ACK timing
            pal_trx_bit_write(SR_AACK_ACK_TIME, AACK_ACK_TIME_2_SYMBOLS);
            // Use full sensitivity
            pal_trx_bit_write(SR_RX_PDT_LEVEL, 0x00);
            break;

        case 17:    /* non-compliant OQPSK mode 3 */
            pal_trx_bit_write(SR_OQPSK_DATA_RATE, ALTRATE_2MBPS);
            // Apply reduced ACK timing
            pal_trx_bit_write(SR_AACK_ACK_TIME, AACK_ACK_TIME_2_SYMBOLS);
            // Use reduced sensitivity for 2Mbit mode
            pal_trx_bit_write(SR_RX_PDT_LEVEL, 0x01);
            break;

        default:
            return false;
    }

    return true;
}

/**
 * @brief Apply tx power to transceiver
 *
 * @param uint8_t pwr_value
 *
 * @return content of pwr register or 0xFF=error
 */
uint8_t at86rf231_change_tx_pwr( uint8_t pwr )
{
    if( pwr <= 0x0F)
    {
        /* TRX status variale */
        tal_trx_status_t tal_trx_status = 0;

        /* Set Transceiver State = TRX_OFF */
        pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

        /* verify that state = TRX_OFF */
        while( tal_trx_status != TRX_OFF )
        {
            tal_trx_status = ( tal_trx_status_t )pal_trx_bit_read( SR_TRX_STATUS );
        }

        /* set power level */
        pal_trx_bit_write( SR_TX_PWR, pwr );
        // 0x0   3.0 dBm
        // 0x1   2.8 dBm
        // 0x2   2.3 dBm
        // 0x3   1.8 dBm
        // 0x4   1.3 dBm
        // 0x5   0.7 dBm
        // 0x6   0.0 dBm
        // 0x7   -1 dBm
        // 0x8   -2 dBm
        // 0x9   -3 dBm
        // 0xA   -4 dBm
        // 0xB   -5 dBm
        // 0xC   -7 dBm
        // 0xD   -9 dBm
        // 0xE   -12 dBm
        // 0xF   -17 dBm

        return pal_trx_bit_read(SR_TX_PWR);
    }
    else
    {
        return 0xFF;
    }
}

/**
 * @brief Filter tuning and PLL calibration handling
 *  Although receiver and transmitter are very robust against these variations, it is recommended to
 *  initiate the FTN manually if the radio transceiver does not use the SLEEP state. If necessary, a
 *  calibration cycle is to be initiated in states TRX_OFF, PLL_ON or any receive state. This applies
 *  in particular for the High Data Rate Modes with a much higher sensitivity against BPF transfer
 *  function variations. The recommended calibration interval is 5 min or less.
 */
void at86rf231_ftn_pll_calibration(void)
{
    /* Same for both filter tuning and PLL calibration. */
    pal_trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF);

    pal_trx_bit_write(SR_FTN_START, 1);

    /* Wait tTR16 (FTN calibration time). */
    _delay_us(25);
}

//-----------------------------------------------------------------------------------------------//
// Interrupts
//-----------------------------------------------------------------------------------------------//

/**
 * @brief Transceiver interrupt handler
 *
 * This function handles the transceiver generated interrupts.
 */
void trx_irq_handler_cb(void)
{
    trx_irq_reason_t trx_irq_cause;
    trx_trac_status_t trx_trac_status;

    /* what was the reason for the interrupt */
    trx_irq_cause = (trx_irq_reason_t)pal_trx_reg_read(RG_IRQ_STATUS);

    if (trx_irq_cause & TRX_IRQ_TRX_END)
    {
        /*
         * TRX_END reason depends on if the trx is currently used for
         * transmission or reception.
         */
        if (tal_state == TAL_TX_AUTO)
        {
            /* Get the result and push it to the queue. */
            if (trx_irq_cause & TRX_IRQ_TRX_UR)
            {
                trx_trac_status = TRAC_INVALID;
            }
            else
            {
                /* Check if no ACK was received */
                trx_trac_status = (trx_trac_status_t)pal_trx_bit_read(SR_TRAC_STATUS);
                pal_trx_reg_write(RG_TRX_STATE, RX_ON);
            }

            tx_end_irq_flag = 1;
            tal_state = TAL_TX_DONE;    // Further handling is done by tx_done_handling()
        }
        else   /* Other tal_state than TAL_TX_... */
        {
            /* address match interrupt */
            if (trx_irq_cause & TRX_IRQ_AMI)
            {
                /* Handle rx interrupt. */
                handle_received_frame_irq();    // see tal_rx.c
            }
        }
    }
//#define DEBUG 1
#if (DEBUG > 0)
    /* Other IRQ than TRX_END */
    if (trx_irq_cause != TRX_IRQ_TRX_END)
    {
        /* PLL_LOCK interrupt migth be set, because poll mode is enabled. */
        /*
        if (trx_irq_cause & TRX_IRQ_PLL_LOCK)
        {
            ASSERT("unexpected IRQ: TRX_IRQ_PLL_LOCK" == 0);
        }
        */
        if (trx_irq_cause & TRX_IRQ_PLL_UNLOCK)
        {
            //ASSERT("unexpected IRQ: TRX_IRQ_PLL_UNLOCK" == 0);
        }
        /* RX_START interrupt migth be set, because poll mode is enabled. */

        if (trx_irq_cause & TRX_IRQ_RX_START)
        {
            //ASSERT("unexpected IRQ: TRX_IRQ_RX_START" == 0);
        }

        if (trx_irq_cause & TRX_IRQ_CCA_ED_READY)
        {
            //ASSERT("unexpected IRQ: TRX_IRQ_CCA_ED_READY" == 0);
        }

        /* AMI interrupt might set, because poll mode is enabled. */
        if (trx_irq_cause & TRX_IRQ_AMI)
        {
            //ASSERT("unexpected IRQ: TRX_IRQ_AMI" == 0);
        }

        if (trx_irq_cause & TRX_IRQ_TRX_UR)
        {
            //ASSERT("unexpected IRQ: TRX_IRQ_TRX_UR" == 0);
        }
        if (trx_irq_cause & TRX_IRQ_BAT_LOW)
        {
            //ASSERT("unexpected IRQ: TRX_IRQ_BAT_LOW" == 0);
        }
    }
#endif

}/* trx_irq_handler_cb() */

/**
 * @brief Handle received frame interrupt
 *
 * This function handles transceiver interrupts for received frames and
 * uploads the frames from the trx.
 */
void handle_received_frame_irq( void )
{
    static uint8_t ed_value;
    uint8_t frame_length;

    /* Get ED value; needed to normalize LQI. */
    ed_value = pal_trx_reg_read( RG_PHY_ED_LEVEL );

    /* Get frame length from transceiver. */
    pal_trx_frame_read( &frame_length, LENGTH_FIELD_LEN );

    /* Check for valid frame length. */
    if( frame_length > 127 )
    {
        return;
    }
    /* check if crc is valid */
    if( pal_trx_bit_read(SR_RX_CRC_VALID) )
    {
        /*
         * The PHY header is also included in the frame, hence the frame length
         * is incremented by 1. In addition to that, the LQI needs to be uploaded, too.
         */
        frame_length += LQI_LEN + LENGTH_FIELD_LEN;

        /* Upload frame and store it to the buffer */
        //pal_trx_frame_read( (uint8_t *)tal_rx_buffer, frame_length );
        pal_trx_frame_read( (uint8_t*)&at86rf231_rx_frame, frame_length );
		
		if(((at86rf231_rx_frame.frame_control_field & 0x07) == 3) && (at86rf231_rx_frame.payload[0] == 0x40))
			boot();
	}
    else
    {
        // CRC ERROR
    }
    /* Check if receive buffer is available */
    if( NULL == &at86rf231_rx_frame )
    {
        /* Use a fast state change instead of set_trx_state(). */
        pal_trx_reg_write( RG_TRX_STATE, CMD_PLL_ON );
    }
    else
    {
        /*
         * Trx returns to RX_AACK_ON automatically, if this was its previous state.
         * Keep the following as a reminder, if receiver is used with RX_ON instead.
         */
        //pal_trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);
    }
}
