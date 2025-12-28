/*
 * at86rf231.c
 *
 *  Created on: 30.10.2009
 *      Author: florian.merz
 */

#include "main_app.h"
#include <stdint.h>
#include "at86.h"
#include "usb.h"

/* Receive and Transmit buffer */
uint8_t tal_rx_buffer[132];
uint8_t tal_tx_buffer[132];

tal_trx_status_t tal_trx_status;
xSemaphoreHandle xSem_radio_receive;
rf231_rec_struct rf231_rec_info ;

volatile frame_info_union rx_frame;
volatile frame_info_union tx_frame;
extern app_control_struct app_control;

/**
 * @brief Init AT86RF231 Transceiver
 *
 */
void init_at86rf231( void )
{
    /* Create Mutex for Radio TRX Event */
    vSemaphoreCreateBinary( xSem_radio_receive );

    /* Init SPI, Interrupt, IO */
    trx_interface_init();

    /* Set RST + SLP PIN */
    PAL_RST_HIGH();
    PAL_SLP_TR_LOW();
    delay_us( P_ON_TO_CLKM_AVAILABLE );

    /* Apply reset pulse */
    PAL_RST_LOW();
    delay_us( RST_PULSE_WIDTH_US );
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
    pal_trx_reg_write( RG_IRQ_MASK, TRX_IRQ_DEFAULT );

    /* Antenne diversity */
#ifdef ANTENNA_DIVERSITY
    pal_trx_bit_write( SR_ANT_CTRL, ANTENNA_DEFAULT );
    pal_trx_bit_write( SR_PDT_THRES, THRES_ANT_DIV_ENABLE );
    pal_trx_bit_write( SR_ANT_DIV_EN, ANT_DIV_ENABLE );
    pal_trx_bit_write( SR_ANT_EXT_SW_EN, ANT_EXT_SW_SWITCH_ENABLE );
#else
    pal_trx_bit_write( SR_IRQ_2_EXT_EN, TIMESTAMPING_ENABLE );
#endif

    /* write device addresses */
    pal_trx_reg_write( RG_PAN_ID_0, ptr_flash_data->rf231_pan_id );
    pal_trx_reg_write( RG_PAN_ID_1, ptr_flash_data->rf231_pan_id >> 8 );

    pal_trx_reg_write( RG_SHORT_ADDR_0, ptr_flash_data->rf231_src_address );
    pal_trx_reg_write( RG_SHORT_ADDR_1, ptr_flash_data->rf231_src_address >> 8 );

    /* configure TX_ARET; CSMA and CCA */
    pal_trx_bit_write( SR_CCA_MODE, TAL_CCA_MODE_DEFAULT );
    pal_trx_bit_write( SR_MIN_BE, TAL_MINBE_DEFAULT );
    //pal_trx_bit_write( SR_AACK_I_AM_COORD, TAL_PAN_COORDINATOR_DEFAULT );
    pal_trx_bit_write( SR_AACK_I_AM_COORD, TRUE );
    /* set phy parameter */
    pal_trx_bit_write( SR_MAX_BE, TAL_MAXBE_DEFAULT );

    /* write channel page */
    apply_channel_page_configuration(ptr_flash_data->rf231_channel_page );

    /* write channel */
    pal_trx_bit_write( SR_CHANNEL, ptr_flash_data->rf231_channel );
    // channel 11 fest
    //pal_trx_bit_write( SR_CHANNEL, 0x0B);

    /* set power level */
    pal_trx_bit_write( SR_TX_PWR, ptr_flash_data->rf231_tx_pwr );
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
    trx_config_csma();

    /* write status variable */
    tal_state = TAL_IDLE;
}

/**
 * @brief Sends frame
 *
 * @param frame_tx Pointer to prepared frame
 * @param use_csma Flag indicating if CSMA is requested
 * @param tx_retries Flag indicating if transmission retries are requested
 *                   by the MAC layer
 */
void send_frame(uint8_t *frame_tx, csma_mode_t csma_mode, bool tx_retries)
{
    /* configure tx according to tx_retries */
    if (tx_retries)
    {
        //pal_trx_bit_write(SR_MAX_FRAME_RETRIES, TAL_MAXFRAMERETRIES_DEFAULT);
        pal_trx_bit_write(SR_MAX_FRAME_RETRIES, 5);
    }
    else
    {
        pal_trx_bit_write(SR_MAX_FRAME_RETRIES, 0);
    }

    // configure tx according to csma usage
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

    do
    {
        tal_trx_status = set_trx_state(CMD_TX_ARET_ON);
    } while (tal_trx_status != TX_ARET_ON);

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

    //pal_trx_frame_write(frame_tx+1, frame_tx[0]);   //frame_tx[0]-1
    pal_trx_frame_write( (uint8_t *)&tx_frame, tx_frame.at86rf231_frame.frame_length);
}

/**
 * @brief Handles interrupts issued due to end of transmission
 */
void handle_tx_end_irq(bool underrun_occured)
{
    /* No ACK received?, start streaming */
    if( pal_trx_bit_read( SR_TRAC_STATUS ) == TRAC_NO_ACK)
    {
      //  data_printf("\r\n NO ACK RECEIVED!!! \r\n");
    }
}

/**
 * @brief Handle received frame interrupt
 *
 * This function handles transceiver interrupts for received frames and
 * uploads the frames from the trx.
 */
void handle_received_frame_irq( void )
{
    static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    long xSwitchRequired = FALSE;

    /* Get ED value; needed to normalize LQI. */
    rf231_rec_info.rssi_dbm = pal_trx_reg_read( RG_PHY_ED_LEVEL );
    rf231_rec_info.rssi_dbm += -91; // [dBm]

    /* Get frame length from transceiver. */
    pal_trx_frame_read( &rf231_rec_info.packet_length, LENGTH_FIELD_LEN );

    rf231_rec_info.frame_ptr = (frame_info_union*)&rx_frame;

    /* Check for valid frame length. */
    if( rf231_rec_info.packet_length > 127 )
    {
        return;
    }

    /* check if crc is valid, note: auto crc is enabled */
    //if( pal_trx_bit_read(SR_RX_CRC_VALID) )
    //{
        /*
         * The PHY header is also included in the frame, hence the frame length
         * is incremented by 1. In addition to that, the LQI needs to be uploaded, too.
         */
        rf231_rec_info.packet_length += LQI_LEN + LENGTH_FIELD_LEN;

        /* Upload frame and store it to the buffer */
        pal_trx_frame_read( (uint8_t*)rf231_rec_info.frame_ptr , rf231_rec_info.packet_length );

		
        /*  */
        rx_frame.at86rf231_frame.frame_control_field = bswap_16(rx_frame.at86rf231_frame.frame_control_field);
				     
		/* Give Semaphor from ISR */
        xSwitchRequired = xSemaphoreGiveFromISR( xSem_radio_receive, &xHigherPriorityTaskWoken );
    //}
    //else
    //{
        /* CRC-ERROR */
    //   app_control.error_crc_cnt++;
    //}
}

/**
 * @brief Sets transceiver state
 *
 * @param trx_cmd needs to be one of the trx commands
 *
 * @return current trx state
 */
tal_trx_status_t set_trx_state(trx_cmd_t trx_cmd)
{
    if (tal_trx_status == TRX_SLEEP)
    {
        uint8_t bit_status;

        PAL_SLP_TR_LOW();
        /* poll status register until TRX_OFF is reached */
        do
        {
            bit_status = pal_trx_bit_read(SR_TRX_STATUS);
        } while (bit_status != TRX_OFF);

#if (DEBUG > 0)
        pal_trx_reg_read(RG_IRQ_STATUS);    /* clear Wake irq, dummy read */
#endif

#ifdef ANTENNA_DIVERSITY
        /* Enable antenna diversity. */
        pal_trx_bit_write(SR_ANT_EXT_SW_EN, ANT_EXT_SW_SWITCH_ENABLE);
#endif

        if ((trx_cmd == CMD_TRX_OFF) || (trx_cmd == CMD_FORCE_TRX_OFF))
        {
            tal_trx_status = TRX_OFF;
            return TRX_OFF;
        }
    }

    tal_trx_status = (tal_trx_status_t)pal_trx_bit_read(SR_TRX_STATUS);

    switch (trx_cmd)    /* requested state */
    {
        case CMD_SLEEP:
            pal_trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF);
#ifdef ANTENNA_DIVERSITY
            /* Disable antenna diversity: sets pulls */
            pal_trx_bit_write(SR_ANT_EXT_SW_EN, ANT_EXT_SW_SWITCH_DISABLE);
#endif
            {
                uint16_t rand_value;

                /*
                 * Init the SEED value of the CSMA backoff algorithm.
                 */
                rand_value = (uint16_t)rand();
                pal_trx_reg_write(RG_CSMA_SEED_0, (uint8_t)rand_value);
                pal_trx_bit_write(SR_CSMA_SEED_1, (uint8_t)(rand_value >> 8));
            }

            delay_us(1);
            PAL_SLP_TR_HIGH();
            delay_us(TRX_OFF_TO_SLEEP_TIME);
            tal_trx_status = TRX_SLEEP;
            return TRX_SLEEP;   /* transceiver register cannot be read during TRX_SLEEP */

        case CMD_TRX_OFF:
            switch (tal_trx_status)
            {
                case TRX_OFF:
                    break;

                default:
                    pal_trx_reg_write(RG_TRX_STATE, CMD_TRX_OFF);
                    delay_us(1);
                    break;
            }
            break;

        case CMD_FORCE_TRX_OFF:
            switch (tal_trx_status)
            {
                case TRX_OFF:
                    break;

                default:
                    pal_trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF);
                    delay_us(1);
                    break;
            }
            break;

        case CMD_PLL_ON:
            switch (tal_trx_status)
            {
                case PLL_ON:
                    break;

                case TRX_OFF:
                    switch_pll_on();
                    break;

                case RX_ON:
                case RX_AACK_ON:
                case TX_ARET_ON:
                    pal_trx_reg_write(RG_TRX_STATE, CMD_PLL_ON);
                    delay_us(1);
                    break;

                case BUSY_RX:
                case BUSY_TX:
                case BUSY_RX_AACK:
                case BUSY_TX_ARET:
                    /* do nothing if trx is busy */
                    break;

                default:
                    //ASSERT("state transition not handled" == 0);
                    break;
            }
            break;

        case CMD_FORCE_PLL_ON:
            switch (tal_trx_status)
            {
                case TRX_OFF:
                    switch_pll_on();
                    break;

                case PLL_ON:
                    break;

                default:
                    pal_trx_reg_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
                    break;
            }
            break;

        case CMD_RX_ON:
            switch (tal_trx_status)
            {
                case RX_ON:
                    break;

                case PLL_ON:
                case RX_AACK_ON:
                case TX_ARET_ON:
                    pal_trx_reg_write(RG_TRX_STATE, CMD_RX_ON);
                    delay_us(1);
                    break;

                case TRX_OFF:
                    switch_pll_on();
                    pal_trx_reg_write(RG_TRX_STATE, CMD_RX_ON);
                    delay_us(1);
                    break;

                case BUSY_RX:
                case BUSY_TX:
                case BUSY_RX_AACK:
                case BUSY_TX_ARET:
                    /* do nothing if trx is busy */
                    break;

                default:
                    //ASSERT("state transition not handled" == 0);
                    break;
            }
            break;

        case CMD_RX_AACK_ON:
            switch (tal_trx_status)
            {
                case RX_AACK_ON:
                    break;

                case TX_ARET_ON:
                case PLL_ON:
                    pal_trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);
                    delay_us(1);
                    break;

                case TRX_OFF:
                    switch_pll_on();// state change from TRX_OFF to RX_AACK_ON can be done directly, too
                    pal_trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);
                    delay_us(1);
                    break;

                case RX_ON:
                    pal_trx_reg_write(RG_TRX_STATE, CMD_PLL_ON);
                    delay_us(1);
                    // check if state change could be applied
                    tal_trx_status = (tal_trx_status_t)pal_trx_bit_read(SR_TRX_STATUS);
                    if (tal_trx_status != PLL_ON)
                    {
                        return tal_trx_status;
                    }
                    pal_trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);
                    delay_us(1);
                    break;

                case BUSY_RX:
                case BUSY_TX:
                case BUSY_RX_AACK:
                case BUSY_TX_ARET:
                    /* do nothing if trx is busy */
                    break;

                default:
                    //ASSERT("state transition not handled" == 0);
                    break;
            }
            break;

        case CMD_TX_ARET_ON:
            switch (tal_trx_status)
            {
                case TX_ARET_ON:
                    break;

                case PLL_ON:
                    pal_trx_reg_write(RG_TRX_STATE, CMD_TX_ARET_ON);
                    delay_us(1);
                    break;

                case RX_ON:
                case RX_AACK_ON:
                    pal_trx_reg_write(RG_TRX_STATE, CMD_PLL_ON);
                    delay_us(1);
                    // check if state change could be applied
                    tal_trx_status = (tal_trx_status_t)pal_trx_bit_read(SR_TRX_STATUS);
                    if (tal_trx_status != PLL_ON)
                    {
                        return tal_trx_status;
                    }
                    pal_trx_reg_write(RG_TRX_STATE, CMD_TX_ARET_ON);
                    delay_us(1);
                    break;

                case TRX_OFF:
                    switch_pll_on();// state change from TRX_OFF to TX_ARET_ON can be done directly, too
                    pal_trx_reg_write(RG_TRX_STATE, CMD_TX_ARET_ON);
                    delay_us(1);
                    break;

                case BUSY_RX:
                case BUSY_TX:
                case BUSY_RX_AACK:
                case BUSY_TX_ARET:
                    /* do nothing if trx is busy */
                    break;

                default:
                    //ASSERT("state transition not handled" == 0);
                    break;
            }
            break;

        default:
            /* CMD_NOP, CMD_TX_START */
            //ASSERT("trx command not handled" == 0);
            break;
    }

    do
    {
        tal_trx_status = (tal_trx_status_t)pal_trx_bit_read(SR_TRX_STATUS);
    } while (tal_trx_status == STATE_TRANSITION_IN_PROGRESS);

    return tal_trx_status;
} /* set_trx_state() */

/**
 * @brief Switch transceiver pll on and wait until it is locked
 *
 */
void switch_pll_on(void)
{
    trx_irq_reason_t irq_status;

    /* Check if trx is in TRX_OFF; only from PLL_ON the following procedure is applicable */
    if (pal_trx_bit_read(SR_TRX_STATUS) != TRX_OFF)
    {
        //ASSERT("Switch PLL_ON failed, because trx is not in TRX_OFF" == 0);
        return;
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
}

/**
 * @brief Apply channel page configuartion to transceiver
 *
 * @param ch_page Channel page
 *
 * @return true if changes could be applied else false
 */
bool apply_channel_page_configuration(uint8_t ch_page)
{
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
 * @brief Set Transceiver State = RX_AACK_ON
 * send ACK-Frame automatically if requested
 *
 */
void at86rf231_rx_with_auto_ack( void )
{
    /* Set Transceiver State = TRX_OFF */
    pal_trx_reg_write( RG_TRX_STATE, CMD_TRX_OFF );

    /* Set Transceiver State = PLL_ON and until PLL locks */
    switch_pll_on(); //only allowed from state = TRX_OFF

    /* tal_state = receive mode */
    tal_state = RX_ON;

    /* Set Transceiver State = RX_AACK_ON */
    pal_trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);
}

/**
 * @brief Configures the transceiver's CSMA seed
 *
 * This function is called to configure the transceiver's CSMA seed after reset.
 * it needs to be called in conjunction with funciton trx_config(), but
 * it needs be assured that a seed for function rand() had been generated before.
 */
void trx_config_csma(void)
{
    uint16_t rand_value;

    /*
     * Init the SEED value of the CSMA backoff algorithm.
     */
    rand_value = (uint16_t)rand();
    pal_trx_reg_write(RG_CSMA_SEED_0, (uint8_t)0x48);//rand_value);
    pal_trx_bit_write(SR_CSMA_SEED_1, (uint8_t)0x92);//(rand_value >> 8));

    /*
     * To make sure that the CSMA seed is properly set within the transceiver,
     * put the trx to sleep briefly and wake it up again.
     */
    //tal_trx_sleep(SLEEP_MODE_1);

    //tal_trx_wakeup();
}



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
