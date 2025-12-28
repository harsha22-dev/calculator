/**
 * @file pal_trx_access.c
 *
 * @brief Transceiver registers & Buffer accessing functions for
 *        AT32UC3A 32bit SPI based MCUs.
 *
 * This file implements functions for reading and writing transceiver
 * registers and transceiver buffer for AT91SAM 32bit SPI based MCUs.
 *
 * === Includes ============================================================= */

#include <stdint.h>
#include "board.h"
#include "at86.h"
#include "spi.h"
#include "gpio.h"
#include "eic.h"
#include "intc.h"
#include "FreeRTOS.h"
#include "task.h"

/* === Implementation ======================================================= */
static irq_handler_t irq_handler;
tal_state_t tal_state;

/**
 * @brief Initializes the transceiver interface
 *
 * This function initializes the transceiver interface.
 */
void trx_interface_init(void)
{
    /*IO Specific Initialization.*/
    gpio_enable_gpio_pin(SLP_TR);
    gpio_enable_gpio_pin(RST);

    /* reset spi0 */
    spi_reset(RF231_SPI);

    /* set spi hardware pins */
    static const gpio_map_t RF230_SPI_GPIO_MAP =
    {
      {RF231_SPI_SCK_PIN,  RF231_SPI_SCK_FUNCTION },  // SPI Clock.
      {RF231_SPI_MISO_PIN, RF231_SPI_MISO_FUNCTION},  // MISO.
      {RF231_SPI_MOSI_PIN, RF231_SPI_MOSI_FUNCTION},  // MOSI.
      {RF231_SPI_NPCS_PIN, RF231_SPI_NPCS_FUNCTION}   // Chip Select NPCS0.
    };

    /* add the spi options driver structure for the CC2500 */
    spi_options_t spiOptions =
    {
      RF231_SPI_NPCS,       // reg
      6500000,              // baudrate
      8,                    // bits
      1,                    // spck_delay
      1,                    // trans_delay
      1,                    // stay_act
      0,                    // spi_mode
      1                     // modfdis
    };

    /* Assign I/Os to SPI */
    gpio_enable_module(RF230_SPI_GPIO_MAP, sizeof(RF230_SPI_GPIO_MAP) / sizeof(RF230_SPI_GPIO_MAP[0]));

    /* Initialize as master */
    spi_initMaster(RF231_SPI, &spiOptions);

    /* Set selection mode: variable_ps, pcs_decode, delay */
    spi_selectionMode(RF231_SPI, 0, 0, 0);

    /* Enable SPI */
    spi_enable(RF231_SPI);

    /* setup chip registers */
    spi_setupChipReg(RF231_SPI, &spiOptions, 66000000UL);

    /*INIT EIC*/
    static const gpio_map_t EIC_GPIO_MAP = {{EXT_INT_PIN_LINE1, EXT_INT_FUNCTION_LINE1}};
    eic_options_t eic_options[EXT_INT_NB_LINES]; //1

    vPortEnterCritical();

    /*Disable_global_interrupt(); */
    INTC_register_interrupt( (__int_handler)&eic_rf230_isr, EXT_INT_IRQ_LINE1 , AVR32_INTC_INT3);

    /* Enable edge-triggered interrupt. */
    eic_options[0].eic_mode   = EIC_MODE_EDGE_TRIGGERED;

    /* Interrupt will trigger on falling edge. */
    eic_options[0].eic_edge   = EIC_EDGE_RISING_EDGE;

    /* Initialize in synchronous mode : interrupt is synchronized to the clock */
    eic_options[0].eic_async  = EIC_ASYNCH_MODE;  //new; SYNCH

    /* Set the interrupt line number. */
    eic_options[0].eic_line   = EXT_INT_LINE1;

    gpio_enable_module(EIC_GPIO_MAP,sizeof(EIC_GPIO_MAP) / sizeof(EIC_GPIO_MAP[0]));

    /* Init the EIC controller with the options */
    eic_init(&AVR32_EIC, eic_options, EXT_INT_NB_LINES);

    /* set irq-handler */
    pal_trx_irq_init(0, (void *)trx_irq_handler_cb);

    /* Enable the chosen lines and their corresponding interrupt feature. */
    eic_enable_line(&AVR32_EIC, eic_options[0].eic_line);
    eic_enable_interrupt_line(&AVR32_EIC, eic_options[0].eic_line);


    //vSemaphoreCreateBinary( xSem_rf230_ext_int );

    vPortExitCritical();
}

/**
 * @brief set irq handler
 *
 * This function sets the main AT86RF231 IRQ handler
 *
 */
void pal_trx_irq_init(uint8_t dummy, void *trx_irq_cb)
{
    /* set irq-handler */
    irq_handler = (irq_handler_t)trx_irq_cb;
}

/**
 * @brief Writes data into a transceiver register
 *
 * This function writes a value into transceiver register.
 *
 * @param addr Address of the trx register
 * @param data Data to be written to trx register
 *
 */
void pal_trx_reg_write(uint8_t addr, uint8_t data)
{
    ENTER_CRITICAL_REGION();

    /* Prepare the command byte */
    addr |= WRITE_ACCESS_COMMAND;

    HAL_SS_LOW( ); //Start the SPI transaction by pulling the Slave Select low.

    spi_write(RF231_SPI,addr);
    spi_write(RF231_SPI,data);

    HAL_SS_HIGH( ); //End the transaction by pulling the Slave Slect High.

    LEAVE_CRITICAL_REGION();
}

/**
 * @brief Reads current value from a transceiver register
 *
 * This function reads the current value from a transceiver register.
 *
 * @param addr Specifies the address of the trx register from which
 *             the data shall be read
 *
 * @return Value of the register read
 */
uint8_t pal_trx_reg_read(uint8_t addr)
{
    ENTER_CRITICAL_REGION();

    uint16_t register_value;

    /* Prepare the command byte */
    addr |= READ_ACCESS_COMMAND;

    HAL_SS_LOW( ); //Start the SPI transaction by pulling the Slave Select low.
       spi_write(RF231_SPI,addr);
       spi_write(RF231_SPI,0x00);
       spi_read(RF231_SPI,&register_value);
    HAL_SS_HIGH( ); //End the transaction by pulling the Slave Select High.

    return (uint8_t)register_value;

    LEAVE_CRITICAL_REGION();

//    return register_value;
}

/**
 * @brief Reads frame buffer of the transceiver
 *
 * This function reads the frame buffer of the transceiver.
 *
 * @param[out] data Pointer to the location to store frame
 * @param[in] length Number of bytes to be read from the frame buffer.
 */
void pal_trx_frame_read(uint8_t *data, uint8_t length)
{
    uint16_t dummy_rx_data;
    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    HAL_SS_LOW( );

    /* Send the command byte */
    spi_write(RF231_SPI, TRX_CMD_FR);

    /*
     * Done to clear the RDRF bit in the SPI status register, which will be set
     * as a result of reception of some data from the transceiver as a result
     * of SPI write operation done above.
     */
    spi_read(RF231_SPI, &dummy_rx_data);

    /*
     * Done to avoid compiler warning about variable being not used after
     * setting.
     */
    dummy_rx_data = dummy_rx_data;

    do
    {
        /* Do dummy write for initiating SPI read */
        spi_write(RF231_SPI, SPI_DUMMY_VALUE);

        /* Upload the received byte in the user provided location */
        spi_read(RF231_SPI, &dummy_rx_data);
        *data++ = (uint8_t) dummy_rx_data;
     } while (--length > 0);

    /* Stop the SPI transaction by setting SEL high */
    HAL_SS_HIGH( );

    LEAVE_CRITICAL_REGION();
}

/**
 * @brief Writes data into frame buffer of the transceiver
 *
 * This function writes data into the frame buffer of the transceiver
 *
 * @param[in] data Pointer to data to be written into frame buffer
 * @param[in] length Number of bytes to be written into frame buffer
 */
void pal_trx_frame_write(uint8_t *data, uint8_t length)
{
    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    HAL_SS_LOW( );

    /* Send the command byte */
    spi_write(RF231_SPI, TRX_CMD_FW);

    do
    {
        /* Write the user provided data in the transceiver data register */
        spi_write(RF231_SPI, *data);
        data++;

    } while (--length > 0);

    /* Stop the SPI transaction by setting SEL high */
    HAL_SS_HIGH( );

    LEAVE_CRITICAL_REGION();
}

/**
 * @brief Subregister read
 *
 * @param addr Offset of the register
 * @param mask Bit mask of the subregister
 * @param pos  Bit position of the subregister
 *
 * @return  value of the read bit(s)
 */
uint8_t pal_trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos)
{
    uint8_t ret;

    ret = pal_trx_reg_read(addr);
    ret &= mask;
    ret >>= pos;

    return ret;
}

/**
 * @brief Subregister write
 *
 * @param[in] reg_addr Offset of the register
 * @param[in] mask Bit mask of the subregister
 * @param[in] pos Bit position of the subregister
 * @param[out] new_value Data, which is muxed into the register
 */
void pal_trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t new_value)
{
    uint8_t current_reg_value;

    current_reg_value = pal_trx_reg_read(reg_addr);
    current_reg_value &= ~mask;
    new_value <<= pos;
    new_value &= mask;
    new_value |= current_reg_value;

    pal_trx_reg_write(reg_addr, new_value);
}

#ifdef ENABLE_TRX_SRAM
/**
 * @brief Writes data into SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver
 *
 * @param addr Start address in the SRAM for the write operation
 * @param data Pointer to the data to be written into SRAM
 * @param length Number of bytes to be written into SRAM
 */
void pal_trx_sram_write(uint8_t addr, uint8_t *data, uint8_t length)
{
    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPI_WRITE(TRX_CMD_SW);

    /* Send the address from which the write operation should start */
    SPI_WRITE(addr);

    do
    {
        /* Upload the user data to transceiver data register */
        SPI_WRITE(*data);
        data++;

    } while (--length > 0);

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();

    LEAVE_CRITICAL_REGION();
}
#endif  /* #ifdef ENABLE_TRX_SRAM */

#ifdef ENABLE_TRX_SRAM
/**
 * @brief Reads data from SRAM of the transceiver
 *
 * This function reads from the SRAM of the transceiver
 *
 * @param[in] addr Start address in SRAM for read operation
 * @param[out] data Pointer to the location where data stored
 * @param[in] length Number of bytes to be read from SRAM
 */
void pal_trx_sram_read(uint8_t addr, uint8_t *data, uint8_t length)
{
    uint8_t dummy_rx_data;

    PAL_WAIT_500_NS();

    ENTER_CRITICAL_REGION();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPI_WRITE(TRX_CMD_SR);

    /*
     * Done to clear the RDRF bit in the SPI status register, which will be set
     * as a result of reception of some data from the transceiver as a result
     * of SPI write operation done above.
     */
    SPI_READ(dummy_rx_data);

    /* Send the address from which the read operation should start */
    SPI_WRITE(addr);

    /*
     * Done to clear the RDRF bit in the SPI status register, which will be set
     * as a result of reception of some data from the transceiver as a result
     * of SPI write operation done above.
     */
    SPI_READ(dummy_rx_data);

    /*
     * Done to avoid compiler warning about variable being not used after
     * setting.
     */
    dummy_rx_data = dummy_rx_data;

    do
    {
        /* Do dummy write for initiating SPI read */
        SPI_WRITE(SPI_DUMMY_VALUE);

        /* Upload the received byte in the user provided location */
        SPI_READ(*data);
        data++;

    } while (--length > 0);

    SS_HIGH();

    LEAVE_CRITICAL_REGION();
}
#endif  /* #ifdef ENABLE_TRX_SRAM */

/*!
 * \brief eic_int_handler1 ISR; Interrupt handler of the External
 */
__attribute__((__naked__))
void eic_rf230_isr( void )
{
    portENTER_SWITCHING_ISR();
    eic_rf230_isr_NonNakedBehaviour();
    portEXIT_SWITCHING_ISR();
}

/*!
 * \brief eic_rf230_isr_NonNakedBehaviour ISR; DSR
 */
__attribute__((__noinline__))
long eic_rf230_isr_NonNakedBehaviour( void )
{
    //vPortEnterCritical();
    /* Mutex Parameters */
    static portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    long xSwitchRequired = FALSE;

    eic_clear_interrupt_line(&AVR32_EIC, EXT_INT_LINE1);
    irq_handler();

    //vPortExitCritical();
    return ( xSwitchRequired );
}

/**
 * @brief Transceiver interrupt handler
 *
 * This function handles the transceiver generated interrupts.
 */
void trx_irq_handler_cb(void)
{
    trx_irq_reason_t trx_irq_cause;

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
                handle_tx_end_irq(true);            // see tal_tx.c
            }
            else
            {
                handle_tx_end_irq(false);            // see tal_tx.c
            }
        }
        else  /* Other tal_state than TAL_TX_... */
        {
            /* address match interrupt */
            if(trx_irq_cause & TRX_IRQ_AMI)
            {
                /* Handle rx interrupt. */
                handle_received_frame_irq();
            }

        }
    }

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
            ASSERT("unexpected IRQ: TRX_IRQ_PLL_UNLOCK" == 0);
        }
        /* RX_START interrupt migth be set, because poll mode is enabled. */
        /*
        if (trx_irq_cause & TRX_IRQ_RX_START)
        {
            ASSERT("unexpected IRQ: TRX_IRQ_RX_START" == 0);
        }
        */
        if (trx_irq_cause & TRX_IRQ_CCA_ED_READY)
        {
            ASSERT("unexpected IRQ: TRX_IRQ_CCA_ED_READY" == 0);
        }
        /* AMI interrupt might set, because poll mode is enabled. */
        /*
        if (trx_irq_cause & TRX_IRQ_AMI)
        {
            ASSERT("unexpected IRQ: TRX_IRQ_AMI" == 0);
        }
        */
        if (trx_irq_cause & TRX_IRQ_TRX_UR)
        {
            ASSERT("unexpected IRQ: TRX_IRQ_TRX_UR" == 0);
        }
        if (trx_irq_cause & TRX_IRQ_BAT_LOW)
        {
            ASSERT("unexpected IRQ: TRX_IRQ_BAT_LOW" == 0);
        }
    }
#endif

}/* trx_irq_handler_cb() */

/**
 * @brief delay 1us with nop method
 *
 */
void delay_us(unsigned int usec)
{
    unsigned int i,ii=0;

    for(i=0;i<=usec;i++)
    {
        // wait 1µs
        for(ii=0;ii<=66;ii++)
        {
            asm volatile("nop\n\t" ::);
        }
    }
}
/* EOF */


