/**
 * @file pal_trx_access.c
 *
 * @brief Transceiver registers & Buffer accessing functions for
 *        AVR 8-Bit SPI based MCUs.
 *
 * This file implements functions for reading and writing transceiver
 * registers and transceiver buffer for AVR 8-Bit SPI based MCUs.  The
 * decision whether to use the implementation within this file is
 * based on the presence of the macro PAL_USE_SPI_TRX, to be defined
 * in the respective board's pal_config.h.  Absence of this macro
 * implies the TAL for the respective board supplies its own
 * implementation of this API.
 *
 * $Id: pal_trx_access.c 16886 2009-07-31 14:04:55Z uwalter $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */
/* === Includes ============================================================ */

#include <stdint.h>
#include "inc/at86.h"
#include "inc/pal_config.h"

#include <avr/io.h>             // Registerdefinitionen
#include <inttypes.h>           // Datentypen
#include <avr/Interrupt.h>      // Interrupts
#include <avr/wdt.h>            // Watchdog
#include <ctype.h>              // Chars
#include <avr/sleep.h>          // Sleepmode
#include <util/delay.h>         // Delays

/* === Macros ============================================================== */

/* === Types =============================================================== */

/**
 * This is a typedef of the function which is called from SPI end
 */
typedef void (*spi_done_handler_t)(void);

/**
 * Typdef SPI States
 */
typedef enum
{
    SPI_IDLE,
    SPI_READ,
    SPI_WRITE
} spi_state_t;

/*
 * Function pointers to store the callback function of
 * the transceiver interrupt
 */
static irq_handler_t irq_handler[NO_OF_TRX_IRQS];

#ifdef NON_BLOCKING_SPI
static uint8_t spi_remaining_bytes;
spi_state_t spi_state = SPI_IDLE;
static uint8_t *spi_data_ptr;
/*
 * Function pointers to store the callback function of
 * the SPI done handling
 */
static spi_done_handler_t spi_done_handler;
#endif

/* === Implementation ====================================================== */

/**
 * @brief Initializes the transceiver interface
 *
 * This function initializes the transceiver interface.
 */
void trx_interface_init(void)
{
    TRX_INIT();
}

/**
 * @brief SPI done callback initialization
 */
#ifdef NON_BLOCKING_SPI
void pal_spi_done_cb_init(void *spi_done_cb)
{
    spi_done_handler = (spi_done_handler_t)spi_done_cb;
}
#endif

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
    cli();
    ENTER_TRX_REGION();

#ifdef NON_BLOCKING_SPI
    while (spi_state != SPI_IDLE)
    {
        /* wait until SPI gets available */
    }
#endif

    /* Prepare the command byte */
    addr |= WRITE_ACCESS_COMMAND;

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the Read command byte */
    SPDR0 = addr;
    SPI_WAIT();

    /* Write the byte in the transceiver data register */
    SPDR0 = data;
    SPI_WAIT();

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();

    LEAVE_TRX_REGION();
    sei();
}

/**
 * @brief Reads current value from a transceiver register
 *
 * This function reads the current value from a transceiver register.
 *
 * @param addr Specifies the address of the trx register
 * from which the data shall be read
 *
 * @return value of the register read
 */
uint8_t pal_trx_reg_read(uint8_t addr)
{
    uint8_t register_value;

    cli();

    ENTER_TRX_REGION();

#ifdef NON_BLOCKING_SPI
    while (spi_state != SPI_IDLE)
    {
        /* wait until SPI gets available */
    }
#endif

    /* Prepare the command byte */
    addr |= READ_ACCESS_COMMAND;

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the Read command byte */
    SPDR0 = addr;
    SPI_WAIT();

    /* Do dummy read for initiating SPI read */
    SPDR0 = SPI_DUMMY_VALUE;
    SPI_WAIT();

    /* Read the byte received */
    register_value = SPDR0;

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();

    LEAVE_TRX_REGION();

    sei();
    return register_value;
}

/**
 * @brief Reads frame buffer of the transceiver
 *
 * This function reads the frame buffer of the transceiver.
 *
 * @param[out] data Pointer to the location to store frame
 * @param[in] length Number of bytes to be read from the frame
 * buffer.
 */
void pal_trx_frame_read(uint8_t *data, uint8_t length)
{
    /* Assumption: This function is called within ISR. */
    cli();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPDR0 = TRX_CMD_FR;
    SPI_WAIT();
    SPDR0 = SPI_DUMMY_VALUE;

    if (length != 1)
    {
        do
        {
            uint8_t temp;

            SPI_WAIT();
            /* Upload the received byte in the user provided location */
            temp = SPDR0;
            SPDR0 = SPI_DUMMY_VALUE; /* Do dummy write for initiating SPI read */
            *data = temp;
            data++;
            length--;
        } while (length > 1);
    }

    SPI_WAIT(); /* Wait until last bytes is transmitted. */
    *data = SPDR0;

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();
    sei();
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
#ifndef NON_BLOCKING_SPI
    cli();
    /* Assumption: The TAL has already disabled the trx interrupt. */

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Start SPI transfer by sending the command byte */
    SPDR0 = TRX_CMD_FW;

    SPI_WAIT();

    do
    {
        uint8_t temp = *data;
        data++;
        length--;
        SPI_WAIT();
        SPDR0 = temp;
    } while (length > 0);

    SPI_WAIT(); /* Wait until last bytes is transmitted. */

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();
    sei();
#else

    spi_state = SPI_WRITE;
    spi_remaining_bytes = length;
    spi_data_ptr = data;
    SPI_IRQ_ENABLE();

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Start SPI transfer by sending the command byte */
    SPDR0 = TRX_CMD_FW;
#endif
}

/**
 * @brief SPI transfer ISR
 *
 * This function handles the non-blocking SPI transfer.
 *
 */
#ifdef NON_BLOCKING_SPI
ISR(SPI_STC_vect)
{
    if (spi_remaining_bytes > 0)
    {
        /* Write the user provided data into the SPI data register */
        SPDR0 = *spi_data_ptr++;
        spi_remaining_bytes--;
    }
    else
    {
        /* Complete the SPI transaction by setting SEL high */
        SS_HIGH();
        SPI_IRQ_DISABLE();
        spi_state = SPI_IDLE;
        /* avoid that the trx interrupts ongoing SPI transfer */
        pal_trx_irq_enable(TRX_MAIN_IRQ_HDLR_IDX);
    }
}
#endif  /* #ifdef NON_BLOCKING_SPI */

/**
 * @brief Subregister read
 *
 * @param   addr  offset of the register
 * @param   mask  bit mask of the subregister
 * @param   pos   bit position of the subregister
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
 * @param[in]   reg_addr  Offset of the register
 * @param[in]   mask  Bit mask of the subregister
 * @param[in]   pos   Bit position of the subregister
 * @param[out]  new_value  Data, which is muxed into the register
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
    ENTER_TRX_REGION();

#ifdef NON_BLOCKING_SPI
    while (spi_state != SPI_IDLE)
    {
        /* wait until SPI gets available */
    }
#endif

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPDR0 = TRX_CMD_SW;
    SPI_WAIT();

    /* Send the address from which the write operation should start */
    SPDR0 = addr;
    SPI_WAIT();

    do
    {
        uint8_t temp = *data;
        data++;
        length--;
        SPI_WAIT();
        SPDR0 = temp;
    } while (length > 0);

    SPI_WAIT(); /* Wait until last bytes is transmitted. */

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();

    LEAVE_TRX_REGION();
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
    PAL_WAIT_500_NS();

    ENTER_TRX_REGION();

#ifdef NON_BLOCKING_SPI
    while (spi_state != SPI_IDLE)
    {
        /* wait until SPI gets available */
    }
#endif

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPDR0 = TRX_CMD_SR;
    SPI_WAIT();

    /* Send the address from which the read operation should start */
    SPDR0 = addr;
    SPI_WAIT();

    SPDR0 = SPI_DUMMY_VALUE;
    length--;

    while (length > 0)
    {
        uint8_t temp;

        SPI_WAIT();
        /* Upload the received byte in the user provided location */
        temp =  SPDR0;
        SPDR0 = SPI_DUMMY_VALUE; /* Do dummy write for initiating SPI read */
        *data++ = temp;
        length--;
    }

    SPI_WAIT(); /* Wait until last bytes is transmitted. */
    *data++ = SPDR0; // read last data byte

    SS_HIGH();

    LEAVE_TRX_REGION();
}
#endif  /* #ifdef ENABLE_TRX_SRAM */

#ifdef ENABLE_TRX_SRAM
/**
 * @brief Writes and reads data into/from SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver and
 * simultaneously reads the bytes.
 *
 * @param addr Start address in the SRAM for the write operation
 * @param idata Pointer to the data written/read into/from SRAM
 * @param length Number of bytes written/read into/from SRAM
 */
void pal_trx_aes_wrrd(uint8_t addr, uint8_t *idata, uint8_t length)
{
    uint8_t *odata;

    PAL_WAIT_500_NS();

    ENTER_TRX_REGION();

#ifdef NON_BLOCKING_SPI
    while (spi_state != SPI_IDLE)
    {
        /* wait until SPI gets available */
    }
#endif

    /* Start SPI transaction by pulling SEL low */
    SS_LOW();

    /* Send the command byte */
    SPDR0 = TRX_CMD_SW;
    SPI_WAIT();

    /* write SRAM start address */
    SPDR0 = addr;
    SPI_WAIT();

    /* now transfer data */
    odata = idata;

    /* write data byte 0 - the obtained value in SPDR0 is meaningless */
    SPDR0 = *idata++;
    SPI_WAIT();

    /* process data bytes 1...length-1: write and read */

    uint8_t itemp;  // use variable to accelerate SPI transfer
    SPDR0 = *idata++;
    length--;
    itemp = *idata;

    while (length > 0)
    {
        SPI_WAIT();
        *odata = SPDR0;
        SPDR0 = itemp;
        odata++;
        length--;
        idata++;
        itemp = *idata;
    }

    SPI_WAIT();

    /* to get the last data byte, write some dummy byte */
    SPDR0 = SPI_DUMMY_VALUE;
    SPI_WAIT();
    *odata = SPDR0;

    /* Stop the SPI transaction by setting SEL high */
    SS_HIGH();
    LEAVE_TRX_REGION();
}
#endif  /* #ifdef ENABLE_TRX_SRAM */

/* === Prototypes ========================================================== */

/* === Implementation ====================================================== */

/**
 * @brief Initializes the transceiver interrupts
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver interrupts
 *
 * @param trx_irq_num Transceiver interrupt line to be initialized
 * @param trx_irq_cb Callback function for the given transceiver
 * interrupt
 */
void pal_trx_irq_init(trx_irq_hdlr_idx_t trx_irq_num, void *trx_irq_cb)
{
    /*
     * Set the handler function.
     * The handler is set before enabling the interrupt to prepare for spurious
     * interrupts, that can pop up the moment they are enabled
     */
    irq_handler[trx_irq_num] = (irq_handler_t)trx_irq_cb;

    if (trx_irq_num == TRX_MAIN_IRQ_HDLR_IDX)
    {
        /* Init Main TRX interrupt */
        /* Select rising edge */
        EICRA |= _BV(ISC00) | _BV(ISC01);
        /* clear pending interrupt */
        EIFR = _BV(INTF0);
    }
#ifndef ANTENNA_DIVERSITY
    else if (trx_irq_num == TRX_TSTAMP_IRQ_HDLR_IDX)
    {
        /* Init RX TIME STAMP interrupt */
        /* The input capture interrupt of timer is disabled */
        TIMSK1 &= ~(_BV(ICIE1));
        /* Rising edge on input capture pin used to trigger IRQ */
        TCCR1B |= (_BV(ICES1));
        /* Input capture flag is cleared */
        TIFR1 |= (_BV(ICF1));
    }
#endif
}

/**
 * @brief ISR for transceiver's main interrupt
 */
ISR(TRX_MAIN_ISR_VECTOR)
{
    irq_handler[TRX_MAIN_IRQ_HDLR_IDX]();
}

/**
 * @brief ISR for transceiver's RX TIME STAMP interrupt
 */
#ifndef ANTENNA_DIVERSITY
ISR(TRX_TSTAMP_ISR_VECTOR)
{
    irq_handler[TRX_TSTAMP_IRQ_HDLR_IDX]();
}
#endif

/* EOF */
