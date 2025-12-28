/* This source file is part of the ATMEL AVR32-SoftwareFramework-AT32UC3A-1.4.0 Release */

/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief This file controls the UART USB functions.
 *
 * These functions allow to use en USB endpoint as we would do using an UART.
 * This is particurly well suited for USB CDC class.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a USB module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ***************************************************************************/

/* Copyright (C) 2006-2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


//_____  I N C L U D E S ___________________________________________________
#include "compiler.h"
#include "conf_usb.h"
#include "usb_drv.h"
#include "uart_usb_lib.h"
#include "main_app.h"

//_____ D E F I N I T I O N S ______________________________________________
U8    dev_rx_cnt;
U8    dev_tx_cnt;
U8    nmb_packets;

void uart_usb_init(void)
{
  dev_rx_cnt=0 ;
  dev_tx_cnt=0 ;
}

// Functions that manage characters input through USB
Bool uart_usb_test_hit(void)
{
  if( dev_rx_cnt==0 )
  {
    if( Is_usb_out_received(RX_EP) )
    {
      dev_rx_cnt = Usb_byte_count(RX_EP);
      Usb_reset_endpoint_fifo_access(RX_EP);
      if( dev_rx_cnt==0 )
      {
        Usb_ack_out_received_free(RX_EP);
        return FALSE;
      }
      else
        return TRUE;
    }
    else
      return FALSE;
  }
  else
    return TRUE;
}

char uart_usb_getchar(void)
{
  register char data_rx;

  while( !uart_usb_test_hit() )
  {
      vTaskDelay( 10 );
  }

  data_rx=Usb_read_endpoint_data(RX_EP, 8);
  dev_rx_cnt--;
  if( dev_rx_cnt==0 ) Usb_ack_out_received_free(RX_EP);

  return data_rx;
}

// Functions that manage characters output through USB
int errrr = 0;
Bool uart_usb_tx_ready(void)
{
  if( !Is_usb_write_enabled(TX_EP) )
    return FALSE;

  return TRUE;
}

int uart_usb_putchar(int data_to_send, unsigned char enable_fast) 
{
  uint32_t timeout = 0;

  while( !uart_usb_tx_ready() )
  {
      timeout++;
  }

  if( dev_tx_cnt==0 )
  {
    Usb_reset_endpoint_fifo_access(TX_EP);
  }
  Usb_write_endpoint_data(TX_EP, 8, data_to_send);
  dev_tx_cnt++;
  if( !uart_usb_tx_ready() ) //If Endpoint full -> flush
  {
      if(enable_fast)
         uart_usb_flush();
      else
         uart_usb_flush_slow();
  }

  return data_to_send;
}



void usb_cdc_performance_print( int32_t *data, const uint16_t len )
{
    static uint32_t i = 0, index = 0, x = 0;
    uint32_t timeout = 0;

    if( len > 4096  )
        return;

    /* max: 64 */
    for( x = 0; x < 64; x++ )
    {
        /* Reset fifo */
        Usb_reset_endpoint_fifo_access( TX_EP );

        /* write 64 bytes */
        for( i = 0; i < 16; i++ )
        {
            //while( !uart_usb_tx_ready() );
            while( !uart_usb_tx_ready() )
            {
                /* only send data if terminal is connected */
                timeout++;
                if(timeout > 300)
                    return;
            }

            /* Write Payload data into fifo */
            Usb_write_endpoint_data( TX_EP, 32, *( data + index ) );
            index++;

            /* End of payload? */
            if( index > (len / 4) )
            {
                /* Send current packet */
                Usb_ack_in_ready_send( TX_EP );

                /* We need a zero length packet */
                if(i == 15)
                {
                    //while( !uart_usb_tx_ready() );
                    while( !Is_usb_write_enabled(TX_EP) );
                    Usb_ack_in_ready_send(TX_EP);
                }

                index = 0;
                return;
            }

        }

        /* Send current packet */
        Usb_ack_in_ready_send( TX_EP );
    }
    index = 0;

}

void uart_usb_flush (void)
{
  Bool zlp=FALSE;
  if( dev_tx_cnt!=0 )
  {
    nmb_packets++;
    if(!Is_usb_write_enabled(TX_EP) && nmb_packets > 15)
    {
        // Endpoint full, need ZLP
       zlp=TRUE;
       nmb_packets=0;
    }

    //Usb_send_in(TX_EP);
    Usb_ack_in_ready_send(TX_EP);

    if( zlp==TRUE )
    {
       while( !Is_usb_write_enabled(TX_EP) )
       {
       }
       Usb_ack_in_ready_send(TX_EP);              // ...and Send ZLP
    }
    dev_tx_cnt = 0;
  }
}

void uart_usb_flush_slow (void)
{
  Bool zlp=FALSE;
  if( dev_tx_cnt!=0 )
  {
    if(!Is_usb_write_enabled(TX_EP))
    {
        // Endpoint full, need ZLP
       zlp=TRUE;
    }

    //Usb_send_in(TX_EP);
    Usb_ack_in_ready_send(TX_EP);

    if( zlp==TRUE )
    {
       while( !Is_usb_write_enabled(TX_EP) )
       {
           // Wait Endpoint ready...
       }
       Usb_ack_in_ready_send(TX_EP);              // ...and Send ZLP
    }
    dev_tx_cnt = 0;
  }
}






