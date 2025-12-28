/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief FreeRTOS Serial Port management for AVR32 UC3.
 * BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR USART.
 *
 * - Compiler:           GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices can be used.
 * - AppNote:
 *
 * \author               flm
 *
 *****************************************************************************/

/* Includes */
#include "board.h"
#include <avr32/io.h>
#include "gpio.h"
#include "usart.h"
#include "main_app.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "serial.h"
#include <stdarg.h>
#include "print_funcs.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "flash.h"
#include "flashc.h"
#include "wdt.h"
#include "eic.h"
#include "ad5410.h"
#include "at86.h"
#include "ieee_const.h"

/* Constants to setup and access the USART. */
#define serINVALID_COMPORT_HANDLER        ( ( xComPortHandle ) 0 )
#define serINVALID_QUEUE                  ( ( xQueueHandle ) 0 )
#define serHANDLE                         ( ( xComPortHandle ) 1 )
#define serNO_BLOCK                       ( ( portTickType ) 0 )

/* Prototypes */
void print_main_menu( void );
void print_transceiver_config( void );
void print_tx_command( void );

/* Global variables */
xComPortHandle data_usart_handle = NULL;
xTaskHandle xHandleCOMM;

int8_t usart_data_string[2048] = { 0 };
extern cc2500_data_struct cc2500_nmb_1;
extern volatile frame_info_union rx_frame;
extern volatile frame_info_union tx_frame;
extern int32_t avg_values_result[5];
extern uint8_t index_average;
extern cock_data cock_d;

/* RX and TX queues */
static xQueueHandle xRxedChars;
static xQueueHandle xCharsForTx;

/* Forward declaration. */
void vTaskCOMM( void *pvParameters );
static void vprvSerialCreateQueues( unsigned portBASE_TYPE uxQueueLength,
                                    xQueueHandle *pxRxedChars,
                                    xQueueHandle *pxCharsForTx );

/* USART RX+TX-DSR */
__attribute__((__noinline__))
static portBASE_TYPE prvUSART_ISR_NonNakedBehaviour( void )
{
    /* Now we can declare the local variables. */
    signed portCHAR cChar;
    portBASE_TYPE xTaskWokenByTx = pdFALSE, xTaskWokenByRx = pdFALSE;
    unsigned portLONG ulStatus;
    volatile avr32_usart_t *usart = serialPORT_USART;
    portBASE_TYPE retstatus;
    static portBASE_TYPE xHigherPriorityTaskWoken;

    /* What caused the interrupt? */
    ulStatus = usart->csr & usart->imr;

    if (ulStatus & AVR32_USART_CSR_TXRDY_MASK)
    {
        /* The interrupt was caused by the THR becoming empty.  Are there any
        more characters to transmit?
        Because FreeRTOS is not supposed to run with nested interrupts, put all OS
        calls in a critical section . */
        portENTER_CRITICAL();
            retstatus = xQueueReceiveFromISR( xCharsForTx, &cChar, &xTaskWokenByTx );
        portEXIT_CRITICAL();

        if (retstatus == pdTRUE)
        {
            /* A character was retrieved from the queue so can be sent to the THR now. */
            usart->thr = cChar;
        }
        else
        {
            /* Queue empty, nothing to send so turn off the Tx interrupt. */
            usart->idr = AVR32_USART_IDR_TXRDY_MASK;
        }
    }

    if (ulStatus & AVR32_USART_CSR_RXRDY_MASK)
    {
        xHigherPriorityTaskWoken = pdFALSE;

        /* The interrupt was caused by the receiver getting data. */
        cChar = usart->rhr;

        portENTER_CRITICAL();
            retstatus = xQueueSendFromISR(xRxedChars, &cChar, &xHigherPriorityTaskWoken);
        portEXIT_CRITICAL();

        if( retstatus )
        {
            xTaskWokenByRx = pdTRUE;
        }
    }

    /* The return value will be used by portEXIT_SWITCHING_ISR() to know if it
    should perform a vTaskSwitchContext(). */
    return ( xTaskWokenByTx || xTaskWokenByRx );
}

/* USART RX+TX ISR */
__attribute__((__naked__))
static void vUSART_ISR( void )
{
    /* This ISR can cause a context switch, so the first statement must be a
    call to the portENTER_SWITCHING_ISR() macro.  This must be BEFORE any
    variable declarations. */
    portENTER_SWITCHING_ISR();

    prvUSART_ISR_NonNakedBehaviour();

    /* Exit the ISR.  If a task was woken by either a character being received
    or transmitted then a context switch will occur. */
    portEXIT_SWITCHING_ISR();
}

/* Init the serial port for the Minimal implementation. */
xComPortHandle xSerialPortInitMinimal( unsigned portLONG ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
    static const gpio_map_t USART_GPIO_MAP =
    {
        { serialPORT_USART_RX_PIN, serialPORT_USART_RX_FUNCTION },
        { serialPORT_USART_TX_PIN, serialPORT_USART_TX_FUNCTION }
    };

    xComPortHandle    xReturn = serHANDLE;
    volatile avr32_usart_t  *usart = serialPORT_USART;

    // USART options.
    usart_options_t USART_OPTIONS =
    {
        .baudrate     = 115200,//230400,
        .charlength   = 8,
        .paritytype   = USART_NO_PARITY,
        .stopbits     = USART_1_STOPBIT,
        .channelmode  = USART_NORMAL_CHMODE
    };

    USART_OPTIONS.baudrate = ulWantedBaud;

    /* Create the rx and tx queues. */
    vprvSerialCreateQueues( uxQueueLength, &xRxedChars, &xCharsForTx );

    /* SET MAX2334E RS232 Transceiver Pins*/
    gpio_set_gpio_pin( AVR32_PIN_PB10 ); // FORCEOFF
    gpio_clr_gpio_pin( AVR32_PIN_PB11 ); // FORCEON 0 = use auto-powerdown
    gpio_enable_pin_glitch_filter(AVR32_PIN_PB13); // INVALID signal
    /* Configure USART. */
    if( ( xRxedChars != serINVALID_QUEUE ) &&
      ( xCharsForTx != serINVALID_QUEUE ) &&
      ( ulWantedBaud != ( unsigned portLONG ) 0 ) )
    {
        portENTER_CRITICAL();
        {
            /**
            ** Configure USART.
            **/
            /* Enable USART RXD & TXD pins. */
            gpio_enable_module( USART_GPIO_MAP, sizeof( USART_GPIO_MAP ) / sizeof( USART_GPIO_MAP[0] ) );

            // Initialize USART in RS232 mode.
            usart_init_rs232(usart, &USART_OPTIONS, configPBA_CLOCK_HZ);

            /* We're not fully done yet: disable receiver and transmitter. */
            usart->cr |= AVR32_USART_CR_RXDIS_MASK | AVR32_USART_CR_TXDIS_MASK;

            /* Register the USART interrupt handler to the interrupt controller and
             enable the USART interrupt. */
            INTC_register_interrupt((__int_handler)&vUSART_ISR, serialPORT_USART_IRQ, AVR32_INTC_INT1);

            /* Enable USART interrupt sources (but not Tx for now)... */
            usart->ier = AVR32_USART_IER_RXRDY_MASK;

            /* Enable receiver and transmitter... */
            usart->cr |= AVR32_USART_CR_TXEN_MASK | AVR32_USART_CR_RXEN_MASK;
        }
        portEXIT_CRITICAL();
    }
    else
    {
        xReturn = serINVALID_COMPORT_HANDLER;
    }

    return xReturn;
}

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed portCHAR *pcRxedChar, portTickType xBlockTime )
{
    /* The port handle is not required as this driver only supports UART0. */
    ( void ) pxPort;

    /* Get the next character from the buffer.  Return false if no characters
    are available, or arrive before xBlockTime expires. */
    if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}

void vSerialPutString( xComPortHandle pxPort, const signed portCHAR * const pcString, unsigned portSHORT usStringLength )
{
    signed portCHAR *pxNext;

    /* NOTE: This implementation does not handle the queue being full as no
    block time is used! */

    /* The port handle is not required as this driver only supports UART0. */
    ( void ) pxPort;

    /* Send each character in the string, one at a time. */
    pxNext = ( signed portCHAR * ) pcString;
    while( *pxNext )
    {
        xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
        pxNext++;
    }
}

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed portCHAR cOutChar, portTickType xBlockTime )
{
    volatile avr32_usart_t  *usart = serialPORT_USART;

    /* Place the character in the queue of characters to be transmitted. */
    if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) != pdPASS )
    {
        return pdFAIL;
    }

    /* Turn on the Tx interrupt so the ISR will remove the character from the
    queue and send it.   This does not need to be in a critical section as
    if the interrupt has already removed the character the next interrupt
    will simply turn off the Tx interrupt again. */
    usart->ier = (1 << AVR32_USART_IER_TXRDY_OFFSET);

    return pdPASS;
}

/*
 * Create the rx and tx queues.
 */
static void vprvSerialCreateQueues(  unsigned portBASE_TYPE uxQueueLength, xQueueHandle *pxRxedChars, xQueueHandle *pxCharsForTx )
{
    /* Create the queues used to hold Rx and Tx characters. */
    xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
    xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );

    /* Pass back a reference to the queues so the serial API file can
    post/receive characters. */
    *pxRxedChars = xRxedChars;
    *pxCharsForTx = xCharsForTx;
}

void init_data_usart(void)
{
    unsigned char ucParameterToPass = 0;

    /* Init USART 230400 Baud, FIFO = 120 Bytes*/
    data_usart_handle = xSerialPortInitMinimal(115200, 2048);

    /* Create the task for USART-COMM-Task */
    xTaskCreate( vTaskCOMM,                         // pvTaskCode
                ( signed portCHAR * ) "COM_TSK",    // pcName
                1024,                               // usStackDepth
                &ucParameterToPass,                 // pvParameters
                tskIDLE_PRIORITY + 2,               // uxPriority
                &xHandleCOMM);                      // pvCreatedTask
}

int data_usart_printf( const char *fmt, ... )
{
    /* check if RS232 cable is connected */
    if( gpio_get_pin_value(AVR32_PIN_PB13) )
    {

        /* use sprintf from printf-stdarg.c!!! in some cases sprintf from stdlib
         * allocates 2KB on stack and causes stack to overflow!!! @FM */
        //extern int sprintf(char *out, const char *format, ...);
        taskENTER_CRITICAL();

        static va_list ap;
        static int ret;

        va_start( ap, fmt );
        ret = vsprintf( (char *)usart_data_string, fmt, ap );
        va_end( ap );
        vSerialPutString( data_usart_handle, usart_data_string, sizeof(usart_data_string) );

        taskEXIT_CRITICAL();
        return ret;
    }
    else
    {
        return 0;
    }
}

void data_usart_get_string( char* buffer, int buffer_len )
{
    int i = 0;
    static signed portCHAR rec_char = 0x00;
    uint8_t status;

    rec_char = 0x00;

    status = xSerialGetChar(data_usart_handle, &rec_char ,10);
    while( ( rec_char != 0x0d ) && (i < buffer_len - 1) )
    {
        if ( status == pdTRUE)
        {
            buffer[i++] = rec_char;
            data_usart_printf( "%c", rec_char );
        }
        status = xSerialGetChar(data_usart_handle, &rec_char ,10);
    }
    buffer[i] = '\0';
    data_usart_printf( "\r\n" );
}

void vTaskCOMM( void *pvParameters )
{
    static signed portCHAR rec_char, dummy_char;
    float fehlerrate = 0;
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    static char buffer[10] = { 0 };
    int val1;

    while( 1 )
    {
        /* Zeichen einlesen */
        if(xSerialGetChar(data_usart_handle, &rec_char ,100) == pdTRUE)
        {
            /* Receive "\r" --> Empty Queue */
            xSerialGetChar( data_usart_handle, &dummy_char ,50 );

            /* Befehl auswerten */
            switch( rec_char )
            {
                /* Task-Ausgabe */
                case 'x':
                    cc2500_nmb_1.streaming = 0;
                    data_usart_printf( "\n\n\n\rTask          State  Priority  Stack    #\r\n************************************************\r\n" );
                    /* list of tasks and their status... */
                    taskENTER_CRITICAL();
                    vTaskList( ( signed char * )usart_data_string );
                    data_usart_printf( "%s \n\r", usart_data_string );
                    taskEXIT_CRITICAL();
                    break;

               /* Empfangs-Statistik */
                case 'i':
                    /* streaming = off */
                    cc2500_nmb_1.streaming = 0;
                    vTaskDelay(6);

                    data_printf( "\n\n\rReceive statistics:\r\n" );
                    data_printf( "Total packets: %u \r\n", cc2500_nmb_1.rec_total );
                    data_printf( "CRC ERROR: %u \r\n", cc2500_nmb_1.error_crc_cnt );
                    data_printf( "Frames missing: %u \r\n",cc2500_nmb_1.frames_missing );

                    if(cc2500_nmb_1.rec_total  > 0)
                    {
                        fehlerrate = ( ( ( float )cc2500_nmb_1.error_crc_cnt * 100 ) / (( float )cc2500_nmb_1.rec_total + (float)cc2500_nmb_1.error_crc_cnt ) );
                        usb_uart_printf( "CRC_ERR: %.2f %% \n\r", fehlerrate );

                        fehlerrate = ( ( ( float )cc2500_nmb_1.frames_missing * 100 ) / (( float )cc2500_nmb_1.rec_total + (float)cc2500_nmb_1.frames_missing ));
                        data_printf( "MISSING_ERR: %.2f %% \r\n\n\n", fehlerrate );
                    }
                    else
                    {
                        data_printf( "CRC_ERR:\r\n");
                        data_printf( "MISSING_ERR:\r\n\n\n");
                    }
                    break;

                    /* Streaming aus/an */
                    case 's':
                        if( cc2500_nmb_1.streaming == 0 )
                        {
                            cc2500_nmb_1.streaming = 0;
                            vTaskDelay(6);
                            data_printf( "\r\n\nStreamingmode = 1 (reduced data)\r\n\r\n" );
                            data_printf( "Fz     Mz     Mxy    RSSI   BATT. TIME / s \r\n" );
                                      //  +00000;+00000;+00000;-59dBm;4.30V;281.738375s" );
                            cc2500_nmb_1.streaming = 1;
                        }

                        else if( cc2500_nmb_1.streaming == 1 )
                        {
                            cc2500_nmb_1.streaming = 0;
                            vTaskDelay(6);
                            data_printf( "\r\n\nStreamingmode = 2 (LogView)\r\n" );
                            cc2500_nmb_1.streaming = 2;
                        }
                        else if( cc2500_nmb_1.streaming == 2 )
                        {
                            cc2500_nmb_1.streaming = 0;
                            vTaskDelay(6);
                            data_printf( "\r\n\nStreamingmode = 3 (full data)\r\n" );
                            data_printf( "Fz     Mz     Mxy    TIME / s \r\n" );
                                        //-00022;+00016;+00187;520.049000
                            cc2500_nmb_1.streaming = 3;
                        }
                        else if( cc2500_nmb_1.streaming == 3 )
                        {
                            cc2500_nmb_1.streaming = 0;
                            vTaskDelay(6);
                            data_printf( "\r\n\nStreamingmode = 4 full raw data\r\n" );
                            data_printf( "Fz     Mz     Mx     My     TIME / s \r\n" );
                                        //+00053;+00035;-00022;-00034;2.977338
                            cc2500_nmb_1.streaming = 4;
                        }
                        else if( cc2500_nmb_1.streaming == 4 )
                        {
                            cc2500_nmb_1.streaming = 0;
                            vTaskDelay(6);
                            data_printf( "\r\n\nStreamingmode = 0 (no streaming)\r\n" );
                            cc2500_nmb_1.streaming = 0;
                        }
                        cc2500_nmb_1.set_dac_manual = 0;
                        break;

                /* Empfangsstatistik zurücksetzen */
                case 'r':
                    /* streaming = off */
                    cc2500_nmb_1.streaming = 0;
                    vTaskDelay(6);

                    cc2500_nmb_1.streaming = 0;
                    cc2500_nmb_1.rec_total = 0;
                    cc2500_nmb_1.error_crc_cnt = 0;
                    cc2500_nmb_1.frames_missing = 0;
                    data_printf( "\r\n\nStatistik resetted\r\n" );
                    data_printf( "Time resetted\r\n\n" );
                    break;

                /* Set moving average size */
                case 'm':
                    /* streaming = off */
                     cc2500_nmb_1.streaming = 0;
                     vTaskDelay(6);

                    data_printf( "\r\nFilter does only affect streaming mode 3 and 4!\r\n1 = Disabled (raw data)\r\nEnter size: " );
                    data_usart_get_string( buffer, 10 );
                    val1 = atoi( buffer );
                    if( (val1 < 100) && (val1 != 0) )
                    {
                        cc2500_nmb_1.moving_average_size = val1;
                        index_average = 0;
                        avg_values_result[0] = 0;
                        avg_values_result[1] = 0;
                        avg_values_result[2] = 0;
                        avg_values_result[3] = 0;
                        avg_values_result[4] = 0;
                    }
                    else
                        data_printf( "Valid values: 1-99! " );
                    break;

                /* DAC setzen */
                case 'd':
                    data_usart_printf( "Set DAC: Enter value [dec]: " );
                    data_usart_get_string( buffer, 10 );
                    val1 = atoi( buffer );
                    cc2500_nmb_1.set_dac_manual = 1;

                    ad5410_set_current( (uint16_t)val1 );

                    volatile uint16_t tempp;
                    ad5410_read_control_reg((uint16_t*)&tempp);
                    data_usart_printf( "ctrl_reg: 0x%x \r\n", tempp );
                    ad5410_read_data_reg((uint16_t*)&tempp);
                    data_usart_printf( "data_reg: %d \r\n", tempp );
                    ad5410_read_status_reg((uint16_t*)&tempp);
                    data_usart_printf( "status_reg: 0x%x \r\n", tempp );

                    break;

                /* PWM setzen*/
                case 'p':
                    data_usart_printf( "Store Value in Userpage: default: 1950; akt. Wert: %d [dec]: ", *( unsigned short * )0x808001F8 );
                    data_usart_get_string( buffer, 10 );
                    val1 = atoi( buffer );

                    taskENTER_CRITICAL();
                    /* Frequenz in User Page speichern */
                    flashc_memset16( ( unsigned char * )0x808001F8, ( U16 )val1, 2, TRUE );
                    taskEXIT_CRITICAL();

                    pwm_disable();
                    //pwm0( ( unsigned int )val1 );
                    pwm_enable();
                    break;

                /* n: reset */
                case 'n':
                    data_usart_printf( "reset MCU\r\n" );
                    vTaskDelay(500);
                    data_usart_printf( "###\r\n" );
                    vTaskDelay(500);
                    data_usart_printf( "##\r\n" );
                    vTaskDelay(500);
                    data_usart_printf( "#\r\n" );
                    vTaskDelay(500);
                    data_usart_printf( "reset; wait 2 more seconds \r\n" );

                    wdt_disable();
                    wdt_enable(2000000);
                    while (1);
                    break;

                /* Zero strain gauges */
                case 'z':
                    /* streaming = off */
                    cc2500_nmb_1.streaming = 0;
                    vTaskDelay(6);

                    zero_strain_gauges();
                    data_printf( "\r\n\r\nSetting strain gauges zero:\r\nOFFSET1: %d\r\nOFFSET2: %d\r\nOFFSET3: %d\r\nOFFSET4: %d\r\n\r\n",cock_d.offsets[0],
                                                                                                                                                cock_d.offsets[1],
                                                                                                                                                cock_d.offsets[2],
                                                                                                                                                cock_d.offsets[3] );
                    index_average = 0;
                    avg_values_result[0] = 0;
                    avg_values_result[1] = 0;
                    avg_values_result[2] = 0;
                    avg_values_result[3] = 0;
                    avg_values_result[4] = 0;

                    rtc_set_value(&AVR32_RTC, 0);
                    break;

               /* logview-start */
                case 17:
                    cc2500_nmb_1.streaming = 2;
                    break;
                case 18:
                    cc2500_nmb_1.streaming = 2;
                    break;
                case 19:
                    cc2500_nmb_1.streaming = 2;
                    break;
                case 20:
                    cc2500_nmb_1.streaming = 2;
                    break;

               /* Tx user data */
               case 't':
                   cc2500_nmb_1.streaming = 0;
                   vTaskDelay(5);

                   /* print menu */
                   print_tx_command();
                   break;

               /* Config Transceiver */
               case 'c':
                   /* print menu  */
                   print_transceiver_config();

                   /* streaming off */
                   cc2500_nmb_1.streaming = 0;
                   break;

                /* Hilfe Menü */
                default:
                    /* streaming aus */
                    cc2500_nmb_1.streaming = 0;

                    /* set_dac_manual = 0; */
                    cc2500_nmb_1.set_dac_manual = 0;

                    /* print main menu */
                    print_main_menu();
                    break;
            }
        }
    }
}

void print_main_menu( void )
{
    /* Hilfe-Screen drucken */
    data_printf( "\f\r\n\n\n\n\n\n\n\n\n\n\n\n\n" );
    data_printf( "--------------------------------------------------------------- \r\n" );
    data_printf( "#####  #####   ####        #    # #  ####  #####   ####  #    # \r\n" );
    data_printf( "#    # #    # #    #       ##  ## # #    # #    # #    # ##   # \r\n" );
    data_printf( "#    # #    # #    # ##### # ## # # #      #    # #    # # #  # \r\n" );
    data_printf( "#####  #####  #    #       #    # # #      #####  #    # #  # # \r\n" );
    data_printf( "#      #   #  #    #       #    # # #    # #   #  #    # #   ## \r\n" );
    data_printf( "#      #    #  ####        #    # #  ####  #    #  ####  #    # \r\n" );
    data_printf( "--------------------------------------------------------------- \r\n" );
    data_printf( "   ?: Show help menu                 \r\n" );
    data_printf( "   x: Show tasks                     \r\n" );
    data_printf( "   c: Config radio transceiver       \r\n" );
    data_printf( "   i: Show receive statistics        \r\n" );
    data_printf( "   r: Reset statistics               \r\n" );
    data_printf( "   s: Streaming mode                 \r\n" );
    data_printf( "      0: Disable                     \r\n" );
    data_printf( "      3: Full data, resulting bending moment \r\n" );
    //data_printf( "Help:    d: Set DAC                         \r\n" );
    //data_printf( "Help:    p: Set PWM frequency               \r\n" );
    data_printf( "   m: moving average size: %d, default: 1  \r\n",cc2500_nmb_1. moving_average_size );
    data_printf( "   z: Zero strain gauges            \r\n" );
    data_printf( "   n: Reset System:                 \r\n" );
    data_printf( "   t: Radio tx                      \r\n" );
    data_printf( "---------------------------------------------------------------\r\n" );
}

void print_transceiver_config( void )
{
    char buffer[10] = { 0 };
    int val1, val2;

    data_printf( "\r\n\n--- AT86RF231 RADIO TRANSCEIVER CONFIG PAGE ---\r\n\n" );
    data_printf( "ESC: Main menue\r\n");
    data_printf( "r: Read all transceiver registers                    \r\n" );
    data_printf( "w: Write one transceiver register                    \r\n" );
    data_printf( "c: Change radio channel (0x%x)                       \r\n", ptr_flash_data->rf231_channel );
    data_printf( "p: Change radio channel page (0x%x)                  \r\n", ptr_flash_data->rf231_channel_page);
    data_printf( "e: Set Transceiver mode = receive                    \r\n" );
    data_printf( "s: Change Source Address  (0x%x)                     \r\n", ptr_flash_data->rf231_src_address );
    data_printf( "d: Change Destination Address  (0x%x)                \r\n", ptr_flash_data->rf231_dest_address );
    data_printf( "p: Change PAN ID (0x%x)                              \r\n", ptr_flash_data->rf231_pan_id );
    data_printf( "t: Change TX power (0x%x)                            \r\n", ptr_flash_data->rf231_tx_pwr );
    data_printf( "\r\nEnter CMD: " );
    usb_uart_get_string( buffer, 10 );

    /* Read all transceiver registers */
    if(buffer[0] == 'r')
    {
        data_printf( "\r\n\r\nADR.     VAL. \r\n" );

        taskENTER_CRITICAL();
        vTaskDelay( 100 );
        for( val1 = 0; val1 <= 0x2F; val1++ )
        {
            vTaskDelay( 1 );
            val2 = pal_trx_reg_read( (uint8_t)val1 );
            data_printf( "0x%x     0x%x \r\n", val1, val2 );
        }
        taskEXIT_CRITICAL();
    }

    /* Write one transceiver register */
    if(buffer[0] == 'w')
    {
        data_printf( "Address [hex]: " );
        usb_uart_get_string( buffer, 10 );
        sscanf( buffer, "%X", &val1 );

        data_printf( "Value[hex]: " );
        usb_uart_get_string( buffer, 10 );
        sscanf( buffer, "%X", &val2 );

        taskENTER_CRITICAL();
        pal_trx_reg_write( (uint8_t)val1, (uint8_t)val2 );
        taskEXIT_CRITICAL();

        data_printf( "Adr: 0x%x     Val: 0x%x \r\n", val1, val2 );
    }

    /* Change radio channel */
    if(buffer[0] == 'c')
    {
        data_printf( "Enter new channel; Store in Userpage: [hex]: " );
        usb_uart_get_string( buffer, 10 );
        sscanf( buffer, "%X", &val1 );

        /* write new channel into transceiver */
        at86rf231_change_channel((uint8_t)val1);

        /* Kanal in User Page speichern */
        ptr_flash_data->rf231_channel = (uint8_t)val1;
        eeprom_save_config();
    }

    /* Change radio channel page */
    if(buffer[0] == 'p')
    {
        data_printf( "0: ALTRATE_250KBPS\r\n2:ALTRATE_500KBPS\r\n16:ALTRATE_1MBPS\r\n17:ALTRATE_2MBPS\r\n : " );
        usb_uart_get_string( buffer, 10 );
        sscanf( buffer, "%d", &val1 );

        /* write new channel page into transceiver */
        apply_channel_page_configuration((uint8_t) val1);

        /* Kanal in User Page speichern */
        ptr_flash_data->rf231_channel_page = (uint8_t)val1;
        eeprom_save_config();
    }

    /* Set Transceiver mode = receive */
    if(buffer[0] == 'e')
    {
        at86rf231_rx_with_auto_ack();
        cc2500_nmb_1.streaming = 1;
    }

    /* Change Source Address */
    if(buffer[0] == 's')
    {
        data_printf( "Store address in Userpage: [hex]: " );
        usb_uart_get_string( buffer, 10 );
        sscanf( buffer, "%X", &val1 );

        /* Kanal in User Page speichern */
        ptr_flash_data->rf231_src_address = (uint16_t)val1;
        eeprom_save_config();
    }

    /* Change Destination Address */
    if(buffer[0] == 'd')
    {
        data_printf( "Store address in Userpage: [hex]: " );
        usb_uart_get_string( buffer, 10 );
        sscanf( buffer, "%X", &val1 );

        /* Kanal in User Page speichern */
        ptr_flash_data->rf231_dest_address = (uint16_t)val1;
        eeprom_save_config();
    }

    /* Change PAN ID */
    if(buffer[0] == 'p')
    {
        data_printf( "Store address in Userpage: [hex]: " );
        usb_uart_get_string( buffer, 10 );
        sscanf( buffer, "%X", &val1 );

        /* Kanal in User Page speichern */
        ptr_flash_data->rf231_pan_id = (uint16_t)val1;
        eeprom_save_config();
    }

    /* Change TX power */
    if(buffer[0] == 't')
    {
        data_printf( "Store pwr value in Userpage: [hex]: " );
        usb_uart_get_string( buffer, 10 );
        sscanf( buffer, "%X", &val1 );

        /* Kanal in User Page speichern */
        ptr_flash_data->rf231_tx_pwr = (uint8_t)val1;
        eeprom_save_config();
    }
}

void print_tx_command( void )
{
    char buffer[10] = { 0 };
    int val1, val2, val3;
	
	data_printf( "\fRadio transmit: \r\n\r\n");
	
    data_printf( "COMMANDS:   PARAM / BYTE 1    2     3      \r\n" );
    data_printf( "0x04: STOP_STREAM        -    -     -      \r\n" );
    data_printf( "0x10: REQ_STREAM         -    -     -      \r\n\n" );

    data_printf( "0x11: READ EEPROM       ADDR  -     -      \r\n" );
    data_printf( "0x12: WRITE EEPROM      ADDR  VAL   -      \r\n" );
    data_printf( "0x13: WRITE_ZMD_CFG     ZMD   PAYLOAD      \r\n" );
    data_printf( "0x14: READ_ZMD_CFG      ZMD   -     -      \r\n" );
    data_printf( "0x15: READ_VERSION       -    -     -      \r\n" );
    data_printf( "0x16: SET_TIMEOUT       TIME1 TIME2 -      \r\n" );
    data_printf( "0x17: SET_SAMPLERATE    RATE1 RATE2 -      \r\n\n" );

    data_printf( "0x20: C_RADIO_CH        CH    -     -      \r\n" );
    data_printf( "0x21: C_DATARATE_MODE   MODE  -     -      \r\n" );
    data_printf( "0x22: NEW_SRC_ADDRESS   ADDR1 ADDR2 -      \r\n" );
    data_printf( "0x23: NEW_DEST_ADDRESS  ADDR1 ADDR2 -      \r\n" );
    data_printf( "0x24: NEW_PANID         PAN1  PAN2  -      \r\n" );
    data_printf( "0x25: NEW_TX_PWR        PWR   -     -      \r\n\n" );

    data_printf( "0x30: READ_TEMPERATURE    -   -     -      \r\n" );
    data_printf( "\r\nEnter CMD: 0x" );
    usb_uart_get_string( buffer, 10 );
    sscanf( buffer, "%X", &val1 );

    data_printf( "\r\nEnter Param1: 0x" );
    usb_uart_get_string( buffer, 10 );
    sscanf( buffer, "%X", &val2 );

    data_printf( "\r\nEnter Param2: 0x" );
    usb_uart_get_string( buffer, 10 );
    sscanf( buffer, "%X", &val3 );

    tx_frame.at86rf231_frame.frame_length = 30;
    tx_frame.at86rf231_frame.frame_control_field = bswap_16(FCF_SET_FRAMETYPE( FCF_FRAMETYPE_MAC_CMD ) |
                                   FCF_SET_DEST_ADDR_MODE( FCF_SHORT_ADDR ) |
                                   FCF_SET_SOURCE_ADDR_MODE( FCF_SHORT_ADDR ) |
                                   FCF_ACK_REQUEST);

    tx_frame.at86rf231_frame.seq_number = 0x77;
    tx_frame.at86rf231_frame.dest_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
    tx_frame.at86rf231_frame.dest_addr = bswap_16(ptr_flash_data->rf231_dest_address);
    tx_frame.at86rf231_frame.src_pan_id = bswap_16(ptr_flash_data->rf231_pan_id);
    tx_frame.at86rf231_frame.src_addr = bswap_16(ptr_flash_data->rf231_src_address);

    tx_frame.at86rf231_frame.payload[0] = val1; // CMD
    tx_frame.at86rf231_frame.payload[1] = val2; // Param1
    tx_frame.at86rf231_frame.payload[2] = val3; // Param2

    tal_state = TAL_TX_AUTO;
    send_frame((uint8_t*)&tx_frame, NO_CSMA_NO_IFS, 1);

    at86rf231_rx_with_auto_ack();
}

