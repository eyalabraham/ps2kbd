/*****************************************************************************
* uart_drv.c
*
* Driver source for AVR UART0 interface
*
* This is driver is interrupt driven.
* All functionality is controlled through passing information to and
* from functions.
*
* Created: January 14, 2015
*
*****************************************************************************/

#include    <stdint.h>
#include    <string.h>
#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/cpufunc.h>

#include    "uart_drv.h"

/****************************************************************************
  UART defines
****************************************************************************/
#define     UART_BUFF_LEN   32  // 32 characters in UART input buffer

/****************************************************************************
  Globals
****************************************************************************/
uint8_t     uart_buffer[UART_BUFF_LEN];
volatile    int inIndex;
volatile    int outIndex;
volatile    int bytesInBuffer;

/* ---------------------------------------------------------------------------
 * uart_initialize()
 *
 * Hard coded UART setup: 8 data bits, 1 stop, no parity, 19,200 BAUD
 *
 */
void uart_initialize()
{
    UCSR0A = _BV(U2X0);                 // double baud rate (sec 19.10 p.195)
    UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);   // enable Tx and Rx
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8 data bits, 1 stop, no parity
    UBRR0 = 51;                         // 19200 baud with the lowest error

    inIndex = 0;
    outIndex = 0;
    bytesInBuffer = 0;
}

/* ---------------------------------------------------------------------------
 * uart_rx_data()
 *
 * attempt to read 'byteCount' data bytes from the UART input buffer.
 * function read as much data as possible and returns number of data bytes
 * actually read.
 *
 */
uint8_t uart_rx_data(uint8_t *data, uint8_t byteCount)
{
    uint8_t rxCount;
    uint8_t i;

    if ( bytesInBuffer == 0 )
        return 0;

    // figure out how many bytes to transfer
    rxCount = (byteCount < bytesInBuffer) ? byteCount : bytesInBuffer;

    // transfer data to caller's buffer
    for ( i = 0; i < rxCount; i++ )
    {
        data[i] = uart_buffer[outIndex++];  // get byte from input buffer to caller's buffer

        if ( outIndex == UART_BUFF_LEN )    // update circular buffer out index
            outIndex = 0;

        bytesInBuffer--;                    // decrement byte count
    }

    return rxCount;
}

/* ---------------------------------------------------------------------------
 * uart_tx_Data()
 *
 * function write 'byteCount' data bytes to the UART transmitter.
 * function blocks until all bytes have been written
 *
 */
void uart_tx_Data(uint8_t *data, uint8_t byteCount)
{
    int i;

    for (i = 0; i < byteCount; i++)
    {
        uart_putchr(data[i]);
    }
}

/* ----------------------------------------------------------------------------
 * uart_putchr()
 *
 * Send character c down the UART Tx, wait until tx holding register is empty
 *
 */
void uart_putchr(uint8_t c)
{
    if ( c == 0 )
        return;
    
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

/* ----------------------------------------------------------------------------
 * uart_getchr()
 *
 * get character from UART buffer
 *
 */
uint8_t uart_getchr(void)
{
    uint8_t byte = 0;

    if ( bytesInBuffer > 0 )
    {
        byte = uart_buffer[outIndex++];     // if buffer has data, get byte from input buffer

        if ( outIndex == UART_BUFF_LEN )    // update circular buffer out index
            outIndex = 0;

        bytesInBuffer--;                    // decrement byte count
    }

    return byte;
}

/* ----------------------------------------------------------------------------
 * uart_ischar()
 *
 * test UART buffer for unread characters, return available characters to read
 *
 */
int uart_ischar(void)
{
    return bytesInBuffer;
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when the UART0 receives a data byte
 * Data is read and stored in a circular buffer.
 * Data can be polled and read from the circular buffer using
 * uart_getchr() or uart_rx_data()
 *
 */
ISR(USART_RX_vect)
{
    uint8_t byte;

    byte = UDR0;    // read byte to remove byte from buffer

    if ( bytesInBuffer < UART_BUFF_LEN )
    {
        uart_buffer[inIndex++] = byte;  // if buffer has free space, store byte in buffer

        if ( inIndex == UART_BUFF_LEN ) // update circular buffer index
            inIndex = 0;

        bytesInBuffer++;                // increment byte count
    }
}
