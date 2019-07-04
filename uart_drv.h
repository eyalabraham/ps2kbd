/*****************************************************************************
* uart_drv.h
*
* Driver header for AVR UART0 interface
*
* This is driver is interrupt driven.
* All functionality is controlled through passing information to and
* from functions.
*
* Created: January 14, 2015
*
*****************************************************************************/

#ifndef __UART_DRV_H__
#define __UART_DRV_H__

/****************************************************************************
  Definitions
****************************************************************************/
#define UART_BUFF_LEN    32 // data byte buffer

/****************************************************************************
  Function prototypes
****************************************************************************/
void uart_initialize();
uint8_t uart_rx_data(uint8_t *, uint8_t);
void uart_tx_Data(uint8_t *, uint8_t);
void uart_putchr(uint8_t);
uint8_t uart_getchr(void);
int uart_ischar(void);

#endif /* __UART_DRV_H__ */
