/* 
 * ps2test.c
 *
 *  This program is a test program for interfacing AVR with a PS2 keyboard
 *  It implements the PS2 half of a PS2 to XT keyboard converter.
 *  The test program controls the keyboard, receives scan codes and print them
 *  through the UART to a serial console.
 * 
 *  +-----+             +-----+
 *  |     |             |     |
 *  | PC  |             | AVR |
 *  |     +--< UART >---+     +---> PS2 keyboard
 *  |     |             |     |
 *  |     |             |     |
 *  |     |             |     |
 *  +-----+             +-----+
 *               
 *
 * ATmega AVR IO
 *
 * | Function      | AVR  | Pin  | I/O               |
 * |---------------|------|------|-------------------|
 * | UART transmit | TxD  | 3    | out               |
 * | PS2 clock     | PB0  | 14   | in/out w/ pull up |
 * | PS2 data      | PB1  | 15   | in/out w/ pull up |
 *
 * Port B bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *  |  |  |  |  |  |  |  |
 *  |  |  |  |  |  |  |  +--- 'i' PS2 clock
 *  |  |  |  |  |  |  +------ 'i' PS2 data
 *  |  |  |  |  |  +--------- 'i'
 *  |  |  |  |  +------------ 'i'
 *  |  |  |  +--------------- 'i'
 *  |  |  +------------------ 'i'
 *  |  +--------------------- 'i'
 *  +------------------------ 'i'
 *
 * Port D bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *  |  |  |  |  |  |  |  |
 *  |  |  |  |  |  |  |  +--- 'i'
 *  |  |  |  |  |  |  +------ 'o' UART Tx
 *  |  |  |  |  |  +--------- 'i'
 *  |  |  |  |  +------------ 'i'
 *  |  |  |  +--------------- 'i'
 *  |  |  +------------------ 'i'
 *  |  +--------------------- 'i'
 *  +------------------------ 'i'
 *
 * Note: all references to data sheet are for ATmega 328P Rev. 8161D–AVR–10/09
 *
 * TODO:
 * 1) add watch-dog to reset system if stuck in receive or send due to bad PS2 clocking
 * 2) parallel or XT serial interface to PC-XT
 *
 */

#include    <stdint.h>
#include    <stdlib.h>
#include    <string.h>
#include    <stdio.h>
#include    <stdarg.h>
#include    <math.h>

#include    <avr/pgmspace.h>
#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/wdt.h>
#include    <util/delay.h>

#include    "uart_drv.h"
#include    "utils.h"

// debug print to UART port definition
// UART is assumed to be defined and initialized
//#define     __DEBUG_PRINT__

// IO ports B and D initialization
#define     PB_DDR_INIT     0x00    // port data direction
#define     PB_PUP_INIT     0x03    // port input pin pull-up
#define     PB_INIT         0x00    // port initial values

#define     PD_DDR_INIT     0x02    // port data direction
#define     PD_PUP_INIT     0x00    // port input pin pull-up
#define     PD_INIT         0x00    // port initial values

// pin change interrupt setting
#define     PCICR_INIT      0x01    // enable pin change sensing on PCINT0..7
#define     PCMSK0_INIT     0x01    // enable pin change interrupt on PB0

// PS2 control line masks
#define     PS2_CLOCK       0x01
#define     PS2_DATA        0x02

// buffers
#define     PS2_BUFF_SIZE   10

// host to Keyboard commands
#define     PS2_HK_LEDS     0xED    // Set Status Indicators, next byte LED bitmask
#define     PS2_HK_ECHO     0xEE    // Echo
#define     PS2_HK_INVALID  0xEF    // Invalid Command
#define     PS2_HK_ALTCODE  0xF0    // Select Alternate Scan Codes, next byte Scan code set
#define     PS2_HK_INVALID2 0xF1    // Invalid Command
#define     PS2_HK_ID       0xF2    // Read ID
#define     PS2_HK_TMDELAY  0xF3    // Set Typematic Rate/Delay, next byte Encoded rate/delay
#define     PS2_HK_ENABLE   0xF4    // Enable
#define     PS2_HK_DISABLE  0xF5    // Default Disable
#define     PS2_HK_DEFAULT  0xF6    // Set Default
#define     PS2_HK_SET1     0xF7    // Set All Keys - Typematic
#define     PS2_HK_SET2     0xF8    // Set All Keys - Make/Break
#define     PS2_HK_SET3     0xF8    // Set All Keys - Make
#define     PS2_HK_SET4     0xFA    // Set All Keys - Typematic/Make/Break
#define     PS2_HK_SET5     0xFB    // Set All Key Type - Typematic, next byte Scan code
#define     PS2_HK_SET6     0xFC    // Set All Key Type - Make/Break, next byte Scan code
#define     PS2_HK_SET7     0xFD    // Set All Key Type - Make, next byte Scan code
#define     PS2_HK_RESEND   0xFE    // Resend
#define     PS2_HK_RESET    0xFF    // Reset

#define     PS2_HK_SCRLOCK  1       // Scrollock - mask 1 on/0 off
#define     PS2_HK_NUMLOCK  2       // Numlock   - mask 1 on/0 off
#define     PS2_HK_CAPSLOCK 4       // Capslock  - mask 1 on/0 off

// keyboard to Host commands
#define     PS2_KH_ERR23    0x00    // Key Detection Error/Overrun (Code Sets 2 and 3)
#define     PS2_KH_ID       0x83 0xAB   // Keyboard ID
#define     PS2_KH_BATOK    0xAA    // BAT Completion Code
#define     PS2_KH_ERR      0xFC    // BAT Failure Code
#define     PS2_KH_ECHO     0xEE    // Echo
#define     PS2_KH_BREAK    0xF0    // Break (key-up)
#define     PS2_KH_ACK      0xFA    // Acknowledge (ACK)
#define     PS2_KH_RESEND   0xFE    // Resend
#define     PS2_KH_ERR1     0xFF    // Key Detection Error/Overrun (Code Set 1)

/****************************************************************************
  Types
****************************************************************************/
typedef enum
        {
            PS2_IDLE,
            PS2_DATA_BITS,
            PS2_PARITY,
            PS2_STOP,
            PS2_RX_ERR_START,
            PS2_RX_ERR_OVERRUN,
            PS2_RX_ERR_PARITY,
            PS2_RX_ERR_STOP
        } ps2_state_t;

/****************************************************************************
  Function prototypes
****************************************************************************/
// This function is called upon a HARDWARE RESET:
void    reset(void) __attribute__((naked)) __attribute__((section(".init3")));
void    ioinit(void);
int     ps2_send(uint8_t);
int     ps2_recv(void);

/****************************************************************************
  Globals
****************************************************************************/
// circular buffer holding PS2 scan codes
uint8_t     ps2_scan_codes[PS2_BUFF_SIZE];
int         ps2_buffer_in = 0;
int         ps2_buffer_out = 0;
int         ps2_scan_code_count = 0;

// variable maintaining state of bit stream from PS2
volatile ps2_state_t ps2_rx_state = PS2_IDLE;
volatile uint8_t  ps2_rx_data_byte = 0;
volatile int      ps2_rx_bit_count = 0;
volatile int      ps2_rx_parity = 0;


/* ----------------------------------------------------------------------------
 * main() control functions
 *
 */
int main(void)
{
    int     temp_scan_code;

    // initialize IO devices
    ioinit();

    // clear console and output message
    printstr_p(PSTR("ps2test.c\n"));
    vprintfunc("build %s %s\n", __DATE__, __TIME__);

    // enable interrupts
    sei();

    _delay_ms(500);

/*
    printstr_p(PSTR("sending RESET\n"));
    ps2_send(PS2_HK_RESET);
    do {} while ( (temp_scan_code = ps2_recv()) == -1 );
    vprintfunc("response 0x%02x\n", temp_scan_code);

    printstr_p(PSTR("sending ENABLE\n"));
    ps2_send(PS2_HK_ENABLE);
    do {} while ( (temp_scan_code = ps2_recv()) == -1 );
    vprintfunc("response 0x%02x\n", temp_scan_code);
*/

    vprintfunc("LEDs cmd (%d)\n", ps2_send(PS2_HK_LEDS));
    do {} while ( (temp_scan_code = ps2_recv()) == -1 );
    vprintfunc("response 0x%02x (state=%d)\n", temp_scan_code, ps2_rx_state);

    vprintfunc("set LEDs (%d)\n", ps2_send(3));
    do {} while ( (temp_scan_code = ps2_recv()) == -1 );
    vprintfunc("response 0x%02x (state=%d)\n", temp_scan_code, ps2_rx_state);

    // loop forever
    while ( 1 )
    {
        if  ( (temp_scan_code = ps2_recv()) != -1 )
        {
            vprintfunc("scan code %3d / 0x%02x (state=%d)\n", temp_scan_code, temp_scan_code, ps2_rx_state);
        }
    }

    return 0;
}

/* ----------------------------------------------------------------------------
 * reset()
 *
 *  Clear SREG_I on hardware reset.
 *  source: http://electronics.stackexchange.com/questions/117288/watchdog-timer-issue-avr-atmega324pa
 */
void reset(void)
{
     cli();
    // Note that for newer devices (any AVR that has the option to also
    // generate WDT interrupts), the watchdog timer remains active even
    // after a system reset (except a power-on condition), using the fastest
    // prescaler value (approximately 15 ms). It is therefore required
    // to turn off the watchdog early during program startup.
    MCUSR = 0; // clear reset flags
    wdt_disable();
}

/* ----------------------------------------------------------------------------
 * ioinit()
 *
 *  initialize IO interfaces
 *  timer and data rates calculated based on 4MHz internal clock
 *
 * initialize IO: UART, timer and IO pins
 * - timer0 provides 2 PWM signals for gripper servo control
 * - timer1 provides time base for stepper motor movement/synchronization
 * - IO pins to control 3 L293D H-bridge for 3 bipolar stepper motors, limit switches etc.
 * - UART to send/receive commands from host
 *
 */
void ioinit(void)
{
    // Reconfigure system clock scaler to 8MHz
    CLKPR = 0x80;   // change clock scaler (sec 8.12.2 p.37)
    CLKPR = 0x00;

    // initialize general IO PB and PD pins
    DDRB  = ~PB_DDR_INIT;
    PORTB = PB_INIT;
    DDRB  = PB_DDR_INIT;
    PORTB = PB_INIT | PB_PUP_INIT;

    DDRD  = PD_DDR_INIT;
    PORTD = PD_INIT | PD_PUP_INIT;

    // pin change interrupt setting
    PCICR = PCICR_INIT;
    PCMSK0 = PCMSK0_INIT;

    // initialize UART interface to 19200 BAUD, 8 bit, 1 stop, no parity
    uart_initialize();
}

/* ----------------------------------------------------------------------------
 * ps2_send_cmd()
 *
 *  Send a command to the PS2 keyboard
 *  1)   Bring the Clock line low for at least 100 microseconds.
 *  2)   Bring the Data line low.
 *  3)   Release the Clock line.
 *  4)   Wait for the device to bring the Clock line low.
 *
 *  5)   Set/reset the Data line to send the first data bit
 *  6)   Wait for the device to bring Clock high.
 *  7)   Wait for the device to bring Clock low.
 *  8)   Repeat steps 5-7 for the other seven data bits and the parity bit
 *
 *  9)   Release the Data line.
 *  10)  Wait for the device to bring Data low.
 *  11)  Wait for the device to bring Clock  low.
 *  12)  Wait for the device to release Data and Clock
 *
 *  param: command byte
 *  return: -1 transmit error, 0 ok
 */
int ps2_send(uint8_t byte)
{
    int     ps2_tx_parity = 1;
    int     ps2_tx_bit_count = 0;
    uint8_t ps2_data_bit;
    int     result;

    // disable interrupts and reset receiver state so receive ISR does not run
    cli();
    ps2_rx_state = PS2_IDLE;
    ps2_rx_data_byte = 0;
    ps2_rx_bit_count = 0;
    ps2_rx_parity = 0;

    // follow byte send steps
    DDRB |= PS2_CLOCK;
    PORTB &= ~PS2_CLOCK;
    _delay_us(100);

    DDRB |= PS2_DATA;
    PORTB &= ~PS2_DATA;

    DDRB &= ~PS2_CLOCK;
    PORTB |= PS2_CLOCK;

    while ( ps2_tx_bit_count < 10 )
    {
        // this will repeat 8 bits of data, one parity bit, and one stop bit for transmission
        if ( ps2_tx_bit_count < 8 )
        {
            ps2_data_bit = byte & 0x01;
            ps2_tx_parity += ps2_data_bit;
        }
        else if ( ps2_tx_bit_count == 8 )
        {
            ps2_data_bit = (uint8_t)ps2_tx_parity & 0x01;
        }
        else
        {
            ps2_data_bit = 1;
        }

        do {} while ( (PINB & PS2_CLOCK) );

        if ( ps2_data_bit )
            PORTB |= PS2_DATA;
        else
            PORTB &= ~PS2_DATA;

        do {} while ( !(PINB & PS2_CLOCK) );

        ps2_tx_bit_count++;
        byte = byte >> 1;
    }

    // restore data line to receive mode
    DDRB &= ~PS2_DATA;
    PORTB |= PS2_DATA;

    // check here for ACK pulse and line to idle
    do {} while ( (PINB & PS2_CLOCK) );
    result = -1 * (int)(PINB & PS2_DATA);

    // wait for clock to go high before enabling interrupts
    do {} while ( !(PINB & PS2_CLOCK) );

    sei();

    return result;
}

/* ----------------------------------------------------------------------------
 * ps2_recv()
 *
 *  Get a byte from the PS2 input buffer
 *
 *  param:  none
 *  return: -1 if buffer empty, otherwise data byte value
 *
 */
int ps2_recv(void)
{
    int     result = -1;

    if ( ps2_scan_code_count )
    {
        result = ps2_scan_codes[ps2_buffer_out];
        ps2_scan_code_count--;
        ps2_buffer_out++;
        if ( ps2_buffer_out == PS2_BUFF_SIZE )
            ps2_buffer_out = 0;
    }

    return result;
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when PB0 changes state.
 * PB0 is connected to the PS2 keyboard clock line, and PB1 to the data line.
 * ISR will check PB0 state and determine if it is '0' or '1',
 * as well as track clock counts and input bits from PB1.
 * Once input byte is assembled is will be added to a circular buffer.
 *
 */
ISR(PCINT0_vect)
{
    uint8_t         ps2_data_bit;

    if ( (PINB & PS2_CLOCK) == 0 )
    {
        ps2_data_bit = (PINB & PS2_DATA) >> 1;

        switch ( ps2_rx_state )
        {
            /* do nothing if an error was already signaled
             * let the main loop handle the error
             */
            case PS2_RX_ERR_START:
            case PS2_RX_ERR_OVERRUN:
            case PS2_RX_ERR_PARITY:
            case PS2_RX_ERR_STOP:
                break;

            /* if in idle, then check for valid start bit
             */
            case PS2_IDLE:
                if ( ps2_data_bit == 0 )
                {
                    ps2_rx_data_byte = 0;
                    ps2_rx_bit_count = 0;
                    ps2_rx_parity = 0;
                    ps2_rx_state = PS2_DATA_BITS;
                }
                else
                    ps2_rx_state = PS2_RX_ERR_START;
                break;

            /* accumulate eight bits of data LSB first
             */
            case PS2_DATA_BITS:
                ps2_rx_parity += ps2_data_bit;
                ps2_data_bit = ps2_data_bit << ps2_rx_bit_count;
                ps2_rx_data_byte += ps2_data_bit;
                ps2_rx_bit_count++;
                if ( ps2_rx_bit_count == 8 )
                    ps2_rx_state = PS2_PARITY;
                break;

            /* evaluate the parity and signal error if it is wrong
             */
            case PS2_PARITY:
                if ( ((ps2_rx_parity + ps2_data_bit) & 1) )
                {
                    if ( ps2_scan_code_count < PS2_BUFF_SIZE )
                    {
                        ps2_scan_codes[ps2_buffer_in] = ps2_rx_data_byte;
                        ps2_scan_code_count++;
                        ps2_buffer_in++;
                        if ( ps2_buffer_in == PS2_BUFF_SIZE )
                            ps2_buffer_in = 0;
                        ps2_rx_state = PS2_STOP;
                    }
                    else
                        ps2_rx_state = PS2_RX_ERR_OVERRUN;
                }
                else
                    ps2_rx_state = PS2_RX_ERR_PARITY;
                break;

            /* check for valid stop bit
             */
            case PS2_STOP:
                if ( ps2_data_bit == 1 )
                    ps2_rx_state = PS2_IDLE;
                else
                    ps2_rx_state = PS2_RX_ERR_STOP;
                break;
        }
    }
}
