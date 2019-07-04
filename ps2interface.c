/*
 * ps2interface.c
 *
 *  This program is the interface code for AVR with a PS2 keyboard.
 *  It implements a PS2 keyboard interface and an PC-XT parallel interface.
 *  The AVR connects directly into the PC-XT 8255 PPI Port A, bypassing the serial XT keyboard
 *  interface.
 *
 *  +-----+               +-----+
 *  |     |               |     |
 *  |     +--< PD0..7 ]---+     |
 *  | PC  |               | AVR |
 *  |     +---[ PC0 >-----+     +---> PS2 keyboard
 *  |     |               |     |
 *  |     +---< PC1 ]-----+     |
 *  |     |               |     |
 *  +-----+               +-----+
 *
 *
 * ATmega AVR IO
 *
 * | Function      | AVR  | Pin         | I/O                   |
 * |---------------|------|-------------|-----------------------|
 * | PS2 clock     | PB0  | 14          | in/out w/ pull up     |
 * | PS2 data      | PB1  | 15          | in/out w/ pull up     |
 * | PC status     | PC0  | 23          | PC Busy / KDB Enable  |
 * | PC Interrupt  | PC1  | 24          | PC-XT IRQ1            |
 * | 8-bit data    | PD   | 2..6,11..13 | 8-bit Scan Code out   |
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
 * Port C bit assignment
 *
 *  b7 b6 b5 b4 b3 b2 b1 b0
 *  |  |  |  |  |  |  |  |
 *  |  |  |  |  |  |  |  +--- 'i' PC-XT status input Busy=1 / Ready=0
 *  |  |  |  |  |  |  +------ 'o' PC-XT IRQ1 interrupt request
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
 *  |  |  |  |  |  |  |  +--- 'o' \
 *  |  |  |  |  |  |  +------ 'o' |
 *  |  |  |  |  |  +--------- 'o' |
 *  |  |  |  |  +------------ 'o'  Scan code data output
 *  |  |  |  +--------------- 'o' |
 *  |  |  +------------------ 'o' |
 *  |  +--------------------- 'o' |
 *  +------------------------ 'o'/
 *
 * Note: all references to data sheet are for ATmega 328P Rev. 8161D–AVR–10/09
 *
 * TODO:
 * 1) add watch-dog to reset system if stuck in receive or send due to bad PS2 clocking
 * 2) keyboard error handling and recovery
 *
 */

#include    <stdint.h>
#include    <stdlib.h>

#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/wdt.h>
#include    <util/delay.h>

// IO ports B, C, and D initialization
#define     PB_DDR_INIT     0x00    // port data direction
#define     PB_PUP_INIT     0x03    // port input pin pull-up
#define     PB_INIT         0x00    // port initial values

#define     PC_DDR_INIT     0x02
#define     PC_PUP_INIT     0x00
#define     PC_INIT         0x00

#define     PD_DDR_INIT     0xff
#define     PD_PUP_INIT     0x00
#define     PD_INIT         0x00

// pin change interrupt setting
#define     PCICR_INIT      0x03    // enable pin change sensing on PCINT0..7, PCINT8..14
#define     PCMSK0_INIT     0x01    // enable pin change interrupt on PB0 -> PCINT0
#define     PCMSK1_INIT     0x01    // enable pin change interrupt on PC0 -> PCINT8

// PS2 control line masks
#define     PS2_CLOCK       0x01
#define     PS2_DATA        0x02

// PC control line masks
#define     PC_BUSY         0x01
#define     PC_IRQ          0x02

// buffers
#define     PS2_BUFF_SIZE   32

// host to Keyboard commands
#define     PS2_HK_LEDS     0xED    // Set Status Indicators, next byte LED bitmask
#define     PS2_HK_ECHO     0xEE    // Echo
#define     PS2_HK_INVALID  0xEF    // Invalid Command
#define     PS2_HK_ALTCODE  0xF0    // Select Alternate Scan Codes, next byte Scan code set
#define     PS2_HK_INVALID2 0xF1    // Invalid Command
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

#define     PS2_HK_SCRLOCK  1       // Scroll lock - mask 1 on/0 off
#define     PS2_HK_NUMLOCK  2       // Num lock   - mask 1 on/0 off
#define     PS2_HK_CAPSLOCK 4       // Caps lock  - mask 1 on/0 off

#define     PS2_HK_TYPEMAT  0b01111111  // 1Sec delay, 2Hz repetition

// keyboard to Host commands
#define     PS2_KH_ERR23    0x00    // Key Detection Error/Overrun (Code Sets 2 and 3)
#define     PS2_KH_BATOK    0xAA    // BAT Completion Code
#define     PS2_KH_ERR      0xFC    // BAT Failure Code
#define     PS2_KH_ECHO     0xEE    // Echo
#define     PS2_KH_BREAK    0xF0    // Break (key-up)
#define     PS2_KH_ACK      0xFA    // Acknowledge (ACK)
#define     PS2_KH_RESEND   0xFE    // Resend
#define     PS2_KH_ERR1     0xFF    // Key Detection Error/Overrun (Code Set 1)

#define     PS2_SCAN_CAPS   0x3a    // Caps lock scan code
#define     PS2_SCAN_SCROLL 0x46    // Scroll lock scan code
#define     PS2_SCAN_NUM    0x45    // Num lock scan code
#define     PS2_SCAN_BREAK  0x54    // Fake scan code for Brea/Pause key

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


typedef enum
{
    PC_KBD_BUSY,
    PC_KBD_READY        // only when PC1 (PC IRQ1)='0' and PC0 (PC ready.busy)='1'
} pc_state_t;

/* | PC0 | PC1 |   pc_state_t   |                       state                   |
 * |-----|-----|----------------|-----------------------------------------------|
 * |  0  |  0  | PC_KBD_READY   | Starting state, PC is ready and IRQ not set   |
 * |  0  |  1  | PC_KBD_BUSY    | AVR placed a scan code and asserted IRQ       |
 * |  1  |  1  | PC_KBD_BUSY    | AVR's IRQ was acknowledged                    |
 * |  1  |  0  | PC_KBD_BUSY    | AVR's IRQ cleared, PC processing scan code    |
 * |  0  |  0  | PC_KBD_READY   | Starting state, PC completed code processing  |
 *
 */
/****************************************************************************
  Function prototypes
****************************************************************************/
void    reset(void) __attribute__((naked)) __attribute__((section(".init3")));
void    ioinit(void);

int     ps2_send(uint8_t);
int     ps2_recv_x(void);       // Blocking
int     ps2_recv(void);         // Non-blocking

void    kbd_test_led(void);
int     kdb_led_ctrl(uint8_t);
int     kbd_code_set(int);
int     kbd_typematic_set(uint8_t);

int     pc_is_kbd_ready();
void    pc_set_irq(void);
void    pc_clear_irq(void);

/****************************************************************************
  Globals
****************************************************************************/
// circular buffer holding PS2 scan codes
uint8_t      ps2_scan_codes[PS2_BUFF_SIZE];
int          ps2_buffer_out = 0;
volatile int ps2_buffer_in = 0;
volatile int ps2_scan_code_count = 0;

// variable maintaining state of bit stream from PS2
volatile ps2_state_t ps2_rx_state = PS2_IDLE;
volatile uint8_t  ps2_rx_data_byte = 0;
volatile int      ps2_rx_bit_count = 0;
volatile int      ps2_rx_parity = 0;

// PC status
volatile pc_state_t pc_state = PC_KBD_READY;

// keyboard status
volatile uint8_t    kbd_lock_keys = 0;;

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 */
int main(void)
{
    int     scan_code;
    uint8_t kdb_lock_state = 0;

    // Initialize IO devices
    ioinit();

    // Wait enough time for keyboard to complete self test
    _delay_ms(2000);

    // Wait for PC to be ready and then enable interrupts
    do {} while ( PINC & PC_BUSY );
    sei();

    /* Start watch-dog timer here after PC is done with RAM test, and keyboard is enabled.
     * Any reset, power-on or watch-dog, will disable the watch-dog, so the enable at this point will
     * not cause a repeat reset by the watch-dog while RAM test is running on the PC.
     * This time out allows the keyboard interrupt to run up to 500mSec on the PC (BUSY line asserted),
     * or provides the PC a way to reset the keyboard by asserting the BUSY line for more than 500mSec.
     */
    //wdt_enable (WDTO_500MS);

    // light LEDs in succession
    kbd_test_led();

    // set typematic delay and rate
    kbd_typematic_set(PS2_HK_TYPEMAT);

    // change code set to "1" so code set translation does not needs to take place on the AVR
    kbd_code_set(1);

    // initialize the system with a fake NumLock on code to the PC
/*
    ps2_scan_codes[0] = PS2_SCAN_NUM;
    ps2_buffer_in = 1;
    ps2_scan_code_count = 1;
    kbd_lock_keys = PS2_HK_NUMLOCK;
*/

    // loop forever
    while ( 1 )
    {
        if ( pc_is_kbd_ready() )
        {
            /* Reset the watch-dog here because we have a READY state from the PC,
             * if PC is not ready for too long, this watch-dog reset will not happen
             * and AVR will reset; such as the case when PC is rebooted, hung, or trying
             * to reset the keyboard by asserting the BUSY line.
             */
            //wdt_reset();

            scan_code = ps2_recv();

            /* Only pass make and break codes for keys in range 1 to 83
             * Handle 'E0' modifier for keypad by removing the 'E0' which
             * will effectively reduce any keyboard to one that is equivalent to an 83 key keyboard.
             * Discard PrtScrn E0,2A,E0,37 and E0,B7,E0,AA; my PC does not support print screen.
             * Convert E1 sequence of Pause/Break to scan code 54h/84
             */
            if  ( scan_code != -1 )
            {
                // Handle 'E1' scan code case for Pause/Break key
                // Translate sequence E1,1D,45,E1,9D,C5 to code 0x54
                if ( scan_code == 0xe1 )
                {
                    // Get the next byte
                    scan_code = ps2_recv_x();

                    // If it is a Pause/Break sequence then discard it (in two passes)
                    if ( scan_code == 0x1d )
                    {
                        ps2_recv_x();
                        continue;
                    }
                    else if ( scan_code == 0x9d )
                    {
                        ps2_recv_x();
                        scan_code = PS2_SCAN_BREAK;
                    }

                }

                // Handle 'E0' scan code cases
                if ( scan_code == 0xe0 )
                {
                    // Get the next byte
                    scan_code = ps2_recv_x();

                    // Discard only unwanted sequence pairs
                    if ( scan_code == 0x2a ||
                         scan_code == 0xaa ||
                         scan_code == 0xb7 ||
                         scan_code == 0x37     )
                    {
                        continue;
                    }
                }

                if  ( ((uint8_t)scan_code & 0x7f) > PS2_SCAN_BREAK || scan_code == 0 )
                {
                    continue;
                }

                // Send code to PC
                PORTD = (uint8_t)scan_code;
                pc_set_irq();

                // Toggle keyboard lock status
                if ( scan_code == PS2_SCAN_SCROLL )
                    kbd_lock_keys ^= PS2_HK_SCRLOCK;
                else if ( scan_code == PS2_SCAN_CAPS )
                    kbd_lock_keys ^= PS2_HK_CAPSLOCK;
                else if ( scan_code == PS2_SCAN_NUM )
                    kbd_lock_keys ^= PS2_HK_NUMLOCK;
            }

            /* Update indicator LEDs if required
             * do this only if there is no pending scan code in the buffer
             * so that host-to-keyboard comm does not interfere with scan code exchange
             */
            else if ( kdb_lock_state != kbd_lock_keys )
            {
                kdb_led_ctrl(kbd_lock_keys);
                kdb_lock_state = kbd_lock_keys;
            }
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

    DDRC  = PC_DDR_INIT;
    PORTC = PC_INIT | PC_PUP_INIT;

    DDRD  = PD_DDR_INIT;
    PORTD = PD_INIT | PD_PUP_INIT;

    // pin change interrupt setting
    PCICR = PCICR_INIT;
    PCMSK0 = PCMSK0_INIT;
    PCMSK1 = PCMSK1_INIT;
}

/* ----------------------------------------------------------------------------
 * ps2_send_cmd()
 *
 *  Send a command to the PS2 keyboard
 *  1)   Bring the Clock line low for at least 100 microseconds.
 *  2)   Bring the Data line low.
 *  3)   Release the Clock line.
 *
 *  4)   Set/reset the Data line to send the first data bit
 *  5)   Wait for the device to bring Clock low.
 *  6)   Repeat steps 5-7 for the other seven data bits and the parity bit
 *  7)   Wait for the device to bring Clock high.
 *
 *  8)   Release the Data line.
 *  9)   Wait for the device to bring Data low.
 *  10)  Wait for the device to bring Clock  low.
 *  11)  Wait for the device to release Data and Clock
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

    // allow keyboard to recover before exiting,
    // so that another ps2_send() is spaced in time from this call.
    _delay_ms(20);

    return result;
}

/* ----------------------------------------------------------------------------
 * ps2_recv_x()
 *
 *  Get a byte from the PS2 input buffer and block until byte is available
 *
 *  param:  none
 *  return: data byte value
 *
 */
int ps2_recv_x(void)
{
    int     result;

    do
    {
        result = ps2_recv();
    } while ( result == -1 );

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

    if ( ps2_scan_code_count > 0 )
    {
        result = (int)ps2_scan_codes[ps2_buffer_out];
        ps2_scan_code_count--;
        ps2_buffer_out++;
        if ( ps2_buffer_out == PS2_BUFF_SIZE )
            ps2_buffer_out = 0;
    }

    return result;
}

/* ----------------------------------------------------------------------------
 * ps2_test_led()
 *
 *  Simple light test for keyboard LEDs.
 *  The function discards the keyboard's response;
 *  if something is wrong LEDs want light up in succession.
 *
 *  param:  none
 *  return: none
 */
void kbd_test_led(void)
{
    kdb_led_ctrl(PS2_HK_SCRLOCK);

    _delay_ms(1000);

    kdb_led_ctrl(0);
    kdb_led_ctrl(PS2_HK_CAPSLOCK);

    _delay_ms(1000);

    kdb_led_ctrl(0);
    kdb_led_ctrl(PS2_HK_NUMLOCK);

    _delay_ms(1000);

    kdb_led_ctrl(0);
}

/* ----------------------------------------------------------------------------
 * kdb_led_ctrl()
 *
 *  Function for setting LED state to 'on' or 'off'
 *
 *  param:  LED bits, b0=Scroll lock b1=Num lock b2=Caps Lock
 *  return: none
 */
int kdb_led_ctrl(uint8_t state)
{
    int temp_scan_code;

    state &= 0x07;

    ps2_send(PS2_HK_LEDS);
    temp_scan_code = ps2_recv_x();

    if ( temp_scan_code == PS2_KH_ACK )
    {
        ps2_send(state);
        temp_scan_code = ps2_recv_x();
    }

    return temp_scan_code;
}

/* ----------------------------------------------------------------------------
 * kbd_code_set()
 *
 *  The function requests the keyboard to change its scan code set,
 *  Legal values are 1, 2 or 3.
 *
 *  param:  scan code set identifier
 *  return: PS2_KH_ACK no errors, other keyboard response if error
 */
int kbd_code_set(int set)
{
    int temp_scan_code;

    if ( set < 1 || set > 3 )
        return PS2_KH_RESEND;

    ps2_send(PS2_HK_ALTCODE);
    temp_scan_code = ps2_recv_x();

    if ( temp_scan_code == PS2_KH_ACK )
    {
        ps2_send((uint8_t)set);
        temp_scan_code = ps2_recv_x();
    }

    return temp_scan_code;
}

/* ----------------------------------------------------------------------------
 * kbd_typematic_set()
 *
 *  The function sets the keyboard typematic rate and delay.
 *
 *  Bit/s   Meaning
 *  ....... ...................................................
 *  0 to 4  Repeat rate (00000b = 30 Hz, ..., 11111b = 2 Hz)
 *  5 to 6  Delay before keys repeat (00b = 250 ms, 01b = 500 ms, 10b = 750 ms, 11b = 1000 ms)
 *     7    Must be zero
 *
 *  param:  typematic rate and delay
 *  return: PS2_KH_ACK no errors, other keyboard response if error
 */
int kbd_typematic_set(uint8_t configuration)
{
    int temp_scan_code;

    configuration &= 0x7f;

    ps2_send(PS2_HK_TMDELAY);
    temp_scan_code = ps2_recv_x();

    if ( temp_scan_code == PS2_KH_ACK )
    {
        ps2_send(configuration);
        temp_scan_code = ps2_recv_x();
    }

    return temp_scan_code;
}
/* ----------------------------------------------------------------------------
 * pc_is_kbd_ready()
 *
 *  Logic for determining if PC keyboard is ready to accept scan code.
 *
 *  param:  none
 *  return: '0'=not ready, '-1'=ready
 *
 */
int pc_is_kbd_ready(void)
{
    return ((pc_state == PC_KBD_BUSY) ? 0 : -1);
}
/* ----------------------------------------------------------------------------
 * pc_set_irq()
 *
 *  Assert (set) the PC's IRQ1 line.
 *
 *  param:  none
 *  return: none
 *
 */
void pc_set_irq(void)
{
    // assert the interrupt line
    PORTC |= PC_IRQ;

    // mark PC as busy until the PC acknowledges the interrupt
    pc_state = PC_KBD_BUSY;
}

/* ----------------------------------------------------------------------------
 * pc_clear_irq()
 *
 *  Clear the PC's IRQ1 line.
 *
 *  param:  none
 *  return: none
 *
 */
void pc_clear_irq(void)
{
    // clear the interrupt line
    PORTC &= ~PC_IRQ;
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
                    ps2_rx_state = PS2_STOP;
                else
                    ps2_rx_state = PS2_RX_ERR_PARITY;
                break;

            /* check for valid stop bit
             */
            case PS2_STOP:
                if ( ps2_data_bit == 1 )
                {
                    if ( ps2_scan_code_count < PS2_BUFF_SIZE )
                    {
                        ps2_scan_codes[ps2_buffer_in] = ps2_rx_data_byte;
                        ps2_scan_code_count++;
                        ps2_buffer_in++;
                        if ( ps2_buffer_in == PS2_BUFF_SIZE )
                            ps2_buffer_in = 0;
                        ps2_rx_state = PS2_IDLE;
                    }
                    else
                        ps2_rx_state = PS2_RX_ERR_OVERRUN;
                }
                else
                    ps2_rx_state = PS2_RX_ERR_STOP;
                break;
        }
    }
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when PC0 changes state.
 * PC0 is connected to the PC-XT busy/ready output, and signals when the PC
 * is busy (0) processing a scan code, and when it is ready (1) to accept a new code.
 *
 */
ISR(PCINT1_vect)
{
    if ( PINC & PC_BUSY )
    {
        // this might be redundant, but safe
        pc_state = PC_KBD_BUSY;
        // PC acknowledged the interrupt, but still processing the scan code
        // we can clear the request
        pc_clear_irq();
    }
    else
    {
        // PC acknowledged the interrupt and finished processing the scan code
        pc_state = PC_KBD_READY;
    }
}
