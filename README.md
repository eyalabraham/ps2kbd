# PS2 keyboard interface for PC-XT
This program is the interface code for AVR with a PS2 keyboard. It implements a PS2 keyboard interface and an PC-XT parallel interface. The AVR connects directly into the PC-XT 8255 PPI Port A, bypassing the serial XT keyboard interface. The code configures the keyboard, accepts scan codes, converts the AT scan codes to XT keyboard caode, and transferres the XT code to the PC.
The AVR to PC-XT interface is a parallel interface that uses two handshake signals: a data ready signal into the PC's IRQ1 keyboard interrupt line, and a busy/ready signal from the PC to the AVR.

## Resources:
- [wiki](https://en.wikipedia.org/wiki/PS/2_port)
- [AVR interface to PS2](http://www.electronics-base.com/projects/complete-projects/108-avr-ps2-keyboard-key-readout)
- [PS2 protocol](http://www.burtonsys.com/ps2_chapweske.htm)
- [PS2 keyboard command protocol](https://wiki.osdev.org/PS/2_Keyboard)
- [XT keyboard interface](https://github.com/tmk/tmk_keyboard/wiki/IBM-PC-XT-Keyboard-Protocol)

## PC-XT interface
Connectivity
```
 +-----+               +-----+
 |     |               |     |
 |     +--< PD0..7 ]---+     |
 | PC  |               | AVR |
 |     +---[ PC0 >-----+     +---> PS2 keyboard
 |     |               |     |
 |     +---< PC1 ]-----+     |
 |     |               |     |
 +-----+               +-----+
```
ATmega AVR IO
| Function      | AVR  | Pin         | I/O                   |
|---------------|------|-------------|-----------------------|
| PS2 clock     | PB0  | 14          | in/out w/ pull up     |
| PS2 data      | PB1  | 15          | in/out w/ pull up     |
| PC status     | PC0  | 23          | PC Busy / KDB Enable  |
| PC Interrupt  | PC1  | 24          | PC-XT IRQ1            |
| 8-bit data    | PD   | 2..6,11..13 | 8-bit Scan Code out   |

## Scan code processing
- Only pass make and break codes for keys in range 1 to 83
- Handle 'E0' modifier for keypad by removing the 'E0' which will effectively reduce any keyboard to one that is equivalent to an 83 key keyboard.
- Discard PrtScrn E0,2A,E0,37 and E0,B7,E0,AA; my PC does not support print screen.
- Convert E1 sequence of Pause/Break to scan code 54h/84
