
#include    <stdlib.h>
#include    <stdio.h>
#include    <stdarg.h>

#include    <avr/pgmspace.h>

#include    "uart_drv.h"
#include    "utils.h"

#define     PRINT_BUFFER    64      // output print buffer

/* ----------------------------------------------------------------------------
 * printstr()
 *
 * Send a NULL-terminated string down the UART Tx
 *
 */
void printstr(const char *s)
{

  while (*s)
    {
      if (*s == '\n')
          uart_putchr('\r');

      uart_putchr(*s++);
    }
}

/* ----------------------------------------------------------------------------
 * printstr_p()
 *
 * Same as printstr(), but the string is located in program memory,
 * so "lpm" instructions are needed to fetch it.
 *
 */
void printstr_p(const char *s)
{
  char c;

  for (c = pgm_read_byte(s); c; ++s, c = pgm_read_byte(s))
    {
      if (c == '\n')
          uart_putchr('\r');

      uart_putchr(c);
    }
}

/* ----------------------------------------------------------------------------
 * vprintfunc()
 *
 * print a formatter string
 *
 */
int vprintfunc(char *format, ...)
{
    char    text[PRINT_BUFFER] = {0};             // text buffer for printing messages
    va_list aptr;
    int     ret;

    va_start(aptr, format);
    ret = vsnprintf(text, PRINT_BUFFER, format, aptr);
    va_end(aptr);

    printstr(text);

    return(ret);
}

/* ----------------------------------------------------------------------------
 * printfloat()
 *
 * print a floating point number
 *
 */
void printfloat(float val)
{
    int     d1, d2;
    float   f2;
    char    sig;

    d1 = (int) val;
    f2 = val - d1;
    d2 = (int) (f2 * 1000.0);
    sig = (val < 0) ? '-' : '+';
    d1 = abs(d1);
    d2 = abs(d2);
    vprintfunc("%c%d.%03d", sig, d1, d2);
}
