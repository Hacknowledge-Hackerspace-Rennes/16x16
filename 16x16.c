
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LED_PORT_CONFIG DDRB
#define LED_PORT PORTB

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

#define USART_BAUDRATE 230400
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define LINE_LATCH 4
#define LINE_CLOCK 5
#define LINE_DATA  3

#define COL_LATCH 1
#define COL_CLOCK 2
#define COL_DATA  0

#define MAX_SIZE 16

//each element is a line
uint16_t image[MAX_SIZE] =
{
  0b1000000100000001,
  0b0100000100000010,
  0b0010000100000100,
  0b0001000100001000,
  0b0000100100010000,
  0b0000010100100000,
  0b0000001101000000,
  0b0000000110000000,
  0b1111111111111111,
  0b0000001101000000,
  0b0000010100100000,
  0b0000100100010000,
  0b0001000100001000,
  0b0010000100000100,
  0b0100000100000010,
  0b1000000100000001,
};

void serial_init(void)
{
  UBRR1H = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
  UBRR1L = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

  UCSR1B |= (1 << RXEN1) | (1 << TXEN1); // Turn on the transmission and reception circuitry

  UCSR1C |= _BV(UCSZ10) | _BV(UCSZ11); /* 8 bits per char */

  UCSR1B |= (1 << RXCIE1); // Enable the USART Receive Complete interrupt (USART_RXC)

  return;
}

unsigned char i;
unsigned char* pimage = (unsigned char*)image;

ISR(USART1_RX_vect)
{
  pimage[i] = UDR1; // Fetch the received byte value
  //UDR1 = pimage[i]; // Echo back the received byte back to the computer

  i++;
  i%=sizeof(image);
}


void
init()
{
  CPU_PRESCALE(0);
  LED_PORT_CONFIG |= 0xFF;
  LED_PORT = 0x00;
  serial_init();
  sei();
}

inline void
refresh()
{
  unsigned char i, j;
  unsigned char k;

  /*
   * on parcourt les lignes
   */
  for(i=0; i<MAX_SIZE; ++i)
    {
      /*
       * on vide les colonnes
       */
      for(j=0; j<MAX_SIZE; ++j)
        {
          LED_PORT = 0;
          LED_PORT = _BV(COL_CLOCK);
        }
      LED_PORT = _BV(COL_LATCH);

      /*
       * chaque iteration va envoyer un bit dans les deux registres ligne+colonne
       */
      for(j=0; j<MAX_SIZE; ++j)
        {
          k = 0;

          if(j == i)
            {
              /*
               * ligne active (une seule dans la boucle for)
               */
              k |= _BV(LINE_DATA);
            }

          if(image[i] & (1<<j))
            {
              /*
               * colonne active
               */
              k |= _BV(COL_DATA);
            }

          /*
           * on positionne les deux bits à charger
           * les autres bits (dont les clocks) sont à zéro
           */
          LED_PORT = k;
          /*
           * on charge
           */
          LED_PORT = (k | _BV(LINE_CLOCK) | _BV(COL_CLOCK));
        }

      /*
       * on active la sortie des registres
       */
      LED_PORT = (_BV(LINE_LATCH) | _BV(COL_LATCH));
    }
}

int
main()
{
  init();

  while (1)
    {
      refresh();
    }

  return 0;
}
