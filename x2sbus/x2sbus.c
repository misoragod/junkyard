/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   Under Section 7 of GPL version 3, you are granted additional
   permissions described in EXCEPTION.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

extern void _start (void);
static void uart0_handler (void);

static void __attribute__ ((naked))
reset (void)
{
  asm volatile ("cpsid	i\n\t"		/* Mask all interrupts. */
		"ldr	r0, 0f\n\t"	/* Go to entry. */
		"bx	r0\n\t"
		".align	2\n"
	"0:	.word	_start"
		: /* no output */ : /* no input */ : "memory");
  /* Never reach here. */
}

static void
unexpected (void)
{
  for (;;);
}

typedef void (*handler)(void);
extern uint8_t __ram_end__;

#if defined (ENABLE_ISP)
uint32_t crp __attribute__ ((section(".crp"))) = 0;
#else
// Code Red protection: NO_ISP
uint32_t crp __attribute__ ((section(".crp"))) = 0x4E697370;
#endif

handler vector[48] __attribute__ ((section(".vectors"))) = {
  (handler)&__ram_end__,
  reset,
  unexpected,	/*  2: NMI */
  unexpected,	/*  3: Hard fault */
  unexpected, unexpected, unexpected, unexpected, /* 4-7 */
  unexpected, unexpected, unexpected, /* 8-10 */
  unexpected,	/* 11: SVC */
  unexpected, unexpected, /* 12-13 */
  unexpected,	/* 14: Pend SV */
  unexpected,	/* 15: SysTick */
  unexpected,	/* 16: SPI0 */
  unexpected,	/* 17: SPI1 */
  unexpected,	/* 18: Reserve */
  uart0_handler,/* 19: UART0 */
  unexpected,	/* 20: UART1 */
  unexpected,	/* 21: UART2 */
  unexpected,	/* 22: Reserve */
  unexpected,	/* 23: Reserve */
  unexpected,	/* 24: I2C */
  unexpected,	/* 25: SCT */
  unexpected,	/* 26: MRT */
  unexpected,	/* 27: CMP */
  unexpected,	/* 28: WDT */
  unexpected,	/* 29: BOD */
  unexpected,	/* 30: Reserve */
  unexpected,	/* 31: WKT */
  unexpected, unexpected, unexpected, unexpected, /* 32-35 */
  unexpected, unexpected, unexpected, unexpected, /* 36-39 */
  unexpected, unexpected, unexpected, unexpected, /* 40-43 PININT0-3 */
  unexpected, unexpected, unexpected, unexpected, /* 44-47 PININT4-7 */
};

#include "lpc8xx.h"

#define UART0_IRQn (19-16)

#define UART_STAT_RXRDY		(0x1 << 0)
#define UART_STAT_RXIDLE	(0x1 << 1)
#define UART_STAT_TXRDY (0x1 << 2)
#define UART_STAT_TXIDLE (0x1 << 3)

static void main_loop (void);

extern char _edata[], _data[], _textdata[];
extern char _bss_start[], _bss_end[];

void
_start (void)
{
  // Copy .data section from flash.  All are word aligned.  See linker script.
  uint32_t *p = (uint32_t *) _data;
  uint32_t *q = (uint32_t *) _textdata;
  uint32_t size = (uint32_t *) _edata - (uint32_t *) _data;

  while (size--)
    *p++ = *q++;

  // Clear .bss.  Also word aligned.
  p = (uint32_t *) _bss_start;
  size = (uint32_t *) _bss_end - (uint32_t *) _bss_start;
  while (size--)
    *p++ = 0;

  // 30Mhz: main 60Mhz, P=2, M=(4+1), 60*2*2=240Mhz 240/(2*2)/5=12Mz
  LPC_SYSCON->SYSPLLCTRL = 0x24;
  LPC_SYSCON->SYSAHBCLKDIV = 0x02;

  LPC_SYSCON->PDRUNCFG &= ~(0x80);
  while (!(LPC_SYSCON->SYSPLLSTAT & 0x01))
    ;

  LPC_SYSCON->MAINCLKSEL = 0x03;
  LPC_SYSCON->MAINCLKUEN = 0x01;
  while (!(LPC_SYSCON->MAINCLKUEN & 0x01))
    ;

  // 1 wait state for flash
  LPC_FLASHCTRL->FLASHCFG &= ~(0x03);

  // PIO0_1 is an output
  LPC_GPIO_PORT->DIR0 |= 0x02;

  // Enable GPIO(6), UART0 and UART1 clock
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6)|(3 << 14);

  // Reset GPIO(10), UART0 and UART1
  LPC_SYSCON->PRESETCTRL &= ~((1 << 10)|(3 << 3));
  LPC_SYSCON->PRESETCTRL |= ((1 << 10)|(3 << 3));

  // Enable U0-RXD on PIO0_0 PINASSIGN0(15:8)
  LPC_SWM->PINASSIGN0 = 0xffff00ff;
  // Enable U1-TXD on PIO0_4 PINASSIGN1(15:8)
  LPC_SWM->PINASSIGN1 = 0xffff04ff;

  // UART0 baud rate
  LPC_SYSCON->UARTCLKDIV = 1;
  LPC_SYSCON->UARTFRGDIV = 255;
  // UARTFRGMUL 115200->207, 250000->128,  500000->128
  // BRG        115200->17,  250000->9,    500000->24
  // 250000 for XBUS, 100000 for S.BUS
  LPC_SYSCON->UARTFRGMULT = 128;
  LPC_USART0->BRG = 9;
  LPC_USART1->BRG = 24;

  // UART0 enabled, 8 bit, no parity, 1 stop bit, no flow control
  LPC_USART0->CFG = 0x05;
  // UART1 enabled, 8 bit, even parity, 2 stop bit, no flow control
  LPC_USART1->CFG = 0x05|(2 << 4)|(1<< 6);

  // Enable UART0 intr with NVIC
  NVIC_ISER = (1 << UART0_IRQn);

  // Wait 20ms not to get noise when going on hot start
  *SYST_RVR = 6000000-1;
  *SYST_CVR = 0;
  *SYST_CSR = 5;
  while (!(*SYST_CSR & (1 << 16)))
    ;

  main_loop ();
}

#define FIFOSZ 256
static uint8_t rx_fifo[FIFOSZ];
static uint32_t rx_begin;
static uint32_t rx_size;

static void
uart0_handler (void)
{
  if (LPC_USART0->INTSTAT & UART_STAT_RXRDY)
    {
      if (rx_size >= FIFOSZ)
	{
	  // Overrun
	  (void) LPC_USART0->RXDATA;
	}
      else
	{
	  rx_fifo[(rx_begin + rx_size) % FIFOSZ] = LPC_USART0->RXDATA;
	  ++rx_size;
	}
    }
}

static void
send (uint8_t b)
{
  while(~LPC_USART1->STAT & UART_STAT_TXRDY)
    ;
  LPC_USART1->TXDATA = b;
  while(~LPC_USART1->STAT & UART_STAT_TXIDLE)
    ;
}

#define SBUS_NUM_CHANNELS 16
#define SBUS_FRAME_SIZE	25
#define SBUS_FLAGS_BYTE	23
#define SBUS_FAILSAFE_BIT 3
#define SBUS_FRAMELOST_BIT 2

static uint8_t crc8_array[256] =
{
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
  0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
  0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
  0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
  0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
  0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
  0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
  0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
  0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
  0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
  0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
  0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
  0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
  0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
  0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
  0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
  0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
  0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
  0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
  0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
  0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
  0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
  0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
  0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
  0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
  0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
  0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
  0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
  0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
  0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
  0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
  0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

/* S.BUS decoder matrix based on src/modules/px4iofirmware/sbus.c from
   PX4Firmware.

   Each channel value can come from up to 3 input bytes. Each row in the
   matrix describes up to three bytes, and each entry gives:

   - byte offset in the data portion of the frame
   - right shift applied to the data byte
   - mask for the data byte
   - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t lshift;
	uint8_t mask;
	uint8_t rshift;
};

static const struct sbus_bit_pick sbus_encoder[SBUS_NUM_CHANNELS][3] = {
  /*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
  /*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
  /*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
  /*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
  /*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
  /*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
  /*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
  /*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
  /*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
  /*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
  /* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
  /* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
  /* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
  /* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
  /* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
  /* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

static uint8_t frame[SBUS_FRAME_SIZE];

static void
sbus_encode (uint16_t values[])
{
  // Frame boundary marker and clear other bytes
  frame[0] = 0x0f;
  for (int i = 1; i < SBUS_FRAME_SIZE; i++)
    frame[i] = 0;

  // Use the matrix to encode channel data
  for (int ch = 0; ch < SBUS_NUM_CHANNELS; ch++)
    {
      unsigned value = values[ch];
      /* Map XBUS range to SBUS 11-bit range.
	 XBUS: 0 -> 800us, 0xffff -> 2200us
	       0x2492 -> 1000us, 0xdb6d -> 2000us
	 SBUS: 0 -> 1000us, 2048 -> 2000us
      */
      if (value < 0x2492)
	value = 0x2492;
      if (value > 0xdb6d)
	value = 0xdb6d;
      value = ((value - 9362) * 2867) >> 16;

      for (int pick = 0; pick < 3; pick++)
	{
	  const struct sbus_bit_pick *encode = &sbus_encoder[ch][pick];

	  if (encode->mask != 0)
	    {
	      unsigned piece = value;
	      piece >>= encode->rshift;
	      piece &= encode->mask;
	      piece <<= encode->lshift;
	      frame[1+encode->byte] |= piece;
	    }
	}
    }

  for (int i = 0; i < SBUS_FRAME_SIZE; i++)
    send (frame[i]);
}

/* XBUS packet is a byte sequence which looks like:
  command(0xa4), length(2+4*(1-50)), key(0), type(0),
  ch-id(1-50), ch-func(0), ch-data-high, ch-data-low,
  repeated ch-* stuff
  crc8 (x^8+x^5+x^4+1)

  Some XBUS recievers send the packet which has odd key, type
  and ch-func.  If you want to check these bytes, define macros
  XBUS_CHECK_KEY
  XBUS_CHECK_TYPE
  XBUS_CHECK_CHANNEL_FANCTION
  so as to ignore that packet.
*/

// xbus frame input state
enum { WAIT_START, READ_HEADER, READ_CHANNEL };

static uint16_t chdata[SBUS_NUM_CHANNELS];

static void
main_loop (void)
{
  uint8_t c = 0;
  uint8_t chid = 0;
  uint8_t chhigh = 0;
  uint8_t crc = 0;
  int offset = 0;
  int length = 0;
  int state = WAIT_START;
  int n;

  // Enable rxrdy inter only
  LPC_USART0->INTENSET = UART_STAT_RXRDY;

  // Unmask all interrupts
  asm volatile ("cpsie      i" : : : "memory");

  for (;;)
    {
      // Critical section
      asm volatile ("cpsid	i" : : : "memory");
      if (rx_size)
	{
	  c = rx_fifo[rx_begin];
	  rx_begin = (rx_begin + 1) % FIFOSZ;
	  --rx_size;
	  n = 1;
	}
      else
	n = 0;
      asm volatile ("cpsie      i" : : : "memory");

      if (n == 0)
	continue;

      switch (state)
	{
        case WAIT_START:
	  if (c == 0xa4)
	    {
	      ++offset;
	      state = READ_HEADER;
	      for (int i = 0; i < SBUS_NUM_CHANNELS; i++)
		chdata[i] = 0x7fff;
	      crc = 0;
	      crc = crc8_array[(crc ^ c) & 0xff];
            }
	  break;
	case READ_HEADER:
	  if (offset == 1)
	    {
	      // Get length
	      length = c;
	      ++offset;
	      crc = crc8_array[(crc ^ c) & 0xff];
            }
	  else if (offset == 2)
	    {
#if defined(XBUS_CHECK_KEY)
	      // Check if key == 0
	      if (c != 0)
		{
		  state = WAIT_START;
		  offset = 0;
		  continue;
                }
#endif
	      --length;
	      ++offset;
	      crc = crc8_array[(crc ^ c) & 0xff];
            }
	  else if (offset == 3)
	    {
#if defined(XBUS_CHECK_TYPE)
	      // Check if type == 0
	      if (c != 0)
		{
		  state = WAIT_START;
                  offset = 0;
		  continue;
                }
#endif
	      --length;
	      ++offset;
	      crc = crc8_array[(crc ^ c) & 0xff];
	      state = READ_CHANNEL;
            }
	  break;
        case READ_CHANNEL:
	  if (length == 0)
	    {
	      state = WAIT_START;
              offset = 0;
	      // Check crc, send sbus frame if OK
	      if (crc == c)
		sbus_encode (chdata);
	      continue;
            }
	  switch (offset % 4)
	    {
	    case 0:
	      chid = c;
	      break;
	    case 1:
#if defined(XBUS_CHECK_CHANNEL_FANCTION)
	      // Check if channel fuction field is 0
	      if (c != 0)
		{
		  state = WAIT_START;
		  offset = 0;
		  continue;
		}
#endif
	      break;
	    case 2:
	      chhigh = c;
	      break;
	    case 3:
	      if (chid > 0 && chid <= SBUS_NUM_CHANNELS)
		chdata[chid-1] = (chhigh << 8) + c;
	      break;
	    default:
	      break;
	    }
	  --length;
	  ++offset;
	  crc = crc8_array[(crc ^ c) & 0xff];
	  break;
        default:
	  break;
        }
    }
}
