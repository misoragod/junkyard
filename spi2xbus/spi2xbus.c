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
static void spi0_handler (void);

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
  spi0_handler,	/* 16: SPI0 */
  unexpected,	/* 17: SPI1 */
  unexpected,	/* 18: Reserve */
  uart0_handler,	/* 19: UART0 */
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

#define SPI0_IRQn (16-16)
#define UART0_IRQn (19-16)

#define UART_STAT_TXRDY (0x1 << 2)
#define UART_STAT_TXIDLE (0x1 << 3)

// SPI registers bit definitions
#define SPI_CFG_ENABLE		(0x1 << 0)
#define SPI_CFG_SPOL		(0x1 << 8)
#define SPI_STAT_RXRDY		(0x1 << 0)
#define SPI_STAT_TXRDY		(0x1 << 1)
#define SPI_STAT_RXOV		(0x1 << 2)
#define SPI_STAT_TXUR		(0x1 << 3)
#define SPI_STAT_SSA		(0x1 << 4)
#define SPI_STAT_SSD		(0x1 << 5)
#define SPI_STAT_STALLED	(0x1 << 6)
#define SPI_STAT_ENDTRANSFER	(0x1 << 7)
#define SPI_RXDAT_RXSSEL_N	(0x1 <<16)
#define SPI_RXDAT_SOT		(0x1 <<20)
#define SPI_TXDATCTL_RXIGNORE	(0x1 << 22)
#define SPI_TXDATCTL_FLEN(n)	((n & 0xf) << 24)	

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

  // Enable GPIO(6), SPI0(11) and UART0(14) clock
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6)|(1 << 11)|(1 << 14);

  // Reset GPIO(10), SPI0(0) and UART(3)
  LPC_SYSCON->PRESETCTRL &= ~((1 << 0)|(1 << 3)|(1 << 10));
  LPC_SYSCON->PRESETCTRL |= ((1 << 0)|(1 << 3)|(1 << 10));

  // Enable U0-TXD on PIO0_0 PINASSIGN0(7:0)
  LPC_SWM->PINASSIGN0 = 0xffffff00;
  // Enable SPI0_SCK on PIO0_12 PINASSIGN3(31:24)
  LPC_SWM->PINASSIGN3 = 0x0cffffff;
  // Enable SPI0_MOSI on PIO0_4 PINASSIGN4(7:0)
  // Enable SPI0_MISO on PIO0_13 PINASSIGN4(15:8)
  // Enable SPI0_SSEL on PIO0_1 PINASSIGN4(23:16)
  LPC_SWM->PINASSIGN4 = 0xff010d04;

  // UART0 baud rate
  LPC_SYSCON->UARTCLKDIV = 1;
  LPC_SYSCON->UARTFRGDIV = 255;
  // UARTFRGMUL 115200->207, 250000->0,  500000->128
  // BRG        115200->17,  250000->14, 500000->4
  // 250000 for XBUS
  LPC_SYSCON->UARTFRGMULT = 0;
  LPC_USART0->BRG = 14;

  // UART0 enabled, 8 bit, no parity, 1 stop bit, no flow control
  LPC_USART0->CFG = 0x05;

  // Configure SPI0 as slave
  LPC_SPI0->CFG = SPI_CFG_ENABLE;

  // Enable SPI0 and UART0 intr with NVIC
  NVIC_ISER = (1 << SPI0_IRQn)|(1 << UART0_IRQn);

  // Wait 200ms not to get noise when going on hot start
  *SYST_RVR = 6000000-1;
  *SYST_CVR = 0;
  *SYST_CSR = 5;
  while (!(*SYST_CSR & (1 << 16)))
    ;

  main_loop ();
}

#define NUM_CHANNELS 8
#define XBUS_PKT_SIZE (4 + NUM_CHANNELS * (2 + 2) + 1)

static volatile uint8_t xpkt[XBUS_PKT_SIZE];
static volatile int xpkt_index;

static void
uart0_handler (void)
{
  if (LPC_USART0->INTSTAT & UART_STAT_TXRDY)
    {
      if (xpkt_index < XBUS_PKT_SIZE) {
	LPC_USART0->TXDATA = xpkt[xpkt_index++];
      }
      if (xpkt_index >= XBUS_PKT_SIZE) {
	xpkt_index = 0;
	LPC_USART0->INTENCLR = UART_STAT_TXRDY;
      }
    }
}

static void
send_uart (uint8_t b)
{
#if defined(USE_USART_POLLING)
  while(~LPC_USART0->STAT & UART_STAT_TXRDY)
    ;
  LPC_USART0->TXDATA = b;
  while(~LPC_USART0->STAT & UART_STAT_TXIDLE)
    ;
#else
  xpkt[xpkt_index++] = b;
#endif
}

static volatile uint8_t spiregs[128];
static volatile uint8_t spi_regdata;

#define FRAME 0x7f

static void
spi0_handler (void)
{
  uint32_t intstat = LPC_SPI0->INTSTAT;
  if (intstat & SPI_STAT_TXRDY)
    {
      LPC_SPI0->TXDATCTL = SPI_TXDATCTL_FLEN(15) | spi_regdata;
    }
  if (intstat & SPI_STAT_RXRDY)
    {
      uint16_t data = LPC_SPI0->RXDAT;
      uint8_t regno = data >> 8;
      if (regno & 0x80)
	{
	  regno &= 0x7f;
	  spi_regdata = spiregs[regno];
	}
      else
	spiregs[regno] = data & 0xff;
    }
}

#define NUM_CHANNELS 8

#define leu16_val(v, idx) (((uint16_t)v[2*idx+1] << 8) | v[2*idx])

/* Map PWM range to XBUS range.
   800u -> 0x0000
   2200u -> 0xffff
*/
static inline uint16_t xconv(uint16_t pwm)
{
  if (pwm <= 800) {
    return 0;
  } else if (pwm >= 2200) {
    return 0xffff;
  }
  // Approximate 0xffff/1400
  return ((pwm - 800) * 23967) >> 9;
}

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

/* XBUS packet is a byte sequence which looks like:
  command(0xa4), length(2+4*(1-50)), key(0), type(0),
  ch-id(1-50), ch-func(0), ch-data-high, ch-data-low,
  repeated ch-* stuff
  crc8 (x^8+x^5+x^4+1)
*/

static void
xbus (void)
{
  int i;
  uint8_t d;
  uint8_t crc = 0;
  uint16_t pwm;

  crc = crc8_array[(crc ^ 0xa4) & 0xff];
  crc = crc8_array[(crc ^ (2+4*NUM_CHANNELS)) & 0xff];
  send_uart (0xa4);
  send_uart (2+4*NUM_CHANNELS);
  crc = crc8_array[(crc ^ 0) & 0xff];
  crc = crc8_array[(crc ^ 0) & 0xff];
  send_uart (0);
  send_uart (0);

  for (i = 1; i <= NUM_CHANNELS; i++)
    {
      crc = crc8_array[(crc ^ i) & 0xff];
      send_uart (i);
      crc = crc8_array[(crc ^ 0) & 0xff];
      send_uart (0);
      pwm = xconv(leu16_val(spiregs, i-1));
      d = (uint8_t) (pwm >> 8);
      crc = crc8_array[(crc ^ d) & 0xff];
      send_uart (d);
      d = (uint8_t) (pwm & 0xff);
      crc = crc8_array[(crc ^ d) & 0xff];
      send_uart (d);
    }
  send_uart (crc);

#if !defined(USE_USART_POLLING)
  xpkt_index = 0;
  // Enable txrdy inter.  Start send packet.
  LPC_USART0->INTENSET = UART_STAT_TXRDY;
#endif
}

static void
main_loop (void)
{
  // Disable usart0 txrdy inter
  LPC_USART0->INTENCLR = UART_STAT_TXRDY;

#if !defined(USE_SPI_POLLING)
  // Enable SPI0 inter
  LPC_SPI0->INTENSET = SPI_STAT_TXRDY|SPI_STAT_RXRDY;
#endif

  // Unmask all interrupts
  asm volatile ("cpsie      i" : : : "memory");

  for (;;)
    {
#if defined(USE_SPI_POLLING)
      uint16_t data, regno;

      while(~LPC_SPI0->STAT & SPI_STAT_TXRDY)
	;
      LPC_SPI0->TXDATCTL = SPI_TXDATCTL_FLEN(15);
      while(~LPC_SPI0->STAT & SPI_STAT_RXRDY)
	;
      data = LPC_SPI0->RXDAT;
      regno = data >> 8;
      data = data & 0xff;
      if (regno != FRAME)
	spiregs[regno] = data;
      else
	xbus ();
#else
      bool eof;

      asm volatile ("cpsid      i" : : : "memory");
      if (spiregs[FRAME] & 1)
	{
	  eof = true;
	  spiregs[FRAME] &= ~1;
	}
      else
	eof = false;
      asm volatile ("cpsie      i" : : : "memory");
      if (eof)
	xbus();
#endif
    }
}
