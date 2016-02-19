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
static void stc_handler (void);
static void uart0_handler (void);
static void uart1_handler (void);

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
  uart1_handler,/* 20: UART1 */
  unexpected,	/* 21: UART2 */
  unexpected,	/* 22: Reserve */
  unexpected,	/* 23: Reserve */
  unexpected,	/* 24: I2C */
  stc_handler,	/* 25: SCT */
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

#define STC_IRQn (25-16)
#define UART0_IRQn (19-16)
#define UART1_IRQn (20-16)

// STC counter prescale 1/3 which means 30/3=10MHz clock
#define STC_CTRL_PRESCALE	(2 << 5)

#define UART_STAT_RXRDY		(0x1 << 0)
#define UART_STAT_RXIDLE	(0x1 << 1)
#define UART_STAT_TXRDY		(0x1 << 2)
#define UART_STAT_TXIDLE	(0x1 << 3)
#define UART_STAT_OVERRUN	(0x1 << 8)
#define UART_STAT_FRAMERR	(0x1 << 13)
#define UART_STAT_PARITYERR	(0x1 << 14)
#define UART_STAT_RXNOISE	(0x1 << 15)

#define DEFAULT_UART0_BRG	215

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

  // Enable SCT and UART0/1 clock
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8)|(3 << 14);

  // Reset SCT and UART0/1
  LPC_SYSCON->PRESETCTRL &= ~((1 << 8)|(3 << 3));
  LPC_SYSCON->PRESETCTRL |= ((1 << 8)|(3 << 3));

  // Enable U0-RXD on PIO0_6 PINASSIGN0(15:8)
  // Enable U0-TXD on PIO0_7 PINASSIGN0(7:0)
  LPC_SWM->PINASSIGN0 = 0xffff0607;
  // Enable U1-RTS on PIO0_12 PINASSIGN1(31:24)
  // Enable U1-RXD on PIO0_9 PINASSIGN1(23:16)
  // Enable U1-TXD on PIO0_1 PINASSIGN1(15:8)
  LPC_SWM->PINASSIGN1 = 0x0c0901ff;
  // Enable U1-CTS on PIO0_13 PINASSIGN2(7:0)
  LPC_SWM->PINASSIGN2 = 0xffffff0d;

  // Enable SCIN_0 on PIO0_6 PINASSIGN5(31:24) i.e. same with U0-RXD
  LPC_SWM->PINASSIGN5 = 0x06ffffff;

  // UARTs baud rate
  LPC_SYSCON->UARTCLKDIV = 1;
  LPC_SYSCON->UARTFRGDIV = 255;
  // UARTFRGMUL 230400->207, 115200->207, 38400->207, ...
  // BRG        230400->8,   115200->17,  38400->53,  9600->215
  LPC_SYSCON->UARTFRGMULT = 207;
  // UART0 default 9600
  LPC_USART0->BRG = DEFAULT_UART0_BRG;
#if defined(UART1_BAUDRATE_115200)
  // UART1 fixed 115200
  LPC_USART1->BRG = 17;
#else
  // UART1 fixed 115200
  LPC_USART1->BRG = 8;
#endif
  // UART0 enabled, 8 bit, no parity, 1 stop bit, no flow control
  LPC_USART0->CFG = 0x05;
  // UART1 enabled, 8 bit, no parity, 1 stop bit, hard flow control
  LPC_USART1->CFG = (1 << 9)|0x05;

  // [0]=1:32-bit op, [2:1]=0:Prescaled bus clock, [16:9]=1:INSYNC, other 0
  LPC_SCT->CONFIG = 0x1|(0 << 1)|(1 << 9);
  // As capture registers
  LPC_SCT->REGMODE_L = 0x1f;
  // Enable event 0 in state 0
  LPC_SCT->EVENT[0].STATE = 0x1;
  // [5]=0:Input, [9:6]=0:CTIN_0, [11:10]=1: Rise, [13:12]=2: IO
  LPC_SCT->EVENT[0].CTRL = (1 << 10)|(2 << 12);
  // Enable event 0 in state 0
  LPC_SCT->EVENT[1].STATE = 0x1;
  // [5]=0:Input, [9:6]=0:CTIN_0, [11:10]=2: Fall, [13:12]=2: IO
  LPC_SCT->EVENT[1].CTRL = (2 << 10)|(2 << 12);
  // Capture 0 for event 0
  LPC_SCT->CAPCTRL[0].U = 1;
  // Capture 1 for event 1
  LPC_SCT->CAPCTRL[1].U = 2;

  // Enable STC and UART0/1 intr with NVIC
  NVIC_ISER = (1 << STC_IRQn)|(3 << UART0_IRQn);

  // Wait 20ms not to get noise when going on hot start
  *SYST_RVR = 6000000-1;
  *SYST_CVR = 0;
  *SYST_CSR = 5;
  while (!(*SYST_CSR & (1 << 16)))
    ;

  // ENABLE UART0/1 RXRDY interrupt requests
  LPC_USART0->INTENSET = (UART_STAT_RXRDY
			  | UART_STAT_FRAMERR | UART_STAT_RXNOISE);
  LPC_USART1->INTENSET = UART_STAT_RXRDY;

  // Enable SCT event 0 and 1 interrupt requests
  LPC_SCT->EVEN = 0x1|0x2;

  main_loop ();
}

/* For the auto baudrate detection, we record some pulse width so to
   find the minimal pulse width in uart0 RX signal.  If we find something
   which can't be there with the current baud rate, vote to request
   the new buad rate matched with that.  Also framing error and rx noise
   error detected with uart0 RX raise it. 
*/
static uint32_t vote_for_new_rate = 0;

static struct brg_table
{
  uint32_t brg;
  uint16_t llim, hlim;
  
} btable[] =
  {
    { 215, 1009, 1074, },	//   9600
    { 107,  504,  537, },	//  11920
    {  53,  252,  268, },	//  38400
    {  35,  168,  179, },	//  57600
    {  17,   83,   90, },	// 115200
  };
#define NBAUD (sizeof (btable)/sizeof (btable[0]))

static uint32_t current_brg = DEFAULT_UART0_BRG;
static uint16_t current_llim = 1009;

#define DT_HLIM 0xffff
// 10*100ns is too narrow
#define DT_LLIM 10

static uint32_t tlast;
static uint16_t dt_min = DT_HLIM;

static void
stc_handler (void)
{
  uint32_t dt, tnow;
  uint32_t ev;

  // Get capture reg and compute DT
  ev = LPC_SCT->EVFLAG;
  if ((ev & 3) == 3)
    {
      // Ignore this case as noise
      LPC_SCT->EVFLAG = 1|2;
      return;
    }
  if (ev & 1)
    {
      LPC_SCT->EVFLAG = 1; // Clear interrupt request for this event.
      tnow = LPC_SCT->CAP[0].U;
      dt = tnow - tlast;
      // Not to wrap up counter
      if (tnow >> 24)
	{
	  // Halt SCT, reset counter and run it again
	  LPC_SCT->CTRL_U = (1 << 2)|STC_CTRL_PRESCALE;
	  tnow = LPC_SCT->COUNT_U - tnow;
	  LPC_SCT->COUNT_U = tnow;
	  LPC_SCT->CTRL_U = STC_CTRL_PRESCALE;
	}
      tlast = tnow;
    }
  else if (ev & 2)
    {
      LPC_SCT->EVFLAG = 2; // Clear interrupt request for this event.
      tnow = LPC_SCT->CAP[1].U;
      dt = tnow - tlast;
      tlast = tnow;
    }
  else
    // Can be ignored as sprious interrupt?
    return;
  // Ignore too narrow dt as noise
  if (dt < DT_LLIM)
    return;
  if (dt < current_llim)
    vote_for_new_rate += 4;
  if (dt < dt_min)
    dt_min = dt;
}

#if defined(LPC810)
#define FIFOSZ 256
#else
#define FIFOSZ 1024
#endif
static uint8_t rx0_fifo[FIFOSZ];
static uint32_t rx0_begin;
static uint32_t rx0_size;
static uint8_t rx1_fifo[FIFOSZ];
static uint32_t rx1_begin;
static uint32_t rx1_size;

static void
uart0_handler (void)
{
  if (LPC_USART0->INTSTAT & UART_STAT_RXRDY)
    {
      if (rx0_size >= FIFOSZ)
	{
	  // Overrun
	  (void) LPC_USART0->RXDATA;
	}
      else
	{
	  rx0_fifo[(rx0_begin + rx0_size) % FIFOSZ] = LPC_USART0->RXDATA;
	  ++rx0_size;
	}
    }
  if (LPC_USART0->STAT & UART_STAT_FRAMERR)
    {
      LPC_USART0->STAT = UART_STAT_FRAMERR;
      vote_for_new_rate += 2;
    }
  if (LPC_USART0->STAT & UART_STAT_RXNOISE)
    {
      LPC_USART0->STAT = UART_STAT_RXNOISE;
      vote_for_new_rate++;
    }
}

static void
uart1_handler (void)
{
  if (LPC_USART1->INTSTAT & UART_STAT_RXRDY)
    {
      if (rx1_size >= FIFOSZ)
	{
	  // Overrun
	  (void) LPC_USART1->RXDATA;
	}
      else
	{
	  rx1_fifo[(rx1_begin + rx1_size) % FIFOSZ] = LPC_USART1->RXDATA;
	  ++rx1_size;
	}
    }
}

static void
send_uart0 (uint8_t b)
{
  while(~LPC_USART0->STAT & UART_STAT_TXRDY)
    ;
  LPC_USART0->TXDATA = b;
  while(~LPC_USART0->STAT & UART_STAT_TXIDLE)
    ;
}

static void
send_uart1 (uint8_t b)
{
  while(~LPC_USART1->STAT & UART_STAT_TXRDY)
    ;
  LPC_USART1->TXDATA = b;
  while(~LPC_USART1->STAT & UART_STAT_TXIDLE)
    ;
}

// Guess uart0 rx baud rate with pulse width sampling
// If votes is above this value, activate ABD
#define ABD_THRESHOLD 7

static void
guess_baudrate (void)
{
  uint16_t min;

  asm volatile ("cpsid	i" : : : "memory");
  min = dt_min;
  asm volatile ("cpsie	i" : : : "memory");

  if (min < DT_HLIM)
    {
      uint32_t idx;

      for (idx = 0; idx < NBAUD; idx++)
	{
	  if (min >= btable[idx].llim && min <= btable[idx].hlim)
	    break;
	}

      if (idx < NBAUD && btable[idx].brg != current_brg)
	{
	  current_brg = btable[idx].brg;
	  LPC_USART0->BRG = current_brg;
	  current_llim = btable[idx].llim;
	  send_uart1 (current_brg);
	}

      asm volatile ("cpsid	i" : : : "memory");
      dt_min = DT_HLIM;
      vote_for_new_rate = 0;
      asm volatile ("cpsie	i" : : : "memory");
    }
}

static void
main_loop (void)
{
  uint8_t c = 0;
  int n;

  // Initialize pulse capture stuff
  dt_min = DT_HLIM;
  vote_for_new_rate = 0;

  // Start, clear counter, 1/3 prescale
  LPC_SCT->CTRL_U = (1 << 3)|STC_CTRL_PRESCALE;

  // Unmask all interrupts
  asm volatile ("cpsie      i" : : : "memory");

  for (;;)
    {
      // Critical section
      asm volatile ("cpsid	i" : : : "memory");
      if (rx0_size)
	{
	  c = rx0_fifo[rx0_begin];
	  rx0_begin = (rx0_begin + 1) % FIFOSZ;
	  --rx0_size;
	  n = 1;
	}
      else
	n = 0;
      asm volatile ("cpsie      i" : : : "memory");
      if (n)
	send_uart1 (c);

      // Critical section
      asm volatile ("cpsid	i" : : : "memory");
      if (rx1_size)
	{
	  c = rx1_fifo[rx1_begin];
	  rx1_begin = (rx1_begin + 1) % FIFOSZ;
	  --rx1_size;
	  n = 1;
	}
      else
	n = 0;
      asm volatile ("cpsie      i" : : : "memory");
      if (n)
	send_uart0 (c);
      
      // Guess baud rate from DT
      if (vote_for_new_rate > ABD_THRESHOLD)
	guess_baudrate ();
    }
}
