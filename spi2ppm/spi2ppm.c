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
static void sct_handler (void);
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
  unexpected,	/* 19: UART0 */
  unexpected,	/* 20: UART1 */
  unexpected,	/* 21: UART2 */
  unexpected,	/* 22: Reserve */
  unexpected,	/* 23: Reserve */
  unexpected,	/* 24: I2C */
  sct_handler,	/* 25: SCT */
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
#define SCT_IRQn (25-16)

// SCT counter prescale 1/15 which means 30/15=2MHz clock
#define SCT_CTRL_PRESCALE	(14 << 5)

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

#define TPAUSE		500
#define TNUETRAL	1500
#define TSYNC		4000
#define TMAX		(100*1000)

// SCT prescaled so as to SCT clock is 2MHz
static inline uint32_t usec2ticks (uint32_t usec) { return (usec << 1);}

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

  // Enable GPIO(6), SPI0(11), SCT(8) clock
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6)|(1 << 8)|(1 << 11);

  // Reset GPIO(10), SPI0(0) and SCT(6)
  LPC_SYSCON->PRESETCTRL &= ~((1 << 0)|(1 << 8)|(1 << 10));
  LPC_SYSCON->PRESETCTRL |= ((1 << 0)|(1 << 8)|(1 << 10));

  // Set initial outputs high
  LPC_SCT->OUTPUT = 0x1;

  // Enable SPI0_SCK on PIO0_12 PINASSIGN3(31:24)
  LPC_SWM->PINASSIGN3 = 0x0cffffff;
  // Enable SPI0_MOSI on PIO0_4 PINASSIGN4(7:0)
  // Enable SPI0_MISO on PIO0_13 PINASSIGN4(15:8)
  // Enable SPI0_SSEL on PIO0_1 PINASSIGN4(23:16)
  LPC_SWM->PINASSIGN4 = 0xff010d04;
  // Enable SCOUT_0 on PIO0_0 PINASSIGN6(31:24)
  LPC_SWM->PINASSIGN6 = 0x00ffffff;

#if 0
  // PIO0_10 is an output (open-drain), default High
  LPC_GPIO_PORT->B0[10] = 1;
  LPC_GPIO_PORT->DIR0 |= (1 << 10);
#endif

  // Configure SPI0 as slave
  LPC_SPI0->CFG = SPI_CFG_ENABLE;

  // Configure SCT
  // [0]=1:32-bit op, [2:1]=0:Prescaled bus clock, [16:9]=1:INSYNC, other 0
  LPC_SCT->CONFIG = 0x1|(1 << 9);
  // As match registers
  LPC_SCT->REGMODE_L = 0;
  // Set initial outputs high
  LPC_SCT->OUTPUT = 0x1;
  // Output doesn't depend on counter direction
  LPC_SCT->OUTPUTDIRCTRL = 0;
  // Match register 0 for 500usec always
  LPC_SCT->MATCH[0].U = usec2ticks (TPAUSE);
  LPC_SCT->MATCHREL[0].U = usec2ticks (TPAUSE);
  // Match register 2 for 100msec always
  LPC_SCT->MATCH[2].U = usec2ticks (TMAX);
  LPC_SCT->MATCHREL[2].U = usec2ticks (TMAX);
  // Enable event 0 in state 0
  LPC_SCT->EVENT[0].STATE = 0x1;
  // [3:0]=1:Match 0, [13:12]=1:Match
  LPC_SCT->EVENT[0].CTRL = (0 << 0)|(1 << 12);
  // Enable event 1 in state 0
  LPC_SCT->EVENT[1].STATE = 0x1;
  // [3:0]=2:Match 1, [13:12]=1:Match
  LPC_SCT->EVENT[1].CTRL = (1 << 0)|(1 << 12);
  // Enable event 2 in state 0
  LPC_SCT->EVENT[2].STATE = 0x1;
  // [3:0]=2:Match 1, [13:12]=1:Match
  LPC_SCT->EVENT[2].CTRL = (2 << 0)|(1 << 12);
  // Set with event 0
  LPC_SCT->OUT[0].SET = 1;
  // Clear with event 1
  LPC_SCT->OUT[0].CLR = 2;
  // Reset counter with event 1
  LPC_SCT->LIMIT_L = (1 << 1);
  // Reset counter with event 2
  LPC_SCT->STOP_L = (1 << 2);
  // Halt, clear counter, 1/15 prescale
  LPC_SCT->CTRL_U = (1 << 3)|(1 << 2)|SCT_CTRL_PRESCALE;

  // Enable SPI0 and SCT intr with NVIC
  NVIC_ISER = (1 << SPI0_IRQn)|(1 << SCT_IRQn);

  // Wait 20ms not to get noise when going on hot start
  *SYST_RVR = 6000000-1;
  *SYST_CVR = 0;
  *SYST_CSR = 5;
  while (!(*SYST_CSR & (1 << 16)))
    ;

  main_loop ();
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

#define PPMSUM_NUM_CHANNELS 8

static uint32_t ppmsum_pw[PPMSUM_NUM_CHANNELS + 2] =
  {
    TNUETRAL, TNUETRAL, TNUETRAL, TNUETRAL,
    TNUETRAL, TNUETRAL, TNUETRAL, TNUETRAL,
    TSYNC, TMAX
  };
static uint32_t pw_index = 0;

static void
sct_handler (void)
{
  uint32_t ev;

  ev = LPC_SCT->EVFLAG;
  LPC_SCT->EVFLAG = ev;
  if (pw_index == PPMSUM_NUM_CHANNELS + 1)
    {
      // SYNC: Don't clear at event 1
      LPC_SCT->OUT[0].CLR = 0;
    }
  else if (pw_index > PPMSUM_NUM_CHANNELS + 1)
    {
      // Halt timer, reset index and disable event 1.
      LPC_SCT->CTRL_U = (1 << 2)|SCT_CTRL_PRESCALE;
      pw_index = 0;
      LPC_SCT->EVENT[1].STATE = 0;
      // Rerun
      LPC_SCT->CTRL_U = SCT_CTRL_PRESCALE;
      return;
    }

  LPC_SCT->MATCHREL[1].U = usec2ticks (ppmsum_pw[pw_index++]);
}

#define leu16_val(v, idx) ((uint16_t)(((uint16_t)v[2*idx+1] << 8) | v[2*idx]))

static void
start_ppmsum (volatile uint8_t *regs)
{
  // Force reset sequence
  pw_index = 0;
  
  for (int ch = 0; ch < PPMSUM_NUM_CHANNELS; ch++)
    ppmsum_pw[ch] = leu16_val(regs, ch);
  
  // Strat PPM-sum sequence
  LPC_SCT->CTRL_U = (1 << 2)|SCT_CTRL_PRESCALE;
  // clear counter
  LPC_SCT->CTRL_U = (1 << 3)|(1 << 2)|SCT_CTRL_PRESCALE;
  LPC_SCT->OUTPUT = 0;
  LPC_SCT->MATCH[1].U = usec2ticks (ppmsum_pw[pw_index++]);
  LPC_SCT->MATCHREL[1].U = usec2ticks (ppmsum_pw[pw_index++]);
  LPC_SCT->OUT[0].CLR = 2;
  LPC_SCT->EVENT[1].STATE = 1;
  LPC_SCT->CTRL_U = SCT_CTRL_PRESCALE;
}

static void
main_loop (void)
{
  // Enable SCT event 1 to request interrupt.
  LPC_SCT->EVEN = (1 << 1);

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
	{
	  asm volatile ("cpsid      i" : : : "memory");
	  start_ppmsum (spiregs);
	  asm volatile ("cpsie      i" : : : "memory");
	}
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
	{
	  asm volatile ("cpsid      i" : : : "memory");
	  start_ppmsum (spiregs);
	  asm volatile ("cpsie      i" : : : "memory");
	}
#endif
    }
}
