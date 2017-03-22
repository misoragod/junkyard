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
  spi0_handler,	/* 16: SPI0 */
  unexpected,	/* 17: SPI1 */
  unexpected,	/* 18: Reserve */
  uart0_handler,/* 19: UART0 */
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
#define UART0_IRQn (19-16)
#define SCT_IRQn (25-16)

// SCT counter prescale 1/15 which means 30/15=2MHz clock
#define SCT_CTRL_PRESCALE	(14 << 5)

// UART register bit definitions
#define UART_STAT_RXRDY		(0x1 << 0)
#define UART_STAT_RXIDLE	(0x1 << 1)
#define UART_STAT_TXRDY		(0x1 << 2)
#define UART_STAT_TXIDLE	(0x1 << 3)
#define UART_STAT_OVERRUN	(0x1 << 8)
#define UART_STAT_FRAMERR	(0x1 << 13)
#define UART_STAT_PARITYERR	(0x1 << 14)
#define UART_STAT_RXNOISE	(0x1 << 15)

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

  // Enable GPIO(6), SPI0(11), SCT(8) and UART0(14) clock
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6)|(1 << 8)|(1 << 11)|(1 << 14);

  // Reset GPIO(10), SPI0(0), UART0(3) and SCT(6)
  LPC_SYSCON->PRESETCTRL &= ~((1 << 0)|(1 << 3)|(1 << 8)|(1 << 10));
  LPC_SYSCON->PRESETCTRL |= ((1 << 0)|(1 << 3)|(1 << 8)|(1 << 10));

  // Enable U0-RXD on PIO0_6 PINASSIGN0(15:8)
  // Enable U0-TXD on PIO0_7 PINASSIGN0(7:0)
  LPC_SWM->PINASSIGN0 = 0xffff0607;
  // Enable SPI0_SCK on PIO0_12 PINASSIGN3(31:24)
  LPC_SWM->PINASSIGN3 = 0x0cffffff;
  // Enable SPI0_MOSI on PIO0_4 PINASSIGN4(7:0)
  // Enable SPI0_MISO on PIO0_13 PINASSIGN4(15:8)
  // Enable SPI0_SSEL on PIO0_1 PINASSIGN4(23:16)
  LPC_SWM->PINASSIGN4 = 0xff010d04;
  // Enable SCIN_0 on PIO0_0 PINASSIGN5(31:24)
  LPC_SWM->PINASSIGN5 = 0x00ffffff;

  // PIO0_10 is an output (open-drain), default High
  LPC_GPIO_PORT->B0[10] = 1;
  LPC_GPIO_PORT->DIR0 |= (1 << 10);

  // UARTs baud rate
  LPC_SYSCON->UARTCLKDIV = 1;
  LPC_SYSCON->UARTFRGDIV = 255;
  // UARTFRGMUL 230400->207, 115200->207, 38400->207, ...
  // BRG        230400->8,   115200->17,  38400->53,  9600->215
  LPC_SYSCON->UARTFRGMULT = 207;
  // UART0 fixed 57600
  LPC_USART0->BRG = 35;
  // UART0 enabled, 8 bit, no parity, 1 stop bit, no flow control
  LPC_USART0->CFG = 0x05;

  // Configure SPI0 as slave
  LPC_SPI0->CFG = SPI_CFG_ENABLE;

  // [0]=1:32-bit op, [2:1]=0:Prescaled bus clock, [16:9]=1:INSYNC, other 0
  LPC_SCT->CONFIG = 0x1|(1 << 9);
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
  // Start, clear counter, 1/15 prescale
  LPC_SCT->CTRL_U = (1 << 3)|SCT_CTRL_PRESCALE;

  // Enable SPI0, UART0 and SCT intr with NVIC
  NVIC_ISER = (1 << SPI0_IRQn)|(1 << SCT_IRQn)|(1 << UART0_IRQn);

  // Wait 20ms not to get noise when going on hot start
  *SYST_RVR = 6000000-1;
  *SYST_CVR = 0;
  *SYST_CSR = 5;
  while (!(*SYST_CSR & (1 << 16)))
    ;

  // Enable SCT event 0 and 1 to request interrupt.
  LPC_SCT->EVEN = 0x1|0x2;

  main_loop ();
}

#define FIFOSZ 512
static uint8_t rx0_fifo[FIFOSZ];
static uint32_t rx0_begin;
static uint32_t rx0_size;
static uint8_t tx0_fifo[FIFOSZ];
static uint32_t tx0_begin;
static uint32_t tx0_size;

static void
uart0_handler (void)
{
  uint32_t intstat = LPC_USART0->INTSTAT;
  if (intstat & UART_STAT_TXRDY)
    {
      if (tx0_size)
	{
	  LPC_USART0->TXDATA = tx0_fifo[tx0_begin];
	  tx0_begin = (tx0_begin + 1) % FIFOSZ;
	  --tx0_size;
	}
      else
	LPC_USART0->INTENCLR = UART_STAT_TXRDY;	
    }
  if (intstat & UART_STAT_RXRDY)
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
}

static volatile uint8_t spiregs[128];
static volatile uint8_t ppm_ready = 0;
static volatile uint8_t spi_regdata;

#define READY           0x7f
#define CLEAR_READY     0x7e

#define RX_READY        0x7d
#define RXREG           0x40
#define TXREG           0x41

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
      uint16_t rxdat = LPC_SPI0->RXDAT;
      uint8_t regno = rxdat >> 8;
      if (regno & 0x80)
	{
	  regno &= 0x7f;
	  if (regno == READY)
	    spi_regdata = ppm_ready;
	  else if (regno == RX_READY)
	    spi_regdata = (rx0_size > 0) ? 1 : 0;
	  else if (regno == RXREG)
	    spi_regdata = rx0_fifo[rx0_begin];
	  else
	    spi_regdata = spiregs[regno];
	}
      else
	{
	  if (regno == CLEAR_READY)
	    ppm_ready = 0;
	  else if (regno == RXREG)
	    {
	      if (rx0_size > 0)
		{
		  rx0_begin = (rx0_begin+1) % FIFOSZ;
		  --rx0_size;
		}
	    }
	  else if (regno == TXREG)
	    {
	      if (tx0_size < FIFOSZ)
		{
		  tx0_fifo[(tx0_begin + tx0_size) % FIFOSZ] = rxdat & 0xff;
		  ++tx0_size;	      
		}
	      LPC_USART0->INTENSET = UART_STAT_TXRDY;
	    }
	  else
	    spiregs[regno] = rxdat & 0xff;
	}
    }
}

#define CPPM_NUM_CHANNELS 8
struct {
  int _channel_counter;
  uint16_t _pulse_capt[CPPM_NUM_CHANNELS];
} ppm_state;

/* Process PPM-sum pulse to pwm values, based on the implementation
   of ardupilot/libraries/AP_HAL_Linux/RCInput.cpp.  */
static void
ppmsum_pulse (uint16_t width_usec)
{
  if (width_usec >= 2700)
    {
      /* A long pulse indicates the end of a frame. Reset the channel
	 counter so next pulse is channel 0.  */
      if (ppm_state._channel_counter >= 5 && ppm_ready == 0)
	{
	  for (int i = 0; i < ppm_state._channel_counter; i++)
	    {
	      spiregs[2*i] = (ppm_state._pulse_capt[i] & 0xff);
	      spiregs[2*i+1] = (ppm_state._pulse_capt[i] >> 8);
	    }
	  ppm_ready = 1;
        }
      ppm_state._channel_counter = 0;
      return;
    }
  if (ppm_state._channel_counter == -1)
    {
      // we are not synchronised
      return;
    }

  /* We limit inputs to between 800usec and 2200usec for XBUS. This
     also allows us to decode SBUS on the same pin, as SBUS will have
     a maximum pulse width of 100usec.  */
  if (width_usec >= 800 && width_usec <= 2200)
    {
      // take a reading for the current channel buffer these
      ppm_state._pulse_capt[ppm_state._channel_counter] = width_usec;

      // move to next channel
      ppm_state._channel_counter++;
    }

  /* If we have reached the maximum supported channels then mark
     as unsynchronised, so we wait for a wide pulse.  */
  if (ppm_state._channel_counter == CPPM_NUM_CHANNELS)
    {
      if (ppm_ready == 0)
	{
	  for (int i = 0; i < CPPM_NUM_CHANNELS; i++)
	    {
	      spiregs[2*i] = (ppm_state._pulse_capt[i] & 0xff);
	      spiregs[2*i+1] = (ppm_state._pulse_capt[i] >> 8);
	    }
	  ppm_ready = 1;
	}

      ppm_state._channel_counter = -1;
    }
}

// Ignore too narrow pulse as noise.
#define DT_MIN 3

static uint32_t tlast;
static uint32_t dt_high, dt_low;

static void
sct_handler (void)
{
  uint32_t dt, tnow;
  uint32_t ev;

  /* Get capture reg and compute DT.  Encode rise/fall to 0th bit.  */
  ev = LPC_SCT->EVFLAG;
  if ((ev & 3) == 3)
    {
      LPC_SCT->EVFLAG = 1|2;
      if (LPC_SCT->CAP[0].U > LPC_SCT->CAP[1].U)
	dt = LPC_SCT->CAP[0].U - LPC_SCT->CAP[1].U;
      else
	dt = LPC_SCT->CAP[1].U > LPC_SCT->CAP[0].U;
      // ATM, ignore this case
      return;
    }
  if (ev & 1)
    {
      LPC_SCT->EVFLAG = 1; // Clear interrupt request for this event.
      tnow = LPC_SCT->CAP[0].U;
      dt = tnow - tlast;
#if 1
      // Not to wrap up counter
      if (tnow >> 24)
	{
	  // Halt SCT, reset counter and run it again
	  LPC_SCT->CTRL_U = (1 << 2)|SCT_CTRL_PRESCALE;
	  tnow = LPC_SCT->COUNT_U - tnow;
	  LPC_SCT->COUNT_U = tnow;
	  LPC_SCT->CTRL_U = SCT_CTRL_PRESCALE;
	}
#endif
      tlast = tnow;
      // Saturate when dt > ~32ms
      if (dt > 0xffff)
	dt = 0xffff;
      dt_low = dt;
    }
  else if (ev & 2)
    {
      LPC_SCT->EVFLAG = 2; // Clear interrupt request for this event.
      tnow = LPC_SCT->CAP[1].U;
      dt = tnow - tlast;
      tlast = tnow;
      // Saturate when dt > ~32ms
      if (dt > 0xffff)
	dt = 0xffff;
      dt_high = dt;
    }
  else
    // Can be ignored as sprious interrupt?
    return;
  if (dt_high && dt_low)
    {
      ppmsum_pulse ((dt_high + dt_low) >> 1);
      dt_high = dt_low = 0;
    }
}

static void
main_loop (void)
{
#if defined(USE_SPI_POLLING)
  uint32_t data = 0;
  uint32_t regno;
#endif

  dt_high = dt_low = 0;

#if !defined(USE_SPI_POLLING)
  // Enable SPI0 inter
  LPC_SPI0->INTENSET = SPI_STAT_TXRDY|SPI_STAT_RXRDY;
#endif
  LPC_USART0->INTENSET = UART_STAT_RXRDY;
  
  // Unmask all interrupts
  asm volatile ("cpsie      i" : : : "memory");

  for (;;)
    {
#if defined(USE_SPI_POLLING)
      while(~LPC_SPI0->STAT & SPI_STAT_TXRDY)
	;
      LPC_SPI0->TXDATCTL = SPI_TXDATCTL_FLEN(15) | data;
      while(~LPC_SPI0->STAT & SPI_STAT_RXRDY)
	;
      asm volatile ("cpsid      i" : : : "memory");
      uint32_t rxdat = LPC_SPI0->RXDAT;
      regno = (rxdat >> 8) & 0x7f;
      if ((rxdat >> 8) & 0x80)
	{
	  if (regno == READY)
	    data = ppm_ready;
	  else
	    data = spiregs[regno];
	}
      else
	{
	  if (regno == CLEAR_READY)
	    {
	      ppm_ready = 0;
	      data = 0;
	    }
	}
      asm volatile ("cpsie      i" : : : "memory");
#endif
    }
}
