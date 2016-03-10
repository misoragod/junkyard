#include <stdint.h>
#include <stdbool.h>

extern void _start (void);
static void stc_handler (void);

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
  unexpected,	/* 19: UART0 */
  unexpected,	/* 20: UART1 */
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

// STC counter prescale 1/15 which means 30/15=2MHz clock
#define STC_CTRL_PRESCALE	(14 << 5)

// UART registers bit definitions
#define UART_STAT_TXRDY (0x1 << 2)
#define UART_STAT_TXIDLE (0x1 << 3)

// I2C registers bit definitions
#define I2C_STAT_MSTPENDING (0x1)
#define I2C_STAT_MSTSTATE (0xe)
#define I2C_STAT_MSTST_IDLE (0x0)
#define I2C_STAT_MSTST_RX (0x2)
#define I2C_STAT_MSTST_TX (0x4)
#define I2C_STAT_MSTST_NACK_ADDR (0x6)
#define I2C_STAT_MSTST_NACK_TX (0x8)
#define I2C_MSTCTL_MSTCONTINUE (0x1)
#define I2C_MSTCTL_MSTSTART (0x2)
#define I2C_MSTCTL_MSTSTOP (0x4)

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

#if defined(LPC810)
  // Set open-drain mode to PIO0_1 and PIO0_4
  LPC_IOCON->PIO0_4 = (1 << 10) | (0x1 << 7) | (0x02 << 3);
  LPC_IOCON->PIO0_1 = (1 << 10) | (0x1 << 7) | (0x02 << 3);
#endif
  
  // Enable I2C, SCT and UART0 clock
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 5)|(1 << 8)|(1 << 14);

  // Reset I2C, SCT and UART0
  LPC_SYSCON->PRESETCTRL &= ~((1 << 8)|(1 << 6)|(1 << 3));
  LPC_SYSCON->PRESETCTRL |= ((1 << 8)|(1 << 6)|(1 << 3));

  // Enable SCIN_0 on PIO0_0 PINASSIGN5(31:24)
  LPC_SWM->PINASSIGN5 = 0x00ffffff;
#if defined(LPC810)
  // Enable I2C-SDA on PIO0_4 PINASSIGN5(31:24)
  LPC_SWM->PINASSIGN7 = 0x04ffffff;
  // Enable I2C-SCL on PIO0_1 PINASSIGN0(7:0)
  LPC_SWM->PINASSIGN8 = 0xffffff01;
#else
  // Enable U0-TXD on PIO0_4 PINASSIGN0(7:0)
  LPC_SWM->PINASSIGN0 = 0xffffff04;
  // Enable I2C-SDA on PIO0_10 PINASSIGN5(31:24)
  LPC_SWM->PINASSIGN7 = 0x0affffff;
  // Enable I2C-SCL on PIO0_11 PINASSIGN0(7:0)
  LPC_SWM->PINASSIGN8 = 0xffffff0b;
#endif

  // UART0 baud rate
  LPC_SYSCON->UARTCLKDIV = 1;
  LPC_SYSCON->UARTFRGDIV = 255;
  // UARTFRGMUL 115200->207, 250000->128,  100000->128
  // BRG        115200->17,  250000->9,    100000->24
  // 100000 for S.BUS
  LPC_SYSCON->UARTFRGMULT = 128;
  LPC_USART0->BRG = 24;

  // UART0 enabled, 8 bit, even parity, 2 stop bit, no flow control
  LPC_USART0->CFG = 0x05|(2 << 4)|(1<< 6);

  // I2C configuration
  // 30M/(375k*(2+2)) = 20
  LPC_I2C->DIV = 19;
  // Master mode
  LPC_I2C->CFG = 0x1;

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
  LPC_SCT->CTRL_U = (1 << 3)|STC_CTRL_PRESCALE;

  // Enable STC intr with NVIC
  NVIC_ISER = 1 << STC_IRQn;

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

#define FIFOSZ 256
static uint16_t dtfifo[FIFOSZ];
static volatile uint32_t dtbegin;
static volatile uint32_t dtsize;
static uint32_t tlast;

// Ignore too narrow pulse as noise.
#define DT_MIN 3

static void
dtoverflow (void)
{
  // This is unexpected and should be treated as fatal error.
  for(;;);
}

static void
stc_handler (void)
{
  uint32_t dt, tnow;
  uint32_t ev;

  if (dtsize == FIFOSZ)
    dtoverflow ();
  else
    {
      /* Get capture reg and compute DT.  Encode rise/fall to 0th bit.  */
      ev = LPC_SCT->EVFLAG;
      if ((ev & 3) == 3)
	{
	  LPC_SCT->EVFLAG = 1|2;
	  if (LPC_SCT->CAP[0].U > LPC_SCT->CAP[1].U)
	    dt = LPC_SCT->CAP[0].U - LPC_SCT->CAP[1].U;
	  else
	    dt = LPC_SCT->CAP[1].U > LPC_SCT->CAP[0].U;
	  if (dt >= DT_MIN)
	    // This is unexpected and should be treated as fatal error.
	    for(;;);
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
	      LPC_SCT->CTRL_U = (1 << 2)|STC_CTRL_PRESCALE;
	      tnow = LPC_SCT->COUNT_U - tnow;
	      LPC_SCT->COUNT_U = tnow;
	      LPC_SCT->CTRL_U = STC_CTRL_PRESCALE;
	    }
#endif
	  tlast = tnow;
	  if (dt & 1)
	    ++dt;
	  dt &= ~1;  // Encode "off" to 0-th bit
	}
      else if (ev & 2)
	{
	  LPC_SCT->EVFLAG = 2; // Clear interrupt request for this event.
	  tnow = LPC_SCT->CAP[1].U;
	  dt = tnow - tlast;
	  tlast = tnow;
	  if (dt & 1)
	    ++dt;
	  dt |= 1;  // Encode "on" to 0-th bit
	}
      else
	// Can be ignored as sprious interrupt?
	return;
      // Saturate when dt > ~32ms
      if (dt > 0xffff)
	dt = 0xffff;
      dtfifo[(dtbegin + dtsize) % FIFOSZ] = (uint16_t) dt;
      ++dtsize;
    }
}

#if 0
static void
send_uart (uint8_t b)
{
  while(~LPC_USART0->STAT & UART_STAT_TXRDY)
    ;
  LPC_USART0->TXDATA = b;
  while(~LPC_USART0->STAT & UART_STAT_TXIDLE)
    ;
}
#endif

static void
i2c_abort (void)
{
  // This is unexpected and should be treated as fatal error.
  for (;;);
}

static void
i2c_write (uint8_t adr, uint8_t reg, uint8_t data)
{
  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_IDLE)
    i2c_abort();
   // address and 0 for RWn bit in order to write reg address
  LPC_I2C->MSTDAT = (adr << 1) | 0;
  // send start
  LPC_I2C->MSTCTL = I2C_MSTCTL_MSTSTART;
  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_TX)
    i2c_abort();
  // send reg address
  LPC_I2C->MSTDAT = reg;
  // continue transaction
  LPC_I2C->MSTCTL = I2C_MSTCTL_MSTCONTINUE;
  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_TX)
    i2c_abort();
  LPC_I2C->MSTDAT = data;
  // continue transaction
  LPC_I2C->MSTCTL = I2C_MSTCTL_MSTCONTINUE;
  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_TX)
    i2c_abort();
  // send stop
  LPC_I2C->MSTCTL = I2C_MSTCTL_MSTSTOP;
  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_IDLE)
    i2c_abort();
}

#if 0
static uint8_t
i2c_read (uint8_t adr, uint8_t reg)
{
  uint8_t data;

  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_IDLE)
    i2c_abort();
  LPC_I2C->MSTDAT = (adr << 1) | 0;
  // send start
  LPC_I2C->MSTCTL = I2C_MSTCTL_MSTSTART;
  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_TX)
    i2c_abort();
  // send reg address
  LPC_I2C->MSTDAT = reg;
  // continue transaction
  LPC_I2C->MSTCTL = I2C_MSTCTL_MSTCONTINUE;
  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_TX)
    i2c_abort();
  // address and 1 for RWn bit in order to read
  LPC_I2C->MSTDAT = (adr << 1) | 1;
  // send repeated start
  LPC_I2C->MSTCTL = I2C_MSTCTL_MSTSTART;
  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_RX)
    i2c_abort();
  data = LPC_I2C->MSTDAT;
  LPC_I2C->MSTCTL = I2C_MSTCTL_MSTSTOP; // send stop
  while (!(LPC_I2C->STAT & I2C_STAT_MSTPENDING))
    ;
  if ((LPC_I2C->STAT & I2C_STAT_MSTSTATE) != I2C_STAT_MSTST_IDLE)
    i2c_abort();
  return data;
}
#endif

// PCA9685 routines
#define PCA9685_ADDRESS            0x40
#define PCA9685_RA_MODE1           0x00
#define PCA9685_RA_MODE2           0x01
#define PCA9685_RA_LED0_ON_L       0x06
#define PCA9685_RA_LED0_ON_H       0x07
#define PCA9685_RA_LED0_OFF_L      0x08
#define PCA9685_RA_LED0_OFF_H      0x09
#define PCA9685_RA_ALL_LED_ON_L    0xFA
#define PCA9685_RA_ALL_LED_ON_H    0xFB
#define PCA9685_RA_ALL_LED_OFF_L   0xFC
#define PCA9685_RA_ALL_LED_OFF_H   0xFD
#define PCA9685_RA_PRE_SCALE       0xFE

#define PCA9685_MODE1_RESTART_BIT  (1 << 7)
#define PCA9685_MODE1_EXTCLK_BIT   (1 << 6)
#define PCA9685_MODE1_AI_BIT       (1 << 5)
#define PCA9685_MODE1_SLEEP_BIT    (1 << 4)
#define PCA9685_MODE1_SUB1_BIT     (1 << 3)
#define PCA9685_MODE1_SUB2_BIT     (1 << 2)
#define PCA9685_MODE1_SUB3_BIT     (1 << 1)
#define PCA9685_MODE1_ALLCALL_BIT  (1 << 0)
#define PCA9685_ALL_LED_OFF_H_SHUT (1 << 4)
#define PCA9685_MODE2_INVRT_BIT    (1 << 4)
#define PCA9685_MODE2_OCH_BIT      (1 << 3)
#define PCA9685_MODE2_OUTDRV_BIT   (1 << 2)
#define PCA9685_MODE2_OUTNE1_BIT   (1 << 1)
#define PCA9685_MODE2_OUTNE0_BIT   (1 << 0)

#define PWM_FREQ_HZ 100

#if (PWM_FREQ_HZ == 200)
#define PCA9685_FREQ_PRESCALE	34
#elif (PWM_FREQ_HZ == 100)
#define PCA9685_FREQ_PRESCALE	67
#elif (PWM_FREQ_HZ == 50)
#define PCA9685_FREQ_PRESCALE	134
#else
#error "unknown freq_hz"
#endif

static void
pwm_out (int ch, uint16_t width)
{
  uint32_t length = 0;
  // length = round((width * 4096)/(1000000.f/(freq_hz*(1+epsilon)) - 1
#if (PWM_FREQ_HZ == 200)
  // approx 0.8201 with 3259/4096
  length = ((width * 3359) >> 12) - 1;
#elif (PWM_FREQ_HZ == 100)
  // approx 0.4099 with 1679/4096
  length = ((width * 1679) >> 12) - 1;
#elif (PWM_FREQ_HZ == 50)
  // approx 0.2063 with 845/4096
  length = ((width * 845) >> 12) - 1;
#else
#error "unknown freq_hz"
#endif

  i2c_write (PCA9685_ADDRESS, PCA9685_RA_LED0_ON_L + 4*ch + 0, 0);
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_LED0_ON_L + 4*ch + 1, 0);
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_LED0_ON_L + 4*ch + 2, length & 0xff);
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_LED0_ON_L + 4*ch + 3, length >> 8);
}

static void
pwm_init (void)
{
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_ALL_LED_ON_L, 0);
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_ALL_LED_ON_H, 0);
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_ALL_LED_OFF_L, 0);
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_ALL_LED_OFF_H, 0);

  // Shutdown before sleeping
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_ALL_LED_OFF_H,
	     PCA9685_ALL_LED_OFF_H_SHUT);
  // Put PCA9685 to sleep so to write prescaler
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT);
  // prescale 67 for freq 99Hz
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_PRE_SCALE, PCA9685_FREQ_PRESCALE);
  // Wait 500us
  *SYST_RVR = 150000-1;
  *SYST_CVR = 0;
  *SYST_CSR = 5;
  while (!(*SYST_CSR & (1 << 16)))
    ;
  // Restart PCA9685
  i2c_write (PCA9685_ADDRESS, PCA9685_RA_MODE1, PCA9685_MODE1_RESTART_BIT);
#if 0
  // test to write PCA9685
  pwm_out (0, 1500);
  pwm_out (1, 800);
  pwm_out (2, 2200);
  pwm_out (3, 1500);
#endif
}

#define SBUS_NUM_CHANNELS 16
#define SBUS_FRAME_SIZE	25
#define SBUS_FLAGS_BYTE	23
#define SBUS_FAILSAFE_BIT 3
#define SBUS_FRAMELOST_BIT 2

struct {
  uint16_t bytes[SBUS_FRAME_SIZE]; // including start bit, parity and stop bits
  uint16_t bit_ofs;
} sbus_state;

static uint16_t pwm_values[SBUS_NUM_CHANNELS];

#define SBUS_ROLL_IDX	(1-1)
#define SBUS_PITCH_IDX	(2-1)
#define SBUS_YAW_IDX	(4-1)

static uint16_t
sat (int v)
{
  if (v < 1000)
    v = 1000;
  else if (v > 2000)
    v = 2000;
  return v;
}

static void
mixer (int num_channels)
{
  uint16_t roll, pitch, yaw;
  int16_t ch1, ch2, ch3, ch4;

  if (num_channels < 4)
    return;

  roll = pwm_values[SBUS_ROLL_IDX];
  pitch = pwm_values[SBUS_PITCH_IDX];
  yaw = pwm_values[SBUS_YAW_IDX];
#if 1
  // mixing
  ch1 = sat ( (pitch - 1500) - (yaw - 1500) + 1500);
  ch2 = sat (-(pitch - 1500) - (yaw - 1500) + 1500);
  ch3 = sat ( (roll - 1500)  - (yaw - 1500) + 1500);
  ch4 = sat (-(roll - 1500)  - (yaw - 1500) + 1500);
#else
  // no mixing
  ch1 = roll;
  ch2 = pitch;
  ch3 = pwm_values[2];
  ch4 = yaw;
#endif
  // write to PCA9685
  pwm_out (0, ch1);
  pwm_out (1, ch2);
  pwm_out (2, ch3);
  pwm_out (3, ch4);
}

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
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};

static const struct sbus_bit_pick sbus_decoder[SBUS_NUM_CHANNELS][3] = {
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

static bool
sbus_decode (const uint8_t frame[SBUS_FRAME_SIZE])
{
  /* check frame boundary markers to avoid out-of-sync cases */
  if (frame[0] != 0x0f)
    return false;

  /* use the decoder matrix to extract channel data */
  for (int ch = 0; ch < SBUS_NUM_CHANNELS; ch++)
    {
      unsigned value = 0;

      for (int pick = 0; pick < 3; pick++)
	{
	  const struct sbus_bit_pick *decode = &sbus_decoder[ch][pick];

	  if (decode->mask != 0)
	    {
	      unsigned piece = frame[1 + decode->byte];
	      piece >>= decode->rshift;
	      piece &= decode->mask;
	      piece <<= decode->lshift;
	      value |= piece;
	    }
	}

      // (2000-1000)/2048.0 * value + 1000
      pwm_values[ch] = ((value * 2001) >> 12) + 1000;
    }

  return true;
}

/* Process S.BUS pulse, based on the implementation of
   ardupilot/libraries/AP_HAL_Linux/RCInput.cpp.  */
static void
sbus_pulse (uint16_t width_s0, uint16_t width_s1)
{
  /* Precision of the internal clock generator is < 1.5%.  */
#define CLK_ADJ 3
  /* uint16_t bits_s0 = (width_s0+1) / 10;
     uint16_t bits_s1 = (width_s1+1) / 10;
     Approximate 1/10 to 205/2048  */
  uint16_t bits_s0 = ((width_s0+1) * (205 + CLK_ADJ)) >> 11;
  uint16_t bits_s1 = ((width_s1+1) * (205 + CLK_ADJ)) >> 11;
  uint16_t nlow;

  /* uint8_t byte_ofs = sbus_state.bit_ofs/12;
     uint8_t bit_ofs = sbus_state.bit_ofs%12;
     Approximate 1/12 to 683/8192  */
  uint8_t byte_ofs = (sbus_state.bit_ofs * 683) >> 13;
  uint8_t bit_ofs = sbus_state.bit_ofs - (byte_ofs * 12);

  if (byte_ofs >= SBUS_FRAME_SIZE)
    {
      // invalid sbus frame
      byte_ofs = SBUS_FRAME_SIZE - 1;
      goto reset;
    }

  if (bits_s0 == 0 || bits_s1 == 0)
    // invalid data
    goto reset;

  if (bits_s0+bit_ofs > 10)
    // invalid data as last two bits must be stop bits
    goto reset;

  // pull in the high bits
  sbus_state.bytes[byte_ofs] |= ((1U<<bits_s0)-1) << bit_ofs;
  sbus_state.bit_ofs += bits_s0;
  bit_ofs += bits_s0;

  // pull in the low bits
  nlow = bits_s1;
  if (nlow + bit_ofs > 12)
    nlow = 12 - bit_ofs;

  bits_s1 -= nlow;
  sbus_state.bit_ofs += nlow;

  if (sbus_state.bit_ofs == SBUS_FRAME_SIZE*12 && bits_s1 > 12)
    {
      // we have a full frame
      static uint8_t bytes[SBUS_FRAME_SIZE];
      uint8_t i;

      for (i=0; i<SBUS_FRAME_SIZE; i++)
	{
	  // get inverted data
	  uint16_t v = ~sbus_state.bytes[i];
	  // check start bit
	  if ((v & 1) != 0)
	    goto reset;
	  // check stop bits
	  if ((v & 0xC00) != 0xC00)
	    goto reset;

	  // check parity
	  uint8_t parity = 0, j;

	  for (j=1; j<=8; j++)
	    parity ^= (v & (1U<<j))?1:0;

	  if (parity != (v&0x200)>>9)
	    goto reset;

	  bytes[i] = ((v>>1) & 0xFF);
        }

      if (sbus_decode (bytes))
	{
	  // Assemble frame and send it to PCA9658
	  mixer (SBUS_NUM_CHANNELS);
	}
      goto reset;
    }
  else if (bits_s1 > 12)
    {
      // break
      goto reset;
    }
  return;
 reset:
  {
    for (int i = 0; i <= byte_ofs; i++)
      sbus_state.bytes[i] = 0;
    sbus_state.bit_ofs = 0;
  }
}

static void
main_loop (void)
{
  uint16_t dt;
  uint16_t dt_high, dt_low;

  dtbegin = 0;
  dtsize = 0;
  dt_high = dt_low = 0;

  
  // Initialize pwm
  pwm_init ();

  // Unmask all interrupts
  asm volatile ("cpsie      i" : : : "memory");

  for (;;)
    {
      asm volatile ("cpsid	i" : : : "memory");
      if (dtsize)
	{
	  dt = dtfifo[dtbegin];
	  dtbegin = (dtbegin + 1) % FIFOSZ;
	  --dtsize;
	  asm volatile ("cpsie      i" : : : "memory");
	}
      else
	{
	  asm volatile ("cpsie      i" : : : "memory");
	  // asm volatile ("wfi" : : : "memory");
	  continue;
	}

      if (dt & 1)
	dt_high = dt >> 1;
      else
	dt_low = dt >> 1;
      if (dt_low && dt_high)
	{
	  sbus_pulse (dt_high, dt_low);

	  dt_high = dt_low = 0;
	}
    }
}
