/*
Copyright (c) 2017, DroneWorks Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Example of hachidori server using UDP protocol.
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>

#include <pthread.h>

#include "b3packet.h"

#include "MadgwickAHRS.h"

#define	MYECHO_PORT 5790
#define MAXLINE sizeof(struct B3packet)

extern int errno;

#define SHOW_RAW_ACC     (1<<0)
#define SHOW_RAW_GYRO    (1<<1)
#define SHOW_RAW_MAG     (1<<2)
#define SHOW_RAW_PRESS   (1<<3)
#define SHOW_RAW_BATT    (1<<4)
#define SHOW_RAW_RANGE   (1<<5)
#define SHOW_QUATERNION  (1<<6)
#define SHOW_ACCZ        (1<<7)
#define SHOW_STREAM      (1<<8)
#define SHOW_STICK       (1<<9)

static int show_flags;
static float filter_gain = 0.2f;

// Green
static int rgb_led = 2;

/*
 * Read the contents of the socket and write each line back to
 * the sender.
 */

extern int *jsaxis;
extern char *jsbutton;
extern pthread_mutex_t jsmutex;
extern void *js_thread (void *);

#define ROLL_AXIS	0
#define PITCH_AXIS	1
#define YAW_AXIS	3
#define THROTTLE_AXIS	2
#define TRIG_BUTTON	0
#define STICK_MIN	900
#define STICK_LOW	1100
#define STICK_HIGH	1900
#define TILT_LIMIT	100

static uint16_t
get_stick(void)
{
  int val;
  if (!jsaxis)
    return 0;
  pthread_mutex_lock (&jsmutex);
  val = jsaxis[THROTTLE_AXIS];
  pthread_mutex_unlock (&jsmutex);
  //printf ("js axis %d\n", val);

  return STICK_LOW + ((STICK_HIGH - STICK_LOW)*((int)val + 0x8000)/0xffff);
}

static void get_tilt(int *r, int *p, int *y)
{
  int rv, pv, yv;
  if (!jsaxis)
    {
      *r = *p = *y = 0;
      return;
    }
  pthread_mutex_lock (&jsmutex);
  rv = jsaxis[ROLL_AXIS];
  pv = jsaxis[PITCH_AXIS];
  yv = jsaxis[YAW_AXIS];
  pthread_mutex_unlock (&jsmutex);

  rv /= 128;
  pv /= 128;
  yv /= 128;
  // limit them
  if (rv < -TILT_LIMIT)
    rv = -TILT_LIMIT;
  else if (rv > TILT_LIMIT)
    rv = TILT_LIMIT;
  *r = rv;
  if (pv < -TILT_LIMIT)
    pv = -TILT_LIMIT;
  else if (pv > TILT_LIMIT)
    pv = TILT_LIMIT;
  *p = pv;
  if (yv < -TILT_LIMIT)
    yv = -TILT_LIMIT;
  else if (yv > TILT_LIMIT)
    yv = TILT_LIMIT;
  *y = yv;

  //printf ("js axis tilt r:%d p:%d y:%d\n", rv, pv, yv);
  return;
}

static bool
get_button(void)
{
  char val;
  if (!jsbutton)
    return false;

  pthread_mutex_lock (&jsmutex);
  val = jsbutton[TRIG_BUTTON];
  pthread_mutex_unlock (&jsmutex);
  return val ? true : false;
}

static int
mymillis (void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

// Compute conjugate i.e. Q^-1 *P Q and return it via *P where *P
// is pure quaternion
static void
qconjugate(float *p0, float *p1, float *p2, float *p3)
{
  float b = *p1, c = *p2, d = *p3;

  *p1 = q2*(d*q0 + c*q1 - b*q2) - q3*(c*q0 - d*q1 + b*q3)
    + q0*(b*q0 + d*q2 - c*q3) -  q1*(- b*q1 - c*q2 - d*q3);
  *p2 = -q1*(d*q0 + c*q1 - b*q2) + q0*(c*q0 - d*q1 + b*q3)
    + q3*(b*q0 + d*q2 - c*q3) - q2*(- b*q1 - c*q2 - d*q3); 
  *p3 = q0*(d*q0 + c*q1 - b*q2) + q1*(c*q0 - d*q1 + b*q3)
    - q2*(b*q0 + d*q2 - c*q3) - q3*(- b*q1 - c*q2 - d*q3);
}

#if 0
static inline float dclip(float x)
{
  const float h = 0.5f;
  if (x < -0.5f) {
    return 1.0f;
  } else if (x > 0.5f) {
    return 1.0f - h;
  }
  return 1.0f - (x + 0.5f)*h;
}

#endif
static float cur_mem[20];

static float sma_filter(float x, float mem[], size_t n)
{
  static int idx = 0;
  float sum = 0;
  mem[idx] = x;
  idx = (idx + 1) % n;
  for (int i = 0; i < n; i++)
    sum += mem[i];
  return sum / n;
}

static bool path_through = false;
static bool no_spin = true;

static void set_width (uint8_t *p, float width)
{
  if (no_spin)
    width = STICK_LOW;
      
  uint16_t w;
  if (width > STICK_HIGH)
    w = STICK_HIGH;
  else if (width < STICK_LOW)
    w = STICK_LOW;
  else
    w = (uint16_t) width;
  p[0] = w >> 8;
  p[1] = w & 0xff;
  //printf("w %d\n", w);
}

#define GRAVITY_MSS 9.80665f
#define GEPSILON 0.05f

#define NUM_MOTORS 4
#define NUM_CHANNELS 4
#define MIN_WIDTH ((float) STICK_MIN)
#define PGAIN 0.8f
#define DGAIN 32.00f
#define ATT 0.1f
#define BCOUNT 10
#define IGAIN 0.01f
#define DELTA 0.05f
#define HEPSILON 0.02f
#define DRATE (1.0f - 0.9965403f)
#define DDGAIN 50.0f

static float base_adjust[4];
static float last_adjust[4];
static float last_width[4];

// ~10ms i.e. 100Hz
#define PWM_PERIOD 10
#define MAXADJ 250
#define ROLL 1.0f
#define PITCH 1.0f
#define YAW 45.0f

static volatile sig_atomic_t interrupted;

void
paracode (int sockfd)
{
  int n, clilen;
  struct sockaddr_in cli_addr;
  struct B3packet pkt;

  float gx, gy, gz;
  float ax, ay, az;
  float mx = 0.0f, my = 0.0f, mz = 0.0f;

  float press;

  int count = 0;

  int inverted = 0;

  bool liftup = true;

  float P, Pm;
  float Q = 0.000001f;
  float R = 0.01f;
  float K;
  float xhat = 0.0f, xhatm;
  float v = 0.0f;
  float Az = 0.0f;

  // time
  int tlast = 0, tnow;
  //
  float dp[4], di[4], dd[4];
  float qp0 = 1.0f, qp1 = 0.0f, qp2 = 0.0f, qp3 = 0.0f;
  //
  float stick_last = STICK_LOW;

  dp[0] = dp[1] = dp[2] = dp[3] = 0.0f;
  dd[0] = dd[1] = dd[2] = dd[3] = 0.0f;
  di[0] = di[1] = di[2] = di[3] = 0.0f;

  for (;;)
    {
      clilen = sizeof(cli_addr);
      bzero (&cli_addr, clilen);
      n = recvfrom(sockfd, &pkt, MAXLINE, 0,
		   (struct sockaddr *) &cli_addr, &clilen);
      if (n < 0) {
	if (interrupted > 0)
	  {
	    // Fake packet
	    pkt.head = 0xb3;
	    pkt.tos = 0xff;
	  }
	else if (errno == EAGAIN)
	  continue;
	else
	  {
	    fprintf (stderr, "server: recvfrom error");
	    exit (1);
	  }
      }

      // printf ("%d %d\n", sizeof(cli_addr), clilen);
      // printf ("%d bytes %02x\n", n, pkt.head);
      if (pkt.head != 0xb3)
	continue;
      if (pkt.tos == TOS_IMU)
	{
	  union { float f; uint8_t bytes[sizeof(float)];} uax, uay, uaz;
	  union { float f; uint8_t bytes[sizeof(float)];} ugx, ugy, ugz;
	  memcpy (uax.bytes, &pkt.data[0], 4);
	  memcpy (uay.bytes, &pkt.data[4], 4);
	  memcpy (uaz.bytes, &pkt.data[8], 4);
	  memcpy (ugx.bytes, &pkt.data[12], 4);
	  memcpy (ugy.bytes, &pkt.data[16], 4);
	  memcpy (ugz.bytes, &pkt.data[20], 4);
	  ax = uax.f; ay = uay.f; az = uaz.f;
	  gx = ugx.f; gy = ugy.f; gz = ugz.f;
	  if (show_flags & SHOW_RAW_ACC)
	    printf("ax: %f ay: %f az: %f\n", ax, ay, az);
	  if (show_flags & SHOW_RAW_GYRO)
	    printf("gx: %f gy: %f gz: %f\n", gx, gy, gz);
	  MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
	}
      else if (pkt.tos == TOS_MAG)
	{
	  union { float f; uint8_t bytes[sizeof(float)];} umx, umy, umz;
	  memcpy (umx.bytes, &pkt.data[0], 4);
	  memcpy (umy.bytes, &pkt.data[4], 4);
	  memcpy (umz.bytes, &pkt.data[8], 4);
	  mx = umx.f; my = umy.f; mz = umz.f;
	  if (show_flags & SHOW_RAW_MAG)
	    printf("mx: %f my: %f mz: %f\n", mx, my, mz);
	  // Use large gain in early stage so as to accelarate converge
	  beta = (count < 1000) ? 2.0f : filter_gain;
	  MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
	}
      else if (pkt.tos == TOS_BARO)
	{
	  union { float f; uint8_t bytes[sizeof(float)];} up, ut, uh;
	  memcpy (up.bytes, &pkt.data[0], 4);
	  memcpy (ut.bytes, &pkt.data[4], 4);
	  memcpy (uh.bytes, &pkt.data[8], 4);
	  if (show_flags & SHOW_RAW_PRESS)
	    printf("p: %f t: %f h: %f\n", up.f, ut.f, uh.f);
	  press = up.f;
	}
      else if (pkt.tos == TOS_BAT)
	{
	  union { float f; uint8_t bytes[sizeof(float)];} uvb, ucur, uvopt;
	  memcpy (uvb.bytes, &pkt.data[0], 4);
	  memcpy (ucur.bytes, &pkt.data[4], 4);
	  memcpy (uvopt.bytes, &pkt.data[12], 4); // optional
	  if (show_flags & SHOW_RAW_BATT)
	    printf("vbat: %f curr: %f (vopt: %f)\n", uvb.f, ucur.f, uvopt.f);
	}
      else if (pkt.tos == TOS_RANGE)
	{
	  union { float f; uint8_t bytes[sizeof(float)];} d;
	  memcpy (d.bytes, &pkt.data[0], 4);
	  if (show_flags & SHOW_RAW_RANGE)
	    printf("range: %f\n", d.f);
	}
      else if (pkt.tos == TOS_GPS)
	{
	  if (show_flags & SHOW_STREAM)
	    {
	      int len = pkt.data[0];
	      if (len <= 30)
		{
		  char *s = &pkt.data[1];
		  while (len--)
		    putchar(*s++);
		}
	    }
	}

      float a = 0.0f, b = ax, c = ay, d = az;
      qconjugate(&a, &b, &c, &d);
      //d = sma_filter(d, az_mem, AZ_SMA_LEN);
      Az = d - GRAVITY_MSS;
      //printf ("vertical acc %7.3f\n", d);
      xhatm = xhat;
      Pm = P + Q;

      K = Pm/(Pm + R);
      xhat = xhatm + K*(Az - xhatm);
      P = (1 - K)*Pm;
      if (show_flags & SHOW_ACCZ)
	printf ("accz %7.3f\n", xhat + GRAVITY_MSS);
#if 0
      if (az < -GRAVITY_MSS * 0.6)
	inverted++;
#endif
      if (show_flags & SHOW_QUATERNION)
	printf ("q0 %7.3f q1 %7.3f q2 %7.3f q3 %7.3f\n", q0, q1, q2, q3);
      //printf ("should R-up %7.3f H-up %7.3f\r", -(q0*q1+q3*q2), q0*q2-q3*q1);

      liftup = !get_button ();
      //printf ("button %d\n", liftup);

      if ((count % PWM_PERIOD) == 0)
	{
	  float rup, hup, ydelta, d[NUM_MOTORS];
	  int i;

	  float stick;
	  if (path_through)
	    stick = (float) get_stick ();
	  else if (liftup)
	    stick = 0.9*stick_last + 0.1*((float) get_stick ());
	  else if (xhat < 0.1f)
	    stick = stick_last;
	  else
	    {
	      stick = (1-DRATE)*stick_last + DRATE*MIN_WIDTH;
	      if (stick < STICK_LOW)
		stick = STICK_LOW;
	    }

	  stick_last = stick;
	  if (show_flags & SHOW_STICK)
	    printf ("stick: %7.5f\n", stick);

	  // These are linear approximations which would be enough for
	  // our purpose.
	  rup = -(q0*q1+q3*q2);
	  hup = q0*q2-q3*q1;

	  // yaw change
	  float qDot0, qDot1, qDot2, qDot3;
	  qDot0 = q0 - qp0;
	  qDot1 = q1 - qp1;
	  qDot2 = q2 - qp2;
	  qDot3 = q3 - qp3;
	  if (qp0 == 1.0f)
	    ydelta = 0;
	  else
	    {
	      // yaw speed = 2 * qDot * qBar
	      ydelta = -q3*qDot0 - q2*qDot1 + q1*qDot2 + q0*qDot3;
	    }
	  //printf ("yaw %7.5f\n", ydelta);

	  qp0 = q0;
	  qp1 = q1;
	  qp2 = q2;
	  qp3 = q3;

#if 0
	  printf ("rup %7.5f hup %7.5f yaw %7.5f accz %7.5f\r",
		  rup, hup, ydelta, xhat);
#endif
	  d[0] =  ROLL*rup + PITCH*hup + YAW*ydelta; // M1 right head
	  d[1] = -ROLL*rup - PITCH*hup + YAW*ydelta; // M2 left  tail
	  d[2] = -ROLL*rup + PITCH*hup - YAW*ydelta; // M3 left  head
	  d[3] =  ROLL*rup - PITCH*hup - YAW*ydelta; // M4 right tail

	  for (i = 0; i < NUM_CHANNELS; i++)
	    {
	      float adj = base_adjust[i];
	      if (i < NUM_MOTORS)
		{
		  float pv = d[i];
		  float dv = dp[i]-d[i];
		  dp[i] = d[i];
		  dd[i] = dv;
		  adj = (1-ATT)*(pv*PGAIN-dv*DGAIN)*200.0f + ATT*adj;
		  if (adj > MAXADJ)
		    adj = MAXADJ;
		  else if (adj < -MAXADJ)
		    adj = -MAXADJ;
		  last_adjust[i] = adj;
		  if ((count % (PWM_PERIOD * BCOUNT)) == 0)
		    base_adjust[i] = adj;
		}
	    }
#if 0
	  printf ("d[0] %7.3f dd[0] %7.3f\n", d[0], dd[0]);
#endif
#if 0
	  printf ("a0 %7.3f a1 %7.3f a2 %7.3f a3 %7.3f\n",
		  last_adjust[0], last_adjust[1], last_adjust[2], last_adjust[3]);
#endif
#if 0
	  printf ("a0 %7.3f a1 %7.3f a2 %7.3f a3 %7.3f hup %7.3f rup %7.4f yaw %7.3f\n",
		  last_adjust[0],
		  last_adjust[1],
		  last_adjust[2],
		  last_adjust[3],
		  hup, rup,
		  ydelta);
#endif

	  pkt.tos = TOS_PWM;
	  bzero (&pkt.data[0], B3SIZE);
	  if (interrupted > 0)
	    {
	      no_spin = true;
	      interrupted++;
	    }

	  // RGB LED channels are 13-15
	  pkt.data[2*13+1] = (rgb_led & 4) ? 255 : 0;
	  pkt.data[2*14+1] = (rgb_led & 2) ? 255 : 0;
	  pkt.data[2*15+1] = (rgb_led & 1) ? 255 : 0;

	  if (inverted > 20)
	    {
	      set_width (&pkt.data[0], STICK_LOW);
	      set_width (&pkt.data[2], STICK_LOW);
	      set_width (&pkt.data[4], STICK_LOW);
	      set_width (&pkt.data[6], STICK_LOW);
	    }
	  else
	    {
	      int mt[4];
	      int rt, pt, yt;
	      get_tilt (&rt, &pt, &yt);
	      mt[0] =  -rt + pt + yt; // M1 right head
	      mt[1] =   rt - pt + yt; // M2 left  tail
	      mt[2] =   rt + pt - yt; // M3 left  head
	      mt[3] =  -rt - pt - yt; // M4 right tail

	      if (path_through)
		{
		  last_width[0] = stick;
		  last_width[1] = stick;
		  last_width[2] = stick;
		  last_width[3] = stick;
		}
	      else
		{
		  last_width[0] = stick + mt[0] + last_adjust[0];
		  last_width[1] = stick + mt[1] + last_adjust[1];
		  last_width[2] = stick + mt[2] + last_adjust[2];
		  last_width[3] = stick + mt[3] + last_adjust[3];
		}

	      set_width (&pkt.data[0], last_width[0]);
	      set_width (&pkt.data[2], last_width[1]);
	      set_width (&pkt.data[4], last_width[2]);
	      set_width (&pkt.data[6], last_width[3]);
	    }
	  if (sendto (sockfd, &pkt, 34, 0, (struct sockaddr *)&cli_addr,
		      clilen) != n)
	    {
	      fprintf (stderr, "server: sendto error");
	      exit (1);
	    }
	  if (interrupted > 1)
	    {
	      printf ("interrupted\n");
	      exit (0);
	    }
	}

      count++;
    }
}

static void
sigint_handler (int p_signame)
{
  interrupted = 1;
}

void
err_quit (const char *msg)
{
  fprintf (stderr, "%s\n", msg);
  exit (1);
}

int
main (int argc, char *argv[])
{
  int sockfd;
  int option;
  char *s;
  struct sockaddr_in serv_addr;
  struct servent *sp;
  pthread_t pthread;

  if (signal (SIGINT, sigint_handler) == SIG_ERR)
    err_quit("can't register sigint handler");

  while (--argc > 0 && (*++argv)[0] == '-')
    for (s = argv[0]+1; *s != '\0'; s++)
      switch (*s)
	{
	case 'h':
	  printf ("Usage:\n"
		  "  bb3test [OPTION...]\n"
		  "\nHelp Options:\n"
		  "  -h	Show help options\n"
		  "\nReport options:\n"
		  "  -A Show raw acceralometer data\n"
		  "  -G Show raw gyroscope data\n"
		  "  -M Show raw magnetometer data\n"
		  "  -P Show raw barometer data\n"
		  "  -B Show raw battery data\n"
		  "  -R Show raw range data\n"
		  "  -S Show current stick value\n"
		  "  -U Show stream data\n"
		  "  -Q Show computed quaternion\n"
		  "  -Z Show computed vertical accelaration\n"
		  "\nMotor options:\n"
		  "  -s Enable motor spin\n"
		  "  -p Pass through joystick to pwm\n"
		  "  -l RGB_VALUE    Set RGB LED value (0-7)\n"
		  "\nFilter options:\n"
		  "  -g FLOAT_VALUE  Set filter gain to FLOAT_VALUE\n"
		  );
	  exit (1);

	case 'g':	/* next arg is gain */
	  if (--argc <=0)
	    err_quit("-g requires another argument");
	  filter_gain = atof(*++argv);
	  break;

	case 'l':	/* next arg is bit set for RGB led */
	  if (--argc <=0)
	    err_quit("-l requires another argument");
	  rgb_led = atoi(*++argv);
	  break;

	case 'p':
	  path_through = true;
	  break;

	case 's':
	  no_spin = false;
	  break;

	case 'A':
	  show_flags |= SHOW_RAW_ACC;
	  break;

	case 'G':
	  show_flags |= SHOW_RAW_GYRO;
	  break;

	case 'M':
	  show_flags |= SHOW_RAW_MAG;
	  break;

	case 'P':
	  show_flags |= SHOW_RAW_PRESS;
	  break;

	case 'B':
	  show_flags |= SHOW_RAW_BATT;
	  break;

	case 'R':
	  show_flags |= SHOW_RAW_RANGE;
	  break;

	case 'S':
	  show_flags |= SHOW_STICK;
	  break;

	case 'U':
	  show_flags |= SHOW_STREAM;
	  break;

	case 'Q':
	  show_flags |= SHOW_QUATERNION;
	  break;

	case 'Z':
	  show_flags |= SHOW_ACCZ;
	  break;

	default:
	  err_quit("illegal option. Try -h");
	}

  /*
   * Open a UDP socket (an Internet datagram socket).
   */

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
      fprintf (stderr, "server: can't open datagram socket");
      exit (1);
    }

  option = 1;
  setsockopt (sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

  bzero ((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl (INADDR_ANY);
  serv_addr.sin_port = htons (MYECHO_PORT);

  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
      fprintf (stderr, "server: can't bind local address");
      exit (1);
    }

  option = 1;
  ioctl (sockfd, FIONBIO, &option);
 
  pthread_mutex_init(&jsmutex, NULL);
  pthread_create (&pthread, NULL, js_thread, NULL);
  
  paracode (sockfd);
  /* NOTREACHED */
}
