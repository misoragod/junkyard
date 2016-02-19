/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* A simple TCP wrapper for ublox uart interface.  Also behaves as server
   for GPS daemon via UDP.  */

#define _GNU_SOURCE

#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <poll.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define NFDS 2
static struct pollfd fds[NFDS];
#define BUFSIZE 512

/* Default ports.  */
#define GPSD_DATAPORT 5000
#define FC_GPS_DATAPORT 5780
static int port = GPSD_DATAPORT;
static int fcport = FC_GPS_DATAPORT;

static int
millis()
{
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

/* Baudrate iterator.  */
static int
nextbaud (void)
{
  static const int baudrates[] =
    { 9600, 19200, 38400, 57600, 115200, 230400 };
  static int idx = 0;

  idx = (idx + 1) % (sizeof (baudrates) / sizeof (baudrates[0]));
  return baudrates[idx];
}

/* Set tty speed.  */
static void
set_speed (int fd, unsigned int baudrate)
{
  struct termios t;
  memset (&t, 0, sizeof(t));

  tcgetattr (fd, &t);
  cfsetspeed (&t, baudrate);
  tcsetattr (fd, TCSANOW, &t);
}

/* Set tty to raw mode.  */
static void
set_raw (int fd, int flow)
{
  struct termios t;
  memset (&t, 0, sizeof(t));

  tcgetattr (fd, &t);
  t.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
  t.c_iflag &= ~(BRKINT|ICRNL|INPCK|ISTRIP|IXON|IXOFF);
  t.c_oflag &= ~OPOST;
  t.c_cflag &= ~CRTSCTS;
  if (flow)
    t.c_cflag |= CRTSCTS;
  tcsetattr (fd, TCSANOW, &t);
}

/* Initialization and main loop.  */
int
main (int argc, char *argv[])
{
  int rtn;
  int ch;
  int opt;
  int debug = 0;
  int dump_raw = 0;
  int flow = 1;
  int default_baudrate = 9600;
  int uartfd;
  int sock, fcsock;
  struct sockaddr_in addr, fcaddr;
  int connected = 0;
  int last_mark;
  char *device_path;
  unsigned char buf[BUFSIZE];

  while ((opt = getopt (argc, argv, "b:Ddfhnt:u:")) != -1)
    {
      switch (opt)
	{
	case 'b':
	  default_baudrate = atoi (optarg);
	  break;
	case 'D':
	  dump_raw = 1;
	  break;
	case 'd':
	  debug = 1;
	  break;
	case 'n':
	  flow = 0;
	  break;
	case 't':
	  fcport = atoi (optarg);
	  break;
	case 'u':
	  port = atoi (optarg);
	  break;
	case 'h':
	  fprintf (stderr,
		   "Usage: %s [-d] [-D] [-t tcp-port] [-u udp-port] "
		   "uart-device-path\n"
" : A simple tcp wrapper of ublox connected via uart.  Uart tty output from\n"
" ublox will be copied to tcp and udp ports.  The default tcp and udp port\n"
" is 5780 and 5000, respectively, of localhost(127.0.0.1).  TCP inputs are\n"
" directed to uart and UDP inputs are ignored.  UDP port is intended to be\n"
" used as the source for GPS daemon and TCP port is intended to be used with\n"
" ardupilot.  UDP and TCP ports can be specified with -g and -u option.\n"
" -n disables hard flow control on tty.  -D enables raw dump for test.\n",
		   argv[0]);
	  return 0;
	default: /* '?' */
	  fprintf (stderr,
		   "Usage: %s [-d] [-D] [-n] [-u tcp-port] [-u udp-port] "
		   "uart-device-path\n",
		   argv[0]);
	  return EXIT_FAILURE;
	}
    }

  if (optind >= argc)
    {
      fprintf (stderr, "uart-device-path required\n");
      return EXIT_FAILURE;
    }

  device_path = argv[optind];

  if (debug)
    fprintf (stderr, "UDP port: %d (GPSD), TCP port: %d (FC), tty: %s\n",
	     port, fcport, device_path);

  uartfd = open (device_path, O_RDWR | O_CLOEXEC);
  if (uartfd < 0)
    {
      fprintf (stderr, "Failed to open UART device %s - %s\n",
	       device_path, strerror (errno));
      return EXIT_FAILURE;
    }
  set_raw (uartfd, flow);
  set_speed (uartfd, default_baudrate);

  sock = socket (AF_INET, SOCK_DGRAM, 0);
  if (sock < 0)
    {
      fprintf (stderr, "Failed to open socket - %s\n", strerror (errno));
      return EXIT_FAILURE;
    }

  addr.sin_family = AF_INET;
  addr.sin_port = htons (port);
  addr.sin_addr.s_addr = inet_addr ("127.0.0.1");

  fcsock = socket (AF_INET, SOCK_STREAM, 0);
  if (fcsock < 0)
    {
      fprintf (stderr, "Failed to open socket - %s\n", strerror (errno));
      return EXIT_FAILURE;
    }

  fcaddr.sin_family = AF_INET;
  fcaddr.sin_port = htons (fcport);
  fcaddr.sin_addr.s_addr = inet_addr ("127.0.0.1");

  rtn = connect (fcsock, (struct sockaddr *) &fcaddr, sizeof (fcaddr));
  if (rtn == 0)
    {
      connected = 1;
      if (debug)
	fprintf (stderr, "Connected to %d\n", fcport);
    }

  fds[0].fd = uartfd;
  fds[0].events = POLLIN;
  fds[1].fd = fcsock;
  fds[1].events = POLLIN | POLLRDHUP;

  last_mark = millis ();

  for (;;)
    {
      fds[0].revents = 0;
      fds[1].revents = 0;
      rtn = poll (fds, (connected ? NFDS : 1), -1);
      if (rtn == 0)
	{
	  /* No inputs from ublox.  Try next baudrate.  */
	  int baud = nextbaud ();

	  if (debug)
	    fprintf (stderr, "Try baudrate %d\n", baud);
	  set_speed (uartfd, baud);
	}
      if (rtn < 0)
	{
	  fprintf (stderr, "Failed to poll - %s\n", strerror (errno));
	  return EXIT_FAILURE;
	}
      if (connected && (fds[1].revents & POLLRDHUP) != 0)
	{
	  connected = 0;
	  close (fcsock);
	  if (debug)
	    fprintf (stderr, "Disconnected from %d\n", fcport);
	  fcsock = socket (AF_INET, SOCK_STREAM, 0);
	  if (fcsock < 0)
	    {
	      fprintf (stderr, "Failed to open socket - %s\n",
		       strerror (errno));
	      return EXIT_FAILURE;
	    }
	}
      if ((fds[0].revents & POLLIN) != 0)
	{
	  int n = read (uartfd, buf, BUFSIZE);
	  if (n <= 0)
	    {
	      fprintf (stderr, "Failed to read uart - %s\n", strerror (errno));
	      return EXIT_FAILURE;
	    }
	  if (dump_raw)
	    {
	      int j;
	      for (j = 0; j < n; j++)
		fprintf (stderr, "%02x ", buf[j]);
	      fprintf (stderr, "\n");
	    }
	  /* Check text and binary marker.  */
	  if (!memchr (buf, '$', n) && !memchr (buf, 0xb5, n))
	    {
	      if (millis () - last_mark > 2000)
		{
		  /* Couldn't find marker 2 sec.  Maybe baudrate mismatch.
		     Try next baudrate.  */
		  int baud = nextbaud ();

		  if (debug)
		    fprintf (stderr, "Try baudrate %d\n", baud);
		  set_speed (uartfd, baud);
		  last_mark = millis ();
		}
	    }
	  else
	    last_mark = millis ();
	  if (!connected)
	    {
	      if (0 == connect (fcsock, (struct sockaddr *) &fcaddr,
				sizeof (fcaddr)))
		{
		  connected = 1;
		  if (debug)
		    fprintf (stderr, "Connected to %d\n", fcport);
		}
	    }
	  if (connected)
	    {
	      rtn = send (fcsock, buf, n, 0);
	      if (rtn < 0)
		{
		  if (errno != ECONNRESET)
		    {
		      fprintf (stderr, "Faile to send -%s\n",
			   strerror (errno));
		    }
		  else
		    {
		      connected = 0;
		      	  close (fcsock);
			  if (debug)
			    fprintf (stderr, "Disconnected from %d\n", fcport);
			  fcsock = socket (AF_INET, SOCK_STREAM, 0);
			  if (fcsock < 0)
			    {
			      fprintf (stderr, "Failed to open socket - %s\n",
				       strerror (errno));
			      return EXIT_FAILURE;
			    }
		    }
		}
	      else if (n != rtn)
		{
		  fprintf (stderr, "Only %d of %d -%s\n", rtn, n,
			   strerror (errno));
		}
	    }
	  sendto (sock, buf, n, 0, (struct sockaddr *) &addr, sizeof (addr));
	}
      if (connected && (fds[1].revents & POLLIN) != 0)
	{
	  rtn = read (fcsock, &ch, 1);
	  if (rtn <= 0)
	    {
	      fprintf (stderr, "Failed to read %d - %s\n", fcport,
		       strerror (errno));
	      continue;
	    }
	  write (uartfd, &ch, 1);
	}
    }
	      
  return 0;
}

  
