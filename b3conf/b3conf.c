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
 * Set parameter in NVS.
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

#include "b3packet.h"

#define	MYECHO_PORT 5790
#define MAXLINE sizeof(struct B3packet)

#define MAX_KEY_LENGTH 15

extern int errno;

static volatile sig_atomic_t interrupted;

void
parameter (int sockfd, int argc, char *argv[])
{
  int n, clilen;
  struct sockaddr_in cli_addr;
  struct B3packet pkt;
  char last_parameter[B3SIZE-(4+2)];
  union { float f; int32_t i; uint8_t bytes[sizeof(float)];} val;

  last_parameter[0] = 0;
  
  for (;;)
    {
      clilen = sizeof(cli_addr);
      bzero (&cli_addr, clilen);
      n = recvfrom(sockfd, &pkt, MAXLINE, 0,
		   (struct sockaddr *) &cli_addr, &clilen);
      if (n < 0) {
	if (interrupted > 0)
	  {
	    printf ("interrupted\n");
	    exit (0);
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

      if (pkt.tos != TOS_PARAM && !last_parameter[0])
	;
      else if (pkt.tos == TOS_PARAM
	       && strcmp(last_parameter, &pkt.data[0]) == 0)
	{
	  memcpy (val.bytes, &pkt.data[B3SIZE-6], 4);
	  if (pkt.data[B3SIZE-2])
	    printf("param: %s failed to get/set/erase\n", last_parameter);
	  else if (pkt.data[B3SIZE-1] == 2)
	    printf("param: %s erased successfully\n", last_parameter);
	  else if (last_parameter[0] == '%')
	    printf("param: %s value: %7.3f\n", last_parameter, val.f);
	  else
	    printf("param: %s value: %d\n", last_parameter, val.i);
	}
      else
	continue;

      pkt.tos = TOS_PARAM;
      bzero (&pkt.data[0], B3SIZE);
      if (argc-- == 0)
	exit(0);

      char *t, *s = *argv++;
      size_t n, m = MAX_KEY_LENGTH;
      t = strchr(s, '=');
      if (t)
	{
	  n = t-s;
	  if (*(t+1) != '\0')
	    {
	      if (*s == '%')
		val.f = atof(t+1);
	      else
		val.i = atoi(t+1);
	      memcpy (&pkt.data[B3SIZE-6], val.bytes, 4);
	      // set value
	      pkt.data[B3SIZE-1] = 1;
	    }
	  else
	    // delete value
	    pkt.data[B3SIZE-1] = 2;
	}
      else
	{
	  n = strlen(s);
	  // request value
	  pkt.data[B3SIZE-1] = 0;
	}
      if (n > m)
	printf("key string is too long. Will be trancated.\n");
      n = (n > m) ? m : n;
      memcpy(&pkt.data[0], s, n);
      pkt.data[n] = 0;
      strcpy(last_parameter, &pkt.data[0]);
      if (pkt.data[B3SIZE-1] == 1)
	{
	  if (pkt.data[0] == '%')
	    printf("set %s to %f\n", last_parameter, val.f);
	  else
	    printf("set %s to %d\n", last_parameter, val.i);
	}
      else if (pkt.data[B3SIZE-1] == 2)
	printf("erase %s\n", last_parameter);
      else
	printf("request %s\n", last_parameter);

      n = sizeof(pkt);
      if (sendto (sockfd, &pkt, n, 0, (struct sockaddr *)&cli_addr,
		  clilen) != n)
	{
	  fprintf (stderr, "server: sendto error");
	  exit (1);
	}
      if (interrupted > 0)
	{
	  printf ("interrupted\n");
	  exit (0);
	}
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

  while (--argc > 0)
    {
      ++argv;
      if ((*argv)[0] == '-')
	{
	  for (s = argv[0]+1; *s != '\0'; s++)
	    switch (*s)
	      {
	      case 'h':
		printf ("Usage:\n"
			"  b3conf [OPTION...] PARAM_NAME[=VALUE] ...\n"
			"    If VALUE is given, set/create PARAM with VALUE.\n"
			"    PARAM_NAME is string of which length <= 15 and\n"
			"    if it starts with '%%', it's handled as float.\n"
			"    PARAM_NAME= can erase that PARAM.\n"
			"\nHelp Options:\n"
			"  -h	Show help options\n"
			);
		exit (1);

	      default:
		err_quit("illegal option. Try -h");
	      }
	}
      else
	break;
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

  if (bind (sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
      fprintf (stderr, "server: can't bind local address");
      exit (1);
    }

  option = 1;
  ioctl (sockfd, FIONBIO, &option);

  parameter (sockfd, argc, argv);
  /* NOTREACHED */
}
