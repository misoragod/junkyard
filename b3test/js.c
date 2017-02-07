/*
Copyright (c) 2017, DroneWorks Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JSDEV "/dev/input/js0"

int *jsaxis;
char *jsbutton;
pthread_mutex_t jsmutex;

void *
js_thread (void *p)
{
  int fd;
  uint8_t naxis, nbutton;

  fd = open (JSDEV, O_RDONLY);
  if (fd < 0)
    {
      fprintf (stderr, "can't open %s\n", JSDEV);
      return NULL;
    }

  ioctl (fd, JSIOCGAXES, &naxis);
  ioctl (fd, JSIOCGBUTTONS, &nbutton);

  jsaxis = (int *) calloc (naxis, sizeof (int));
  jsbutton = (char *) calloc (nbutton, sizeof (char));

  fcntl (fd, F_SETFL, O_NONBLOCK);

  while (1)
    {
      struct js_event js;
      int n = read (fd, &js, sizeof(struct js_event));

      if (n != sizeof(struct js_event))
	{
	  usleep(10000);
	  continue;
	}

      pthread_mutex_lock (&jsmutex);
      switch (js.type & ~JS_EVENT_INIT)
	{
	case JS_EVENT_AXIS:
	  //printf ("js axis %d %d\n", js.number, js.value);
	  if((int)js.number < naxis)
	    jsaxis[(int)js.number]= js.value;
	  break;
	case JS_EVENT_BUTTON:
	  //printf ("js button %d %d\n", js.number, js.value);
	  if((int)js.number < nbutton)
	    jsbutton[(int)js.number]= js.value;
	  break;
	}
      pthread_mutex_unlock (&jsmutex);

      usleep (10000);
    }

  return NULL;
}
