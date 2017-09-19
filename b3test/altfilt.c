/*
Copyright (c) 2017, DroneWorks Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Complementary filter for simple altitude estimation.

#include <stdio.h>
#include <math.h>

volatile float vertical_position;
volatile float vertical_velocity;

static float kp, kv;
static float ialt = 0.0f;

#define K_I 1000
#define I_GAIN (1.0f/K_I)
#define MAX_IALT 2000
void
altFinit(float accz_offset)
{
  float h = 0.8f;
  // filter gain
  kp = sqrtf(2)*h;
  kv = h*h;
  ialt = K_I*accz_offset;
}
  
void
altFupdate(float balt, float accz, float tdelta)
{
  float xp = vertical_position;
  float xv = vertical_velocity;
  float xdelta = balt - xp;
  ialt += xdelta;
  if (ialt > MAX_IALT)
    ialt = MAX_IALT;
  else if (ialt < -MAX_IALT)
    ialt = -MAX_IALT;
  accz = -accz + ialt * I_GAIN;
  float vdelta = accz*tdelta;
  xp = xp + xv*tdelta + (kp+0.5f*tdelta*kv)*tdelta*xdelta + 0.5f*tdelta*vdelta;
  xv = xv + kv*tdelta*xdelta + vdelta;
  vertical_position = xp;
  vertical_velocity = xv;
  //printf("balt: %7.3f accz: %7.3f alt: %7.3f vertical speed %7.3f dt %7.3f\n", balt, accz, xp, xv, tdelta);
}
