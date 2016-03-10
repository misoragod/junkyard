# Makefile for extra mixer application

PROJECT = extmix

LDSCRIPT= extmix.ld
CSRC = extmix.c

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m0
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DHAVE_SYS_H -DFREE_STANDING -DENABLE_ISP -DLPC810
OPT   = -O3 -Os -g
LIBS  =

####################
include ./rules.mk

distclean: clean

