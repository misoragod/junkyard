# Makefile for RC input conveter application

PROJECT = x2ppmsum

LDSCRIPT= x2ppmsum.ld
CSRC = x2ppmsum.c

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m0
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DHAVE_SYS_H -DFREE_STANDING -DENABLE_ISP
OPT   = -O3 -Os -g
LIBS  =

####################
include ./rules.mk

distclean: clean

