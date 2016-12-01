# Makefile for PPM to SPI

PROJECT = ppm2spi

LDSCRIPT= ppm2spi.ld
CSRC = ppm2spi.c

###################################
CROSS = arm-none-eabi-
CC   = $(CROSS)gcc
LD   = $(CROSS)gcc
OBJCOPY   = $(CROSS)objcopy

MCU   = cortex-m0
CWARN = -Wall -Wextra -Wstrict-prototypes
DEFS  = -DHAVE_SYS_H -DFREE_STANDING
OPT   = -O3 -Os -g
LIBS  =

####################
include ./rules.mk

distclean: clean

