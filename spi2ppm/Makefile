# Makefile for SPI to PPM

PROJECT = spi2ppm

LDSCRIPT= spi2ppm.ld
CSRC = spi2ppm.c

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

