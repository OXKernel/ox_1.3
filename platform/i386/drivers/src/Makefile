##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
##########################################################
# makefile for drivers
#
##########################################################

AS	=	/usr/bin/nasm
CC	=	/usr/bin/cc
LD	=	/usr/bin/ld

LDFLAGS =	-s -x -M
TOP = ../../../..
IFLAGS  =       -nostdinc -I. -I.. -I../include \
		-I$(TOP)/include \
		-I$(TOP)/include/platform \
		-I$(TOP)/net/include \
		-I$(TOP)/net/include/ipv4 \
		-I../include/ox -I../include/sys -I../include/platform
#CFLAGS  =	$(IFLAGS) -ffreestanding -Wall -O -fstrength-reduce -fomit-frame-pointer -m32 -fpic
CFLAGS  =	$(IFLAGS) -MD -ffreestanding -Wall -m32 -fomit-frame-pointer
CHAR    = 	./chara
BLOCK   = 	./block

.c.s:
	$(CC)	$(CFLAGS) \
	-ffreestanding -I$(IFLAGS) -S -o $*.s $<
.s.o:
	$(AS)	-f elf -o $*.o $<

#
# This should be generated with a configuration
# script when we are able to support various
# hardware configurations.
#
OBJS = \
	$(CHAR)/keybisr.o \
	$(CHAR)/keyboard.o  \
	$(CHAR)/rs_io.o     \
	$(CHAR)/console.o   \
	$(CHAR)/serial.o    \
	$(CHAR)/tty_io.o    \
	$(CHAR)/pit.o \
	$(CHAR)/delay.o \
	$(CHAR)/ne2k.o \
	$(BLOCK)/pio/pioread.o \
	$(BLOCK)/pio/piowrite.o \
	$(BLOCK)/pio/pioutil.o \
	$(BLOCK)/ide/ide.o

drivers.a: $(OBJS)
	  ar rv $@ $(OBJS)

drivers.o: $(OBJS)
	  $(LD) -r -melf_i386 -o $@ $(OBJS)

.PHONY: clean
clean:
	rm -f $(OBJS) drivers.[ao] $(OBJS:.o=.d)

# Dependencies
-include $(OBJS:.o=.d)
