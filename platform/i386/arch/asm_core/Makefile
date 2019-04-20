##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
##########################################################
# makefile for asm_core, the kernel's lowest level
# primitives, written in assembler
#
##########################################################

AS	=	/usr/bin/nasm
CC	=	/usr/bin/cc
LD	=	/usr/bin/ld

LDFLAGS =	-s -x -M
IFLAGS  =       -nostdinc -I. -I.. -I../include -I../include/ox -I../include/sys -I../include/platform
#CFLAGS  =	$(IFLAGS) -ffreestanding -Wall -O -fstrength-reduce -fomit-frame-pointer -fpic
CFLAGS  =	$(IFLAGS) -ffreestanding -Wall -m32 -nostartfiles -nodefaultlibs -fomit-frame-pointer

.c.s:
	$(CC)	$(CFLAGS) \
	-ffreestanding -I$(IFLAGS) -S -o $*.s $<

.s.o:
	$(AS)	-f elf -o $*.o $<


OBJS = \
	atom.o	    \
	scheduler.o \
	exception.o \
	interrupt.o \
	io.o        \
	start.o     \
	syscall.o   \
	util.o

asm.a:	$(OBJS)
	ar rv $@ $(OBJS)
	cp $@ ..

asm.o:  $(OBJS)
	$(LD) -r -melf_i386 -o $@ $(OBJS)
	cp $@ ..

.PHONY: clean
clean:
	rm -f $(OBJS) asm.[ao] ../asm.[ao]
