##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
##########################################################
# makefile for memory manager
#
##########################################################

AS	=	/usr/bin/nasm
CC	=	/usr/bin/cc
LD	=	/usr/bin/ld

LDFLAGS =	-s -x -M
IFLAGS  =       -nostdinc -I. -I../include -I../include/ox -I../include/sys -I../include/platform -I../include/mm
#CFLAGS  =	$(IFLAGS) -ffreestanding -Wall -O -fstrength-reduce -fomit-frame-pointer -m32 -fpic
CFLAGS  =	$(IFLAGS) -ffreestanding -Wall -m32 -nostdlib -nostartfiles -nodefaultlibs -fomit-frame-pointer -D_ENABLE_PAGING

.s.o:
	$(AS) -f elf -o $*.o $<

OBJS = \
	page.o \
	page_enable.o \
	dma.o \
	kmalloc.o
OBJS += malloc.o

all: mm.o mm.a
mm.a:	$(OBJS)
	ar rv mm.a $(OBJS)

mm.o:	$(OBJS)
	$(LD) -r -melf_i386 -o mm.o $(OBJS)

clean:
	rm -f core $(OBJS) mm.[ao]

.PHONY: clean all
