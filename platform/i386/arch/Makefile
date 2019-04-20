##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
##########################################################
# makefile for lowcore.o, the kernel's low-level
# machine-dependent primives
#
##########################################################

AS	=	/usr/bin/nasm
CC	=	/usr/bin/cc
LD	=	/usr/bin/ld

LDFLAGS =	-s -x -M
IFLAGS  =       -nostdinc -I. -I../include -I../.. -I../../../include -I../../include/ox \
		-I../../include/sys -I../../include/platform
#CFLAGS  =	$(IFLAGS) -ffreestanding -Wall -O -fstrength-reduce -fomit-frame-pointer -D_VM_DISABLED -m32 -fpic
CFLAGS  =	$(IFLAGS) -ffreestanding -Wall -m32 -D_VM_DISABLED -fomit-frame-pointer -D_USE_NEW_OFFSET
SUBDIRS = 	asm_core

.c.s:
	$(CC)	$(CFLAGS) \
	-ffreestanding -I$(IFLAGS) -S -o $*.s $<

.s.o:
	$(AS)	-f elf -o $*.o $<


OBJS = \
	asm.o            \
	idt.o \
	exception.o	 \
	interrupt.o      \
	protected_mode.o \
	segment.o	 \
	io_req.o	 \
	main.o

AOBJS = \
	asm.a            \
	idt.o \
	exception.o	 \
	interrupt.o      \
	protected_mode.o \
	segment.o	 \
	io_req.o	 \
	main.o

lowcore.a: $(AOBJS)
	  ar rv $@ $(AOBJS)
	  cp $@ ../../../kernel

lowcore.o: $(OBJS)
	  $(LD) -r -melf_i386 -o $@ $(OBJS)
	  cp $@ ../../../kernel

asm.o asm.a: $(SUBDIRS); @true

clean:
	rm -f $(OBJS) $(AOBJS) lowcore.[ao] ../../kernel/lowcore.[ao]

include ../../../mk/subdirs.mk
