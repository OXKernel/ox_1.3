##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
##########################################################
# makefile for kernel 
#
##########################################################

AS	=	/usr/bin/nasm
CC	=	/usr/bin/cc
LD	=	/usr/bin/ld

LDFLAGS =	-s -x -M
IFLAGS  =       -nostdinc -I. -I.. -I../include -I../include/ox -I../include/sys -I../include/platform
IFLAGS +=	-I../net/include -I../net/include/ipv4
#CFLAGS  =	$(IFLAGS) -ffreestanding -Wall -O -fstrength-reduce -fomit-frame-pointer -m32 -fpic
CFLAGS  =	$(IFLAGS) -ffreestanding -Wall -m32 -nostdlib -nostartfiles -nodefaultlibs -fomit-frame-pointer

SUBDIRS = platform syscall

.c.s:
	$(CC)	$(CFLAGS) \
	-ffreestanding -I$(IFLAGS) -S -o $*.s $<

.s.o:
	$(AS)	-f elf -o $*.o $<


OBJS = \
	lowcore.o \
	syscall.o	\
	init.o      \
	exit.o		\
	exec.o      \
	fork.o		\
	mktime.o	\
	misc.o      \
	halt.o		\
	ox_main.o	\
	panic.o		\
	process_queue.o \
	ptrace.o	\
	scheduler.o	\
	signal.o	\
	def_int.o	\
	syscall_tab.o

AOBJS = \
	lowcore.a \
	syscall.a	\
	init.o		\
	exit.o		\
	exec.o      \
	fork.o		\
	mktime.o	\
	misc.o      \
	halt.o		\
	ox_main.o	\
	panic.o		\
	process_queue.o \
	ptrace.o	\
	scheduler.o	\
	signal.o	\
	def_int.o	\
	syscall_tab.o

all: $(OBJS)

kernel.a: $(OBJS)
	ar rv $@ $(AOBJS)

kernel.o: $(OBJS)
	$(LD) -r -melf_i386 -o $@ $(OBJS)

lowcore.o lowcore.a: platform; @true
syscall.o syscall.a: syscall; @true

.PHONY: clean
clean:
	rm -f $(OBJS) $(AOBJS) kernel.[ao]

include ../mk/subdirs.mk
