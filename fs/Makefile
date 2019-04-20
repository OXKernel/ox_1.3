##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
##########################################################
# makefile for filesystem
#
##########################################################

AS	=	/usr/bin/nasm
CC	=	/usr/bin/cc
LD	=	/usr/bin/ld

LDFLAGS =	-s -x -M
IFLAGS  =       -nostdinc -I. -I.. -I../include -I../include/ox -I../include/sys -I../include/platform -I../include/ox/fs
#CFLAGS  =	$(IFLAGS) -Wall -O -ffreestanding -fstrength-reduce -fomit-frame-pointer -m32 -fpic
CFLAGS  =	$(IFLAGS) -Wall -ffreestanding -m32 -nostdlib -nostartfiles -nodefaultlibs -fomit-frame-pointer

OBJS = \
	paths.o \
	bitmap.o \
	dev.o \
	block.o \
	krealpath.o \
	inode.o \
	file.o \
	dir.o \
	link.o \
	init.o

fs.a: $(OBJS)
	ar rv $@ $(OBJS)

fs.o: $(OBJS)
	$(LD) -r -melf_i386 -o $@ $(OBJS)

.PHONY: clean
clean:
	rm -f $(OBJS) fs.[ao]
