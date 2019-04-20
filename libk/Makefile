##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
# makefile for lib
# kernel library
#
##########################################################
AR       =  ar
AS       =  as
CC       =  cc
LD       =  ld
ARFLAGS  =  rcs
LDFLAGS  =  -s -x
IFLAGS   =  -nostdinc -I../include -I../include/platform/
#CFLAGS	 =  $(IFLAGS) -D_POSIX_SOURCE -Wall -O -fstrength-reduce \
#	    -fomit-frame-pointer -finline-functions -m32 -fpic
CFLAGS	 =  $(IFLAGS) -MD -D_POSIX_SOURCE -Wall -m32 -ffreestanding -fomit-frame-pointer

OBJS = \
	bit.o	   \
	errno.o	   \
	string.o   \
	strtoul.o  \
	ultostr.o  \
	util.o     \
	printk.o   \
	vsprintk.o

libk.o: $(OBJS)
	$(LD) -r -melf_i386 -o $@ $(OBJS)

libk.a: $(OBJS)
	ar rv $@ $(OBJS)

.PHONY: all clean
all: libk.a libk.o

clean:
	rm -f core $(OBJS) libk.[ao] $(OBJS:.o=.d)

### Dependencies:
-include $(OBJS:.o=.d)
