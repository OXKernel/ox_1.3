##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
# makefile for syscall
#
##########################################################
AR       =  ar
AS       =  as
CC       =  gcc
LD       =  ld
ARFLAGS  =  rcs
LDFLAGS  =  -s -x
IFLAGS   =  -nostdinc -I../../include
CFLAGS	 =  $(IFLAGS) -MD -D_POSIX_SOURCE -Wall -O -fstrength-reduce \
	    -fomit-frame-pointer -finline-functions -m32 -fpic

OBJS = \
	string.o \
	strerror.o

all:   $(OBJS)

clean:
	rm -f $(OBJS) $(OBJS:.o=.d)

### Dependencies:
-include $(OBJS:.o=.d)
