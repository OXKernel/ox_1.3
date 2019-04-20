##########################################################
#
# makefile for libnet
#
##########################################################
AR       =  ar
ARFLAGS  =  rcs

CC	=	/usr/bin/cc
IFLAGS  =	-nostdinc -I./ -I./src -I./include -I./include/ipv4 -I./include/posix -I../include -I../include/ox/ -I../include/ox/lib -I../include/platform
CFLAGS  =	$(IFLAGS) -MD -g -c -ffreestanding -Wall -m32 -fomit-frame-pointer

# FIXME this is not portable
SOURCES=$(wildcard src/*.c test/*.c)
OBJS=$(SOURCES:.c=.o)

.c.o:
	$(CC) $(CFLAGS) $< -o $@

libnet.a: $(OBJS)
	$(AR) $(ARFLAGS) libnet.a $(OBJS)

.PHONY: clean
clean:
	rm -f libnet.a $(OBJS) $(OBJS:.o=.d)

# Dependencies
-include $(OBJS:.o=.d)
