##########################################################
# Copyright (C)  Roger George Doss. All Rights Reserved.
##########################################################
#
# makefile for libc
#
##########################################################
AR       =  ar
ARFLAGS  =  rcs

SUBDIRS = std syscall

libc.a: $(SUBDIRS)
	$(AR) $(ARFLAGS) $@ $(SUBDIRS:=/*.o)

.PHONY: clean
clean:
	rm -f libc.a

include ../mk/subdirs.mk
