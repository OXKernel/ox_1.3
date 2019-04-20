###################################################################################
# Copyright (C) Roger G. Doss. All Rights Reserved.
####################################################################################

#
# stage 1 and stage 2 loaders
OBJS = s1 s2
TDIR=../../..
AS=nasm

.PHONY: all clean
all: $(OBJS) mkboot

clean:
	rm -f $(OBJS)

s1: s1.s
	$(AS) $(BUILD_FLAGS) s1.s

s2: s2.s
	$(AS) $(BUILD_FLAGS) s2.s

s1ls:
	$(AS) -l s1_list s1.s

s2ls:
	$(AS) -l s2_list s2.s

s1h:
	$(AS) -e s1.s > s1.h

s2h:
	$(AS) -e s2.s > s2.h
