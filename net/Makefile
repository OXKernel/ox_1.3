TOPDIR = ..
LWIPDIR = lwip/src

include $(LWIPDIR)/Filelists.mk

CFLAGS += -nostdinc -isystem $(TOPDIR)/include \
	  -isystem $(TOPDIR)/platform/i386/include
CFLAGS += -iquote $(LWIPDIR)/include -iquote lwip-ox/include
CFLAGS += -isystem lwip-ox/include/syscompat

CFLAGS += -ffreestanding
CFLAGS += -march=i386 -m32 -Os

#lwip-ox/mbox.o
#lwip-ox/sem.o

lwip-objs += $(COREFILES:.c=.o)
lwip-objs += $(CORE4FILES:.c=.o)
lwip-objs += $(APIFILES:.c=.o)
lwip-objs += $(LWIPDIR)/netif/ethernet.o
lwip-objs += \
	lwip-ox/sys_i386.o \
	lwip-ox/mm.o \
	lwip-ox/mutex.o \
	lwip-ox/prot.o \
	lwip-ox/misc.o \
	lwip-ox/test_start.o

.PHONY: all clean
all: libnet.a drivers.a

clean:
	rm -f libnet.a $(lwip-objs)

SUBDIRS = drivers
drivers.a: drivers; @true

include $(TOPDIR)/mk/subdirs.mk

libnet.a: $(lwip-objs)
	$(AR) cru $@ $(lwip-objs)
