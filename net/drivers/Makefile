TOPDIR = ../..
LWIPDIR = ../lwip/src

CFLAGS += -nostdinc -isystem $(TOPDIR)/include \
	  -isystem $(TOPDIR)/platform/i386/include
CFLAGS += -iquote $(LWIPDIR)/include -iquote ../lwip-ox/include
CFLAGS += -MD -march=i386 -m32 -Os -ffreestanding

objs += ne2000.o lwip-wrapper-ne2000.o

.PHONY: all clean
all: ../drivers.a
clean:
	rm -f ../drivers.a $(objs)

../drivers.a: $(objs)
	$(AR) cru $@ $(objs)
