#
# Copyright (C) Roger George Doss. All Rights Reserved.
#
#
#
# top-level makefile for OX 
#
#

# Make sure "all" is the default target
all:

AS      =       nasm
CC      =       gcc
LD      =       ld

libk_OBJS = \
	./libk/util.o \
	./libk/strtoul.o \
	./libk/vsprintk.o \
	./libk/string.o \
	./libk/bit.o \
	./libk/errno.o \
	./libk/printk.o \
	./libk/ultostr.o
OBJS += $(libk_OBJS)
$(libk_OBJS): libk; @true

mm_OBJS = \
	./mm/page.o \
	./mm/page_enable.o \
	./mm/malloc.o \
	./mm/dma.o \
	./mm/kmalloc.o
OBJS += $(mm_OBJS)
$(mm_OBJS): mm; @true

fs_OBJS = \
	./fs/file.o \
	./fs/link.o \
	./fs/krealpath.o \
	./fs/bitmap.o \
	./fs/inode.o \
	./fs/dev.o \
	./fs/paths.o \
	./fs/block.o \
	./fs/dir.o \
	./fs/init.o
OBJS += $(fs_OBJS)
$(fs_OBJS): fs; @true

drivers_OBJS = \
	./platform/i386/drivers/src/chara/rs_io.o \
	./platform/i386/drivers/src/chara/keybisr.o \
	./platform/i386/drivers/src/chara/keyboard.o \
	./platform/i386/drivers/src/chara/tty_io.o \
	./platform/i386/drivers/src/chara/serial.o \
	./platform/i386/drivers/src/chara/console.o \
	./platform/i386/drivers/src/chara/pit.o \
	./platform/i386/drivers/src/chara/ne2k.o \
	./platform/i386/drivers/src/chara/delay.o \
	./platform/i386/drivers/src/block/pio/pioread.o \
	./platform/i386/drivers/src/block/pio/piowrite.o \
	./platform/i386/drivers/src/block/pio/pioutil.o \
	./platform/i386/drivers/src/block/ide/ide.o
OBJS += $(drivers_OBJS)
$(drivers_OBJS): drivers; @true

kernel_OBJS = \
	./platform/i386/arch/interrupt.o \
	./platform/i386/arch/exception.o \
	./platform/i386/arch/idt.o \
	./platform/i386/arch/protected_mode.o \
	./platform/i386/arch/segment.o \
	./platform/i386/arch/main.o \
	./platform/i386/arch/asm_core/interrupt.o \
	./platform/i386/arch/asm_core/syscall.o \
	./platform/i386/arch/asm_core/exception.o \
	./platform/i386/arch/asm_core/util.o \
	./platform/i386/arch/asm_core/scheduler.o \
	./platform/i386/arch/asm_core/io.o \
	./platform/i386/arch/asm_core/atom.o \
	./platform/i386/arch/asm_core/start.o \
	./platform/i386/arch/io_req.o \
	./kernel/signal.o \
	./kernel/ox_main.o \
	./kernel/exit.o \
	./kernel/fork.o \
	./kernel/init.o \
	./kernel/exec.o \
	./kernel/syscall_tab.o \
	./kernel/scheduler.o \
	./kernel/def_int.o \
	./kernel/mktime.o \
	./kernel/ptrace.o \
	./kernel/misc.o \
	./kernel/panic.o \
	./kernel/halt.o \
	./kernel/process_queue.o \
	./kernel/syscall/syscall_0.o \
	./kernel/syscall/syscall_1.o \
	./kernel/syscall/syscall_2.o \
	./kernel/syscall/syscall_3.o \
	./kernel/syscall/syscall_4.o \
	./kernel/syscall/syscall_5.o
OBJS += $(kernel_OBJS)
$(kernel_OBJS): kernel; @true

#LOBJS = \
# 	./kernel/kernel.o     \
# 	./drivers/drivers.o   \
# 	./fs/fs.o     	      \
# 	./mm/mm.o	      \
# 	./net/libnet.a	      \
#	./libk/libk.o
#ARCHIVES= \
#	./kernel/kernel.a     \
#	./drivers/drivers.a   \
#	./fs/fs.a     	      \
#	./mm/mm.a	      \
#	./libk/libk.a

libnet_OBJS = ./net/libnet.a ./net/drivers.a
OBJS += $(libnet_OBJS)
$(libnet_OBJS): net; @true

.PHONY: all
all: vmox.img

vmox.img: boot vmox.boot
	./boot/mkboot vmox.boot $@

vmox.boot: boot vmox
	cat boot/s1 boot/s2 vmox > $@

vmox: $(OBJS) libc boot init
	#$(CC) -m32 -nostdinc -nostdlib -nostartfiles -nodefaultlibs -static $(TOBJS) -o vmox -Ttext 0x100000 --unresolved-symbols=ignore-all
	#$(LD) -melf_i386 -Ttext 0x100000 -Tdata 0x200000 $(OBJS) -o vmox
	#$(LD) -melf_i386 -Ttext 0x14000 $(OBJS) -o vmox --unresolved-symbols=ignore-all
	#$(LD) -melf_i386 -T ./ld/linker.ld $(TOBJS) -o vmox --unresolved-symbols=ignore-all
	$(CC) -m32 -ffreestanding -nostartfiles -nodefaultlibs -static $(OBJS) ./net/libnet.a -o vmox -Ttext 0x100000 # --unresolved-symbols=ignore-all
	strip vmox
	#$(LD) -melf_i386 -T nlinker.ld $(TOBJS) -o vmox --unresolved-symbols=ignore-all
	#strip vmox
	#$(LD) -melf_i386 -T linker.ld $(OBJS) -o vmox
	#$(CC) -m32 -nostdinc -nostdlib -nostartfiles -nodefaultlibs -static ./kernel/platform/main.o $(ARCHIVES) ./kernel/lowcore.a ./kernel/platform/asm.a ./drivers/chara/console.o ./kernel/platform/asm_core/io.o -o vmox -Ttext 0x100000
	#$(CC) -m32 -nostdinc -nostdlib -nostartfiles -nodefaultlibs -static $(OBJS) -o vmox -Ttext 0x100000
	#$(CC) -m32 -fPIE -pie -nostdinc -nostdlib -nostartfiles -nodefaultlibs -static vmox.o -o vmox -Ttext 0x100000
	#$(LD) -melf_i386 $(OBJS) -o vmox -Ttext 0x100000 # 0xe9300
	#$(CC) -m32 -nostdinc -nostdlib vmox.o -o vmox -Ttext 0x100000

.PHONY: bin_kernel __kernel
bin_kernel:	$(OBJS) libc boot init
	$(LD) -melf_i386 -T ./ld/flat_linker.ld $(OBJS) -o vmox # --unresolved-symbols=ignore-all
	objcopy ./vmox ./vmox.bin --target=binary --input-target=elf32-i386
	cp ./vmox ./vmox.exe
	cp ./vmox.bin ./vmox
	./boot/mkboot vmox.boot vmox.img

__kernel: $(OBJS)
	$(LD) -o vmox $(OBJS) -Ttext 0x100000
	strip vmox

.PHONY: install_old
install_old: all
	@if [ -e "./vmox.boot" ]; then \
		dd if=./vmox.boot of=/dev/fd0 bs=512; \
		exit; \
	fi; \
	echo "error during installation"

.PHONY: install
install: all
	@if [ -e "./vmox.boot" ]; then \
		dd if=./vmox.boot of=/dev/sdb bs=512; \
		exit; \
	fi; \
	echo "error during installation"


# Subdirectory targets
SUBDIRS = boot init libc net \
	  kernel drivers fs mm libk

include mk/subdirs.mk

kernel_clean: vmox_clean
.PHONY: vmox_clean
vmox_clean:
	rm -f vmox vmox.boot vmox.img

.PHONY: clean clean_all
clean_all: clean
clean:

#------------------
.PHONY: bin
bin:
	objcopy ./vmox ./vmox.bin --target=binary \
	--input-target=elf32-i386	
	mv ./vmox.bin ./vmox

#
# EOF
#
