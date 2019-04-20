qemu-system-i386 -fda vmox.img -netdev user,id=usernet,io=0x300 -device ne2k_isa,irq=5,netdev=usernet
