#ifndef _OX_IO_LINUX_H
#define _OX_IO_LINUX_H

#define outb outb_p
#define outw outw_p
#define outl outl_p
static void outb_p(unsigned short port, unsigned char value){
	__asm__("out %%al,%%dx\n\t" :: "d"(port), "a"(value));
}
static void outw_p(unsigned short port, unsigned short value){
	__asm__("outw %%ax,%%dx\n\t" :: "d"(port), "a"(value));
}
static void outl_p(unsigned short port, unsigned int value){
	__asm__("outl %%eax,%%dx\n\t" :: "d"(port), "a"(value));
}

#define inb inb_p
#define inw inw_p
#define inl inl_p
static unsigned char inb_p(unsigned short port){
	__asm__("in %%dx,%%al\n\t" :: "d"(port));
}
static unsigned short inw_p(unsigned short port){
	__asm__("inw %%dx,%%ax\n\t" :: "d"(port));
}
static unsigned int inl_p(unsigned short port){
	__asm__("inl %%dx,%%eax\n\t" :: "d"(port));
}

#endif
