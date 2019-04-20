/*

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*/
/*
 * @file:
 * 	keyboard.c
 *
 * @description:
 * 	Keyboard device driver.
 *
 * @author:
 *
 * 	Dr. Roger G. Doss, PhD
 *
 *      Some codes based on GazOS Operating System
 *      Copyright (C) 1999  Gareth Owen <gaz@athene.co.uk>
 *
 *      Distributed under GPL.
*/
#include <ox/defs.h>
#include <ox/types.h>
#include <ox/linkage.h>
#include <ox/error_rpt.h>
#include <ox/bool_t.h>

#include <platform/segment.h>
#include <platform/segment_defs.h>
#include <platform/segment_selectors.h>

#include <platform/interrupt.h>
#include <platform/interrupt_admin.h>
#include <platform/protected_mode_defs.h>
#include <platform/asm_core/interrupt.h>
#include <platform/asm_core/util.h>
#include <platform/gdt.h>
#include <platform/8259.h>

#include <asm_core/io.h>
#include <ox/types.h>

#include <drivers/chara/keyboard.h>
#include <drivers/chara/console.h>

#define LED_NUM_LOCK		2
#define LED_SCROLL_LOCK		1
#define LED_CAPS_LOCK		4

unsigned char led_status = 0;

/* keyboard_buffer stores every key typed */
unsigned char keyboard_buffer[255];
/* keyboard_buffer_size stores the number of keys in the buffer */
unsigned char keyboard_buffer_size = 0;

unsigned char control_keys = 0;

#define CK_SHIFT	1
#define	CK_ALT		2
#define CK_CTRL		4

unsigned int scan2ascii_table[][8] = 
{
/* 	ASCII -	Shift - Ctrl - 	Alt - 	Num - 	Caps - 	Shift Caps - 	Shift Num */
{  	0,	0,	0,	0,	0,	0,	0,		0},
{	0x1B,	0x1B,	0x1B,	0,	0x1B,	0x1B,	0x1B,		0x1B},
/* 1 -> 9 */
{	0x31,	0x21,	0,	0x7800,	0x31,	0x31,	0x21,		0x21},
{	0x32,	0x40,	0x0300,	0x7900,	0x32,	0x32,	0x40,		0x40},
{	0x33,	0x23,	0,	0x7A00, 0x33,	0x33,	0x23,		0x23},
{	0x34,	0x24,	0,	0x7B00, 0x34,	0x34,	0x24,		0x24},
{	0x35,	0x25,	0,	0x7C00,	0x35,	0x35,	0x25,		0x25},
{	0x36,	0x5E,	0x1E,	0x7D00, 0x36,	0x36,	0x5E,		0x5E},
{	0x37,	0x26,	0,	0x7E00,	0x37,	0x37,	0x26,		0x26},
{	0x38,	0x2A,	0,	0x7F00, 0x38,	0x38,	0x2A,		0x2A},
{	0x39,	0x28,	0,	0x8000, 0x39,	0x39,	0x28,		0x28},
{	0x30,	0x29,	0,	0x8100,	0x30,	0x30,	0x29,		0x29},
/* -, =, Bksp, Tab */
{	0x2D,	0x5F,	0x1F,	0x8200,	0x2D,	0x2D,	0x5F,		0x5F},
{	0x3D,	0x2B,	0,	0x8300,	0x3D,	0x3D,	0x2B,		0x2B},
{	0x08,	0x08,	0x7F,	0,	0x08,	0x08,	0x08,		0x08},
{	0x09,	0x0F00,	0,	0,	0x09,	0x09,	0x0F00,		0x0F00},
/*	QWERTYUIOP[] */
{	0x71,	0x51,	0x11,	0x1000,	0x71,	0x51,	0x71,		0x51},
{	0x77,	0x57,	0x17,	0x1100,	0x77,	0x57,	0x77,		0x57},
{	0x65,	0x45,	0x05,	0x1200,	0x65,	0x45,	0x65,		0x45},
{	0x72,	0x52,	0x12,	0x1300,	0x72,	0x52,	0x72,		0x52},
{	0x74,	0x54,	0x14,	0x1400,	0x74,	0x54,	0x74,		0x54},
{	0x79,	0x59,	0x19,	0x1500,	0x79,	0x59,	0x79,		0x59},
{	0x75,	0x55,	0x15,	0x1600,	0x75,	0x55,	0x75,		0x55},
{	0x69,	0x49,	0x09,	0x1700,	0x69,	0x49,	0x69,		0x49},
{	0x6F,	0x4F,	0x0F,	0x1800,	0x6F,	0x4F,	0x6F,		0x4F},
{	0x70,	0x50,	0x10,	0x1900,	0x70,	0x50,	0x70,		0x50},
{	0x5B,	0x7B,	0x1B,	0x0,	0x5B,	0x5B,	0x7B,		0x7B},
{	0x5D,	0x7D,	0x1D,	0,	0x5D,	0x5D,	0x7D,		0x7D},
/* ENTER, CTRL */
{	0x0A,	0x0A,	0x0D,	0,	0x0A,	0x0A,	0x0D,		0x0D},
{	0,	0,	0,	0,	0,	0,	0,		0},
/* ASDFGHJKL;'~ */
{	0x61,	0x41,	0x01,	0x1E00,	0x61,	0x41,	0x61,		0x41},
{	0x73,	0x53,	0x13,	0x1F00,	0x73,	0x53,	0x73,		0x53},
{	0x64,	0x44,	0x04,	0x2000,	0x64,	0x44,	0x64,		0x44},
{	0x66,	0x46,	0x06,	0x2100,	0x66,	0x46,	0x66,		0x46},
{	0x67,	0x47,	0x07,	0x2200,	0x67,	0x47,	0x67,		0x47},
{	0x68,	0x48,	0x08,	0x2300,	0x68,	0x48,	0x68,		0x48},
{	0x6A,	0x4A,	0x0A,	0x2400,	0x6A,	0x4A,	0x6A,		0x4A},
{	0x6B,	0x4B,	0x0B,	0x3500,	0x6B,	0x4B,	0x6B,		0x4B},
{	0x6C,	0x4C,	0x0C,	0x2600,	0x6C,	0x4C,	0x6C,		0x4C},
{	0x3B,	0x3A,	0,	0,	0x3B,	0x3B,	0x3A,		0x3A},
{	0x27,	0x22,	0,	0,	0x27,	0x27,	0x22,		0x22},
{	0x60,	0x7E,	0,	0,	0x60,	0x60,	0x7E,		0x7E},
/* Left Shift*/
{	0x2A,	0,	0,	0,	0,	0,	0,		0},
/* \ZXCVBNM,./ */
{	0x5C,	0x7C,	0x1C,	0,	0x5C,	0x5C,	0x7C,		0x7C},
{	0x7A,	0x5A,	0x1A,	0x2C00,	0x7A,	0x5A,	0x7A,		0x5A},
{	0x78,	0x58,	0x18,	0x2D00,	0x78,	0x58,	0x78,		0x58},
{	0x63,	0x43,	0x03,	0x2E00,	0x63,	0x43,	0x63,		0x43},
{	0x76,	0x56,	0x16,	0x2F00,	0x76,	0x56,	0x76,		0x56},
{	0x62,	0x42,	0x02,	0x3000,	0x62,	0x42,	0x62,		0x42},
{	0x6E,	0x4E,	0x0E,	0x3100,	0x6E,	0x4E,	0x6E,		0x4E},
{	0x6D,	0x4D,	0x0D,	0x3200,	0x6D,	0x4D,	0x6D,		0x4D},
{	0x2C,	0x3C,	0,	0,	0x2C,	0x2C,	0x3C,		0x3C},
{	0x2E,	0x3E,	0,	0,	0x2E,	0x2E,	0x3E,		0x3E},
{	0x2F,	0x3F,	0,	0,	0x2F,	0x2F,	0x3F,		0x3F},
/* Right Shift */
{	0,	0,	0,	0,	0,	0,	0,		0},
/* Print Screen */
{	0,	0,	0,	0,	0,	0,	0,		0},
/* Alt  */
{	0,	0,	0,	0,	0,	0,	0,		0},
/* Space */
{	0x20,	0x20,	0x20,	0,	0x20,	0x20,	0x20,		0x20},
/* Caps */
{	0,	0,	0,	0,	0,	0,	0,		0},
/* F1-F10 */
{	0x3B00,	0x5400,	0x5E00,	0x6800,	0x3B00,	0x3B00,	0x5400,		0x5400},
{	0x3C00,	0x5500,	0x5F00,	0x6900,	0x3C00,	0x3C00,	0x5500,		0x5500},
{	0x3D00,	0x5600,	0x6000,	0x6A00,	0x3D00,	0x3D00,	0x5600,		0x5600},
{	0x3E00,	0x5700,	0x6100,	0x6B00,	0x3E00,	0x3E00,	0x5700,		0x5700},
{	0x3F00,	0x5800,	0x6200,	0x6C00,	0x3F00,	0x3F00,	0x5800,		0x5800},
{	0x4000,	0x5900,	0x6300,	0x6D00,	0x4000,	0x4000,	0x5900,		0x5900},
{	0x4100,	0x5A00,	0x6400,	0x6E00,	0x4100,	0x4100,	0x5A00,		0x5A00},
{	0x4200,	0x5B00,	0x6500,	0x6F00,	0x4200,	0x4200,	0x5B00,		0x5B00},
{	0x4300,	0x5C00,	0x6600,	0x7000,	0x4300,	0x4300,	0x5C00,		0x5C00},
{	0x4400,	0x5D00,	0x6700,	0x7100,	0x4400,	0x4400,	0x5D00,		0x5D00},
/* Num Lock, Scrl Lock */
{	0,	0,	0,	0,	0,	0,	0,		0},
{	0,	0,	0,	0,	0,	0,	0,		0},
/* HOME, Up, Pgup, -kpad, left, center, right, +keypad, end, down, pgdn, ins, del */
{	0x4700,	0x37,	0x7700,	0,	0x37,	0x4700,	0x37,		0x4700},
{	0x4800,	0x38,	0,	0,	0x38,	0x4800,	0x38,		0x4800},
{	0x4900,	0x39,	0x8400,	0,	0x39,	0x4900,	0x39,		0x4900},
{	0x2D,	0x2D,	0,	0,	0x2D,	0x2D,	0x2D,		0x2D},
{	0x4B00,	0x34,	0x7300,	0,	0x34,	0x4B00,	0x34,		0x4B00},
{	0x4C00,	0x35,	0,	0,	0x35,	0x4C00,	0x35,		0x4C00},
{	0x4D00,	0x36,	0x7400,	0,	0x36,	0x4D00,	0x36,		0x4D00},
{	0x2B,	0x2B,	0,	0,	0x2B,	0x2B,	0x2B,		0x2B},
{	0x4F00,	0x31,	0x7500,	0,	0x31,	0x4F00,	0x31,		0x4F00},
{	0x5000,	0x32,	0,	0,	0x32,	0x5000,	0x32,		0x5000},
{	0x5100,	0x33,	0x7600,	0,	0x33,	0x5100,	0x33,		0x5100},
{	0x5200,	0x30,	0,	0,	0x30,	0x5200,	0x30,		0x5200},
{	0x5300,	0x2E,	0,	0,	0x2E,	0x5300,	0x2E,		0x5300}
};

void setleds()
{
	io_outb(0x60, 0xED);
	while(io_inb(0x64) & 2)
		/* wait. */;
	io_outb(0x60, led_status);
	while(io_inb(0x64) & 2)
		/* wait. */;
}

// 
// keyboard_enable:
//
// References pit_enable/pit_disable in pit.c.
// calls interrupt_install_handler(1, IRQ_EXCL, keyboard_handler,info);
// followed by irq_enable(1);
// Then a keyboard_disable is needed to call irq_disable(1)
// Our keyboard_handler is the actual handler.
// Like pit_enable, this should be called in ox_main()
//
irq_info_t keyboard_info()
{
    irq_info_t info;
    info.name  = "keyboard";
    info.descr = "OX Kernel Keyboard Driver";
    info.version.major = 1;
    info.version.minor = 0; 
    return info;
}

extern void keyb_ISR();

void keyboard_enable()
{
    int i = 0;
    asm_disable_interrupt();
#ifdef _USE_GAZ
    /* This is for debugging purposes. */
    set_vector(keyb_ISR, M_VEC+1, D_PRESENT + D_INT + D_DPL3, KERNEL_CS);
    enable_irq(1);
#else /* Default - use OX driver logic. */
    printk("initializing Keyboard driver version 1.0\n");
    interrupt_install_handler(1,
                              IRQ_EXCL,
                              keyboard_handler,
                              keyboard_info);
    printk("done initializing Keyboard driver version 1.0\n");
    /* irq_enable(1); */
    enable_irq(1);
#endif
    led_status = 0; /* All leds off */
    setleds();
}

void keyboard_disable()
{
    printk("disabling Keyboard driver version 1.0\n");
    /* irq_disable(1); */
    disable_irq(1);
    printk("done disabling Keyboard driver version 1.0\n");
}

// 
// keyboard_handler:
//
// This is the main handler, changed to use our io code.
// It should work, there is a trap to halt the cpu if
// ctrl+alt+del is hit.
//
irq_stat_t keyboard_handler(int irq)
{
	unsigned int key = io_inb(0x60);
	unsigned int key_ascii = 0;
	
#ifdef _DEBUG
    printk(" keyboard_handler := file %s line %d\n",__FILE__,__LINE__);
#endif
    // Control+ALT+DEL
	if((control_keys & (CK_CTRL+CK_ALT)) && key==0x53)
	{
		while(io_inb(0x64) & 2);
		io_outb(0x64, 0xFE);
		asm ("cli;hlt");
	}		
	
	/* 'LED Keys', ie, Scroll lock, Num lock, and Caps lock */
	if(key == 0x3A)	/* Caps Lock */
	{
		led_status ^= LED_CAPS_LOCK;
		setleds();
	}
	if(key == 0x45)	/* Num Lock */
	{
		led_status ^= LED_NUM_LOCK;
		setleds();
	}
	if(key == 0x46) /* Scroll Lock */
	{
		led_status ^= LED_SCROLL_LOCK;
		setleds();
	}


	if(key == 0x1D && !(control_keys & CK_CTRL))	/* Ctrl key */
		control_keys |= CK_CTRL;
	if(key == 0x80 + 0x1D)	/* Ctrl key depressed */
		control_keys &= (0xFF - CK_CTRL);
	if((key == 0x2A || key == 0x36) && !(control_keys & CK_SHIFT))	/* Shift key */
		control_keys |= CK_SHIFT;
	if((key == 0x80 + 0x2A) || (key == 0x80 + 0x36))	/* Shift key depressed */
		control_keys &= (0xFF - CK_SHIFT);
	if(key == 0x38 && (!(control_keys & CK_ALT)))
		control_keys |= CK_ALT;
	if(key == 0x80 + 0x38)
		control_keys &= (0xFF - CK_ALT);
		
	if((control_keys & CK_SHIFT) && (led_status & LED_CAPS_LOCK)) key_ascii = scan2ascii_table[key][6]; 
	else if(control_keys & CK_SHIFT) key_ascii = scan2ascii_table[key][1];
	else if(control_keys & CK_CTRL) key_ascii = scan2ascii_table[key][2];
	else if(control_keys & CK_ALT) key_ascii = scan2ascii_table[key][3];	
	else if((control_keys & CK_SHIFT) && (led_status & LED_NUM_LOCK)) key_ascii = scan2ascii_table[key][7];
	else if(led_status & LED_CAPS_LOCK) key_ascii = scan2ascii_table[key][5];
	else if(led_status & LED_NUM_LOCK) key_ascii = scan2ascii_table[key][4];
	else if(control_keys == 0) key_ascii = scan2ascii_table[key][0];

	if(key_ascii != 0)
	{
        if(key_ascii >= 0 && key_ascii <= 0xFF) {
		    keyboard_buffer[keyboard_buffer_size] = key_ascii;
		    keyboard_buffer_size++;
        }
#if 0
        // The following looks like code to handle multi-byte
        // input but this doesn't work with keyboard_getch() below
        // which only returns one byte at a time.
		if(key_ascii <= 0xFF)
		{
			keyboard_buffer[keyboard_buffer_size] = key_ascii;
			keyboard_buffer_size++;
		}
		else
		{
			keyboard_buffer[keyboard_buffer_size] = (key_ascii & 0xFF);
			keyboard_buffer[keyboard_buffer_size+1] = (key_ascii & 0xFF00);
			keyboard_buffer_size += 2;
		}
#endif
	}

#ifdef _DEBUG
    printk(" keyboard_handler := file %s line %d\n",__FILE__,__LINE__);
#endif
	io_outb(MASTER_PIC, EOI);
#ifdef _DEBUG
    printk(" keyboard_handler := file %s line %d\n",__FILE__,__LINE__);
#endif
    return IRQ_ENABLE;
}

unsigned char keyboard_getch()
{
	unsigned char ret_key=0;
	unsigned int loop=0;

	printk("$");
#ifdef _DEBUG
    printk("keyboard_buffer_size [%d]\n",keyboard_buffer_size);
#endif
	while(keyboard_buffer_size == 0)
            /* waiting */;
	ret_key = keyboard_buffer[0];
	keyboard_buffer_size--;
	
	for(loop=0; loop<254; loop++)
		keyboard_buffer[loop] = keyboard_buffer[loop+1];

#ifdef _DEBUG
    printk("ret_key [%d]\n",ret_key);
#endif
	return ret_key;
}

unsigned char keyboard_hit()
{
	if(keyboard_buffer_size == 0) return 0;
	
	return 1;
}	
