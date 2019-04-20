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
 *      ne2k.c
 *
 * @description:
 *      Device driver setup for ne2kif.c driver.
 *
 * @author:
 *      Dr. Roger G. Doss, PhD
 *
 */
#include <ox/defs.h>
#include <ox/types.h>
#include <ox/linkage.h>
#include <ox/error_rpt.h>
#include <ox/bool_t.h>

#include <platform/interrupt.h>
#include <platform/interrupt_admin.h>
#include <platform/protected_mode_defs.h>
#include <platform/asm_core/interrupt.h>
#include <platform/asm_core/util.h>
#include <drivers/chara/ne2k.h>

#include <asm_core/io.h>
#include <ox/types.h>

static unsigned long long tick = 0;
static int previrq = 0;

typedef int (*ne2k_isr_t)(void *arg);
static struct {
	ne2k_isr_t func;
	void *arg;
} ne2k_irq_handler;

irq_stat_t isr_wrapper(int irq)
{
//#ifdef _DEBUG
    printk("line %d file %s ne2k driver entry\n",__LINE__,__FILE__);
//#endif
    /* Call the actual ISR.  */
    ne2k_irq_handler.func(ne2k_irq_handler.arg);
    /* Keep the IRQ enabled. */
    return IRQ_ENABLE;
}

void
oxinterrupt_install(int irq, ne2k_isr_t handler, void *arg)
{
    printk("oxinterrupt_install\n");
	ne2k_irq_handler.func = handler;
	ne2k_irq_handler.arg = arg;
	interrupt_install_handler(irq, IRQ_EXCL, isr_wrapper, arg);
	enable_irq(irq);
}

/*
 * Call ne2k_enable() which sets up our entry in the Interrupt Vector
 * and enables the Ethernet IRQ in exclusive mode.
 *
 */
irq_info_t ne2k_info()
{
    irq_info_t info;
    info.name  = "ne2k";
    info.descr = "OX Kernel ne2k Driver";
    info.version.major = 1;
    info.version.minor = 0;
    return info;
}

void ne2k_enable(int irq)
{
    printk("initializing ne2k driver version 1.0\n");
    if(previrq) {
        printk("disabling irq=%d\n",previrq);
	    disable_irq(previrq);
    }

    /* FIXME hardcoded values must be removed! */
    u16_t iobase = 0x300;
    if (!irq)
	    irq = 5;
    u16_t membase = 16 * 1024;
    u16_t memsize = 16 * 1024;
    lwip_ne2k_add_interface(iobase, irq, membase, memsize);
    printk("done initializing ne2k driver version 1.0 at irq=%d\n", irq);
}

void ne2k_disable()
{
    printk("disabling ne2k driver version 1.0\n");
    /* irq_disable(0); */
    disable_irq(previrq);
    printk("done disabling ne2k driver version 1.0\n");
}
