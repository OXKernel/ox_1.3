#define WAIT() for(;;) /* WAIT */;

#include <string.h>
#include <asm_core/io.h>
#include <sys/errno.h>
#include <ox/mm/malloc.h>
#include <ox/lib/printk.h>
#include <ox/sleep.h>
#include <lwip/sys.h>
typedef unsigned int u32_t;
#include <arch/sys_arch.h>	// For mbox.
#include "ne2000.h"

extern struct netif *ne2k_if_netif;

// TODO:
//  [0] Find out how the dev->recieve method is implemented and where it is
//      assigned. Note that we should download the sources and grep through them.
//  [1] Replace all the OS Specific code with ours.
//  [2] See what needs to be done in the data structurs area of Porter, some
//      of the structs are just dummies.
//
// REFERENCES:
//      http://www.jbox.dk/sanos/source/sys/dev/ne2000.c.html
//
#include <errno.h>

#ifndef NULL
#define NULL (char *)0x0
#endif

#define NE2K_TXTIMEOUT 100

void
debug(int line, char *file)
{
	printk("line %d file %s\n", line, file);
	for (;;)		/* WAIT */
		;
}

//
// ne2000.c
//
// NE2000 network card driver
//
// Copyright (C) 2002 Michael Ringgaard. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
// 1. Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.  
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.  
// 3. Neither the name of the project nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
// SUCH DAMAGE.
// 

//#include <os/krnl.h>
typedef unsigned long size_t;
//
// Page 0 register offsets
//

#define NE_P0_CR        0x00	// Command Register

#define NE_P0_CLDA0     0x01	// Current Local DMA Addr low (read)
#define NE_P0_PSTART    0x01	// Page Start register (write)

#define NE_P0_CLDA1     0x02	// Current Local DMA Addr high (read)
#define NE_P0_PSTOP     0x02	// Page Stop register (write)

#define NE_P0_BNRY      0x03	// Boundary Pointer

#define NE_P0_TSR       0x04	// Transmit Status Register (read)
#define NE_P0_TPSR      0x04	// Transmit Page Start (write)

#define NE_P0_NCR       0x05	// Number of Collisions Reg (read)
#define NE_P0_TBCR0     0x05	// Transmit Byte count, low (write)

#define NE_P0_FIFO      0x06	// FIFO register (read)
#define NE_P0_TBCR1     0x06	// Transmit Byte count, high (write)

#define NE_P0_ISR       0x07	// Interrupt Status Register

#define NE_P0_CRDA0     0x08	// Current Remote DMA Addr low (read)
#define NE_P0_RSAR0     0x08	// Remote Start Address low (write)

#define NE_P0_CRDA1     0x09	// Current Remote DMA Addr high (read)
#define NE_P0_RSAR1     0x09	// Remote Start Address high (write)

#define NE_P0_RBCR0     0x0A	// Remote Byte Count low (write)

#define NE_P0_RBCR1     0x0B	// Remote Byte Count high (write)

#define NE_P0_RSR       0x0C	// Receive Status (read)
#define NE_P0_RCR       0x0C	// Receive Configuration Reg (write)

#define NE_P0_CNTR0     0x0D	// Frame alignment error counter (read)
#define NE_P0_TCR       0x0D	// Transmit Configuration Reg (write)

#define NE_P0_CNTR1     0x0E	// CRC error counter (read)
#define NE_P0_DCR       0x0E	// Data Configuration Reg (write)

#define NE_P0_CNTR2     0x0F	// Missed packet counter (read)
#define NE_P0_IMR       0x0F	// Interrupt Mask Register (write)

//
// Page 1 register offsets
//

#define NE_P1_CR        0x00	// Command Register
#define NE_P1_PAR0      0x01	// Physical Address Register 0
#define NE_P1_PAR1      0x02	// Physical Address Register 1
#define NE_P1_PAR2      0x03	// Physical Address Register 2
#define NE_P1_PAR3      0x04	// Physical Address Register 3
#define NE_P1_PAR4      0x05	// Physical Address Register 4
#define NE_P1_PAR5      0x06	// Physical Address Register 5
#define NE_P1_CURR      0x07	// Current RX ring-buffer page
#define NE_P1_MAR0      0x08	// Multicast Address Register 0
#define NE_P1_MAR1      0x09	// Multicast Address Register 1
#define NE_P1_MAR2      0x0A	// Multicast Address Register 2
#define NE_P1_MAR3      0x0B	// Multicast Address Register 3
#define NE_P1_MAR4      0x0C	// Multicast Address Register 4
#define NE_P1_MAR5      0x0D	// Multicast Address Register 5
#define NE_P1_MAR6      0x0E	// Multicast Address Register 6
#define NE_P1_MAR7      0x0F	// Multicast Address Register 7

//
// Page 2 register offsets
//

#define NE_P2_CR        0x00	// Command Register
#define NE_P2_PSTART    0x01	// Page Start (read)
#define NE_P2_CLDA0     0x01	// Current Local DMA Addr 0 (write)
#define NE_P2_PSTOP     0x02	// Page Stop (read)
#define NE_P2_CLDA1     0x02	// Current Local DMA Addr 1 (write)
#define NE_P2_RNPP      0x03	// Remote Next Packet Pointer
#define NE_P2_TPSR      0x04	// Transmit Page Start (read)
#define NE_P2_LNPP      0x05	// Local Next Packet Pointer
#define NE_P2_ACU       0x06	// Address Counter Upper
#define NE_P2_ACL       0x07	// Address Counter Lower
#define NE_P2_RCR       0x0C	// Receive Configuration Register (read)
#define NE_P2_TCR       0x0D	// Transmit Configuration Register (read)
#define NE_P2_DCR       0x0E	// Data Configuration Register (read)
#define NE_P2_IMR       0x0F	// Interrupt Mask Register (read)

//
// Command Register (CR)
//

#define NE_CR_STP       0x01	// Stop
#define NE_CR_STA       0x02	// Start
#define NE_CR_TXP       0x04	// Transmit Packet
#define NE_CR_RD0       0x08	// Remote DMA Command 0
#define NE_CR_RD1       0x10	// Remote DMA Command 1
#define NE_CR_RD2       0x20	// Remote DMA Command 2
#define NE_CR_PS0       0x40	// Page Select 0
#define NE_CR_PS1       0x80	// Page Select 1

#define NE_CR_PAGE_0    0x00	// Select Page 0
#define NE_CR_PAGE_1    0x40	// Select Page 1
#define NE_CR_PAGE_2    0x80	// Select Page 2

//
// Interrupt Status Register (ISR)
//

#define NE_ISR_PRX      0x01	// Packet Received
#define NE_ISR_PTX      0x02	// Packet Transmitted
#define NE_ISR_RXE      0x04	// Receive Error
#define NE_ISR_TXE      0x08	// Transmission Error
#define NE_ISR_OVW      0x10	// Overwrite
#define NE_ISR_CNT      0x20	// Counter Overflow
#define NE_ISR_RDC      0x40	// Remote Data Complete
#define NE_ISR_RST      0x80	// Reset status

//
// Interrupt Mask Register (IMR)

#define NE_IMR_PRXE     0x01	// Packet Received Interrupt Enable
#define NE_IMR_PTXE     0x02	// Packet Transmit Interrupt Enable
#define NE_IMR_RXEE     0x04	// Receive Error Interrupt Enable
#define NE_IMR_TXEE     0x08	// Transmit Error Interrupt Enable
#define NE_IMR_OVWE     0x10	// Overwrite Error Interrupt Enable
#define NE_IMR_CNTE     0x20	// Counter Overflow Interrupt Enable
#define NE_IMR_RDCE     0x40	// Remote DMA Complete Interrupt Enable

//
// Data Configuration Register (DCR)

#define NE_DCR_WTS      0x01	// Word Transfer Select
#define NE_DCR_BOS      0x02	// Byte Order Select
#define NE_DCR_LAS      0x04	// Long Address Select
#define NE_DCR_LS       0x08	// Loopback Select
#define NE_DCR_AR       0x10	// Auto-initialize Remote
#define NE_DCR_FT0      0x20	// FIFO Threshold Select 0
#define NE_DCR_FT1      0x40	// FIFO Threshold Select 1

//
// Transmit Configuration Register (TCR)

#define NE_TCR_CRC      0x01	// Inhibit CRC
#define NE_TCR_LB0      0x02	// Loopback Control 0
#define NE_TCR_LB1      0x04	// Loopback Control 1
#define NE_TCR_ATD      0x08	// Auto Transmit Disable
#define NE_TCR_OFST     0x10	// Collision Offset Enable

//
// Transmit Status Register (TSR)

#define NE_TSR_PTX      0x01	// Packet Transmitted
#define NE_TSR_COL      0x04	// Transmit Collided
#define NE_TSR_ABT      0x08	// Transmit Aborted
#define NE_TSR_CRS      0x10	// Carrier Sense Lost
#define NE_TSR_FU       0x20	// FIFO Underrun
#define NE_TSR_CDH      0x40	// CD Heartbeat
#define NE_TSR_OWC      0x80	// Out of Window Collision

//
// Receiver Configuration Register (RCR)
//

#define NE_RCR_SEP      0x01	// Save Errored Packets
#define NE_RCR_AR       0x02	// Accept Runt packet
#define NE_RCR_AB       0x04	// Accept Broadcast
#define NE_RCR_AM       0x08	// Accept Multicast
#define NE_RCR_PRO      0x10	// Promiscuous Physical
#define NE_RCR_MON      0x20	// Monitor Mode

//
// Receiver Status Register (RSR)

#define NE_RSR_PRX      0x01	// Packet Received Intact
#define NE_RSR_CRC      0x02	// CRC Error
#define NE_RSR_FAE      0x04	// Frame Alignment Error
#define NE_RSR_FO       0x08	// FIFO Overrun
#define NE_RSR_MPA      0x10	// Missed Packet
#define NE_RSR_PHY      0x20	// Physical Address
#define NE_RSR_DIS      0x40	// Receiver Disabled
#define NE_RSR_DFR      0x80	// Deferring

//
// Novell NE2000
//

#define NE_NOVELL_NIC_OFFSET    0x00
#define NE_NOVELL_ASIC_OFFSET   0x10

#define NE_NOVELL_DATA          0x00
#define NE_NOVELL_RESET         0x0F

#define NE_PAGE_SIZE            256	// Size of RAM pages in bytes
#define NE_TXBUF_SIZE           6	// Size of TX buffer in pages
#define NE_TX_BUFERS            2	// Number of transmit buffers

#define NE_TIMEOUT              10000
#define NE_TXTIMEOUT            30000

//
// Receive ring descriptor
//

int debugging = 0;

struct ether_msg {
	struct pbuf *p;
	struct netif *netif;
};

struct recv_ring_desc {
	unsigned char rsr;	// Receiver status
	unsigned char next_pkt;	// Pointer to next packet
	unsigned short count;	// Bytes in packet (length + 4)
};

//
// NE2000 NIC status
//

// Porter {
#include <lwip/pbuf.h>
#if 0
struct pbuf {			// TODO: OS Specific.
	struct pbuf *next;

	unsigned short flags;
	unsigned short ref;
	void *payload;

	int tot_len;		// Total length of buffer + additionally chained buffers.
	int len;		// Length of this buffer.
	int size;		// Allocated size of buffer
};
#endif

#define printf printk
struct stats_proto {
	unsigned long xmit;	// Transmitted packets
	unsigned long rexmit;	// Retransmitted packets
	unsigned long recv;	// Received packets
	unsigned long fw;	// Forwarded packets
	unsigned long drop;	// Dropped packets
	unsigned long chkerr;	// Checksum error
	unsigned long lenerr;	// Invalid length error
	unsigned long memerr;	// Out of memory error
	unsigned long rterr;	// Routing error
	unsigned long proterr;	// Protocol error
	unsigned long opterr;	// Error in options
	unsigned long err;	// Misc error
};

struct stats_pbuf {
	unsigned long avail;
	unsigned long used;
	unsigned long max;
	unsigned long err;
	unsigned long reclaimed;

	unsigned long alloc_locked;
	unsigned long refresh_locked;

	unsigned long rwbufs;
};

struct netstats {
	struct stats_proto link;
	struct stats_proto ip;
	struct stats_proto icmp;
	struct stats_proto udp;
	struct stats_proto tcp;
	struct stats_proto raw;
	struct stats_pbuf pbuf;
};

static struct netstats stats;

// TODO: OS Specific.
#define inp(port)   io_inb_p(port)
#define inpw(port)  io_inw_p(port)
#define inpd(port)  io_inl_p(port)

#define outp(port, val)  io_outb_p(port,val)
#define outpw(port, val) io_outw_p(port,val)
#define outpd(port, val) io_outl_p(port,val)

typedef int port_t;

static inline void
insw(port_t port, void *buf, int count)
{
	io_insw(port, buf, count);
}

static inline void
insd(port_t port, void *buf, int count)
{
	io_insl(port, buf, count);
}

static inline void
outsw(port_t port, void *buf, int count)
{
	io_outsw(port, buf, count);
}

static inline void
outsd(port_t port, void *buf, int count)
{
	io_outsl(port, buf, count);
}

struct unit {
};				// TODO: OS Specific.
struct context {
};

#include <lwip/netif.h>

#define num_devs 16
struct dev devtab_mem[num_devs];
struct dev *devtab[num_devs] = { 0 };

//
// ether_input
//
// This function should be called when a packet is received
// from the interface. 
//

err_t
ether_input(struct netif *netif, struct pbuf *p)
{
	if (netif_is_up(netif) == 0)
		return -ENETDOWN;

	if (p->len < ETHER_ADDR_LEN) {
		printk("ether: Packet dropped due to too short packet %d %s\n",
		       p->len, netif->name);
		stats.link.lenerr++;
		stats.link.drop++;
		return -EINVAL;
	}
	// RGDTODO: See in sanos what enqueue does.
	//          => What's happening here is after we install this call,
	//          we get the packet this way and give to system.
	//          => This is called from the low-level ISR.
	//          => Make the low-level ISR work with our system.
	//             The ISR should be called from the interrupt system.
	//          This has to interface with ne2kif.c.
	//          Make a copy of ne2kif.c before we begin replacing drivers.
	//          Old version of ne2kif.c is ne2kif.c.linux_ne2k
#if 0

	struct ether_msg *msg;
	msg = (struct ether_msg *)kmalloc(sizeof(struct ether_msg));
	if (!msg)
		return -ENOMEM;

	msg->p = p;
	msg->netif = netif;

	if (enqueue(ether_queue, msg, 0) < 0) {
		if (!debugging)
			printk("ether: drop (queue full)\n");
		kfree(msg);
		stats.link.memerr++;
		stats.link.drop++;
		return -ENOMEM;
	}
#endif
	netif_rx(p);
	return 0;
}

// TODO: Signal end of interrupt, this is OS Specific.
#define PIC_MSTR_CTRL 0x20
#define PIC_EOI_BASE  0x60
#define PIC_SLV_CTRL  0x21
#define PIC_EOI_CAS   0x62
void
eoi(unsigned int irq)
{
	if (irq < 8) {
		outp(PIC_MSTR_CTRL, irq + PIC_EOI_BASE);
	} else {
		outp(PIC_SLV_CTRL, (irq - 8) + PIC_EOI_BASE);
		outp(PIC_MSTR_CTRL, PIC_EOI_CAS);
	}
}

struct netstats *
get_netstats()
{
	return &stats;
}

char *
ether2str(struct eth_addr *hwaddr, char *s)
{
	sprintk(s, "%02x:%02x:%02x:%02x:%02x:%02x",
		hwaddr->addr[0], hwaddr->addr[1], hwaddr->addr[2],
		hwaddr->addr[3], hwaddr->addr[4], hwaddr->addr[5]);
	return s;
}

// }

struct netstats *netstats;

static void
ne_readmem(struct ne *ne, unsigned short src, void *dst, unsigned short len)
{
	char *ptr = NULL;
	// Word align length
	if (len & 1)
		len++;

	// Abort any remote DMA already in progress
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);

	// Setup DMA byte count
	outp(ne->nic_addr + NE_P0_RBCR0, (unsigned char)len);
	outp(ne->nic_addr + NE_P0_RBCR1, (unsigned char)(len >> 8));

	// Setup NIC memory source address
	outp(ne->nic_addr + NE_P0_RSAR0, (unsigned char)src);
	outp(ne->nic_addr + NE_P0_RSAR1, (unsigned char)(src >> 8));

	// Select remote DMA read
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD0 | NE_CR_STA);

	// Read NIC memory
	printk("dst=%d len=%d\n", dst, len);
	printk("line=%d file=%s\n", __LINE__, __FILE__);
	// RGD WAIT
	// This is the offending line that causes the screen to go crazy...
	ptr = kmalloc(len);
	insw(ne->asic_addr + NE_NOVELL_DATA, ptr, len >> 1);
	memcpy(dst, ptr, len >> 1);
	kfree(ptr);
	//insw(ne->asic_addr + NE_NOVELL_DATA, dst, len >> 1);
	printk("line=%d file=%s\n", __LINE__, __FILE__);
}

static int
ne_probe(struct ne *ne)
{
	unsigned char byte;

	// Reset
	byte = inp(ne->asic_addr + NE_NOVELL_RESET);
	outp(ne->asic_addr + NE_NOVELL_RESET, byte);
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STP);

	//msleep(5000);
	local_sleep(1);

	// Test for a generic DP8390 NIC
	byte = inp(ne->nic_addr + NE_P0_CR);
	byte &= NE_CR_RD2 | NE_CR_TXP | NE_CR_STA | NE_CR_STP;
	if (byte != (NE_CR_RD2 | NE_CR_STP))
		return 0;

	byte = inp(ne->nic_addr + NE_P0_ISR);
	byte &= NE_ISR_RST;
	if (byte != NE_ISR_RST)
		return 0;

	printk("successfully found ne2000\n");
	return 1;
}

void
ne_get_packet(struct ne *ne, unsigned short src, char *dst, unsigned short len)
{
	printk("line %d file %s\n", __LINE__, __FILE__);
	if (src + len > ne->rx_ring_end) {
		printk("line %d file %s\n", __LINE__, __FILE__);
		unsigned short split = ne->rx_ring_end - src;

		ne_readmem(ne, src, dst, split);
		len -= split;
		src = ne->rx_ring_start;
		dst += split;
	}

	printk("line %d file %s\n", __LINE__, __FILE__);
	ne_readmem(ne, src, dst, len);
}

void
ne_receive(struct ne *ne)
{
	struct recv_ring_desc packet_hdr;
	unsigned short packet_ptr;
	unsigned short len;
	unsigned char bndry;
	struct pbuf *p, *q;
	int rc;

	// Set page 1 registers
	outp(ne->nic_addr + NE_P0_CR, NE_CR_PAGE_1 | NE_CR_RD2 | NE_CR_STA);

	while (ne->next_pkt != inp(ne->nic_addr + NE_P1_CURR)) {
		// Get pointer to buffer header structure
		packet_ptr = ne->next_pkt * NE_PAGE_SIZE;

		// Read receive ring descriptor
		ne_readmem(ne, packet_ptr, &packet_hdr,
			   sizeof(struct recv_ring_desc));

		// Allocate packet buffer
		len = packet_hdr.count - sizeof(struct recv_ring_desc);
		p = pbuf_alloc(PBUF_RAW, len, PBUF_RAM);	// TODO: OS Specific.

		// Get packet from nic and send to upper layer

		if (p != NULL) {
			packet_ptr += sizeof(struct recv_ring_desc);
			for (q = p; q != NULL; q = q->next) {
				ne_get_packet(ne, packet_ptr, q->payload,
					      (unsigned short)q->len);
				packet_ptr += q->len;
			}

			printk("ne2000: received packet, %d bytes devno %d\n",
			       len, ne->devno);
			rc = dev_receive(ne->devno, p);	// TODO: This is an error, we need to know what the callback is setup to.
			if (rc < 0) {
				printk("ne2000: error %d processing packet\n",
				       rc);
				pbuf_free(p);
			}
		} else {
			// Drop packet
			printk("ne2000: packet dropped\n");
			netstats->link.memerr++;
			netstats->link.drop++;
		}

		// Update next packet pointer
		ne->next_pkt = packet_hdr.next_pkt;

		// Set page 0 registers
		outp(ne->nic_addr + NE_P0_CR,
		     NE_CR_PAGE_0 | NE_CR_RD2 | NE_CR_STA);

		// Update boundry pointer
		bndry = ne->next_pkt - 1;
		if (bndry < ne->rx_page_start)
			bndry = ne->rx_page_stop - 1;
		outp(ne->nic_addr + NE_P0_BNRY, bndry);

		printk("start: %02x stop: %02x next: %02x bndry: %02x\n",
		       ne->rx_page_start, ne->rx_page_stop, ne->next_pkt,
		       bndry);

		// Set page 1 registers
		outp(ne->nic_addr + NE_P0_CR,
		     NE_CR_PAGE_1 | NE_CR_RD2 | NE_CR_STA);
	}
}

// RGDTODO: This is the low-level driver to be called from ISR.
void
ne_dpc(void *arg)
{
	struct ne *ne = arg;
	unsigned char isr;

	printk("ne2000: dpc\n");

	// Select page 0
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);

	// Loop until there are no pending interrupts
	while ((isr = inp(ne->nic_addr + NE_P0_ISR)) != 0) {
		printk("ne_dpc:: processing packet\n");
		// Reset bits for interrupts being acknowledged
		outp(ne->nic_addr + NE_P0_ISR, isr);

		// Packet received
		if (isr & NE_ISR_PRX) {
			printk("ne2000: new packet arrived\n");
			ne_receive(ne);
		}
		// Packet transmitted
		if (isr & NE_ISR_PTX) {
			printk("ne2000: packet transmitted\n");
			sys_mbox_post(&ne->ptx, 0);	// TODO: OS Specific, set event.
		}
		// Remote DMA complete
		if (isr & NE_ISR_RDC) {
			printk("ne2000: remote DMA complete\n");
			sys_mbox_post(&ne->rdc, 0);	// TODO: OS Specific, set event.
		}
		// Select page 0
		outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);
	}

	eoi(ne->irq);		// RGDTODO: Experiment with this, do we need it?
}

int
ne_handler()
{
	struct dev *dev = devtab[5];
	struct ne *ne = (struct ne *)dev->privdata;

	if (ne->devno != 5) {
		ne_start(ne2k_if_netif);
		dev = devtab[5];
		ne = (struct ne *)dev->privdata;
	}
	ne_dpc(ne);		// Call our ISR.
	return 0;
}

int
ne_transmit(struct dev *dev, struct pbuf *p)
{
	struct ne *ne = dev->privdata;
	unsigned short dma_len;
	unsigned short dst;
	unsigned char *data;
	int len;
	int wrap;
	unsigned char save_byte[2];
	struct pbuf *q;
	unsigned *mdata = kmalloc(4);

	printk("ne_transmit: transmit packet len=%d\n", p->tot_len);

	// Get transmit lock
#ifdef _WAIT
	sys_sem_wait(&ne->txlock);
#endif

	// We need to transfer a whole number of words
	dma_len = p->tot_len;
	if (dma_len & 1)
		dma_len++;

	// Clear packet transmitted and dma complete event
	//sys_sem_new(&ne->ptx,0); // TODO: OS Specific.
	//sys_sem_new(&ne->rdc,0); // TODO: OS Specific.

	// Set page 0 registers
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);

	// Reset remote DMA complete flag
	outp(ne->nic_addr + NE_P0_ISR, NE_ISR_RDC);

	// Set up DMA byte count
	outp(ne->nic_addr + NE_P0_RBCR0, (unsigned char)dma_len);
	outp(ne->nic_addr + NE_P0_RBCR1, (unsigned char)(dma_len >> 8));

	// Set up destination address in NIC memory
	dst = ne->rx_page_stop;	// for now we only use one tx buffer
	outp(ne->nic_addr + NE_P0_RSAR0, (dst * NE_PAGE_SIZE));
	outp(ne->nic_addr + NE_P0_RSAR1, (dst * NE_PAGE_SIZE) >> 8);

	// Set remote DMA write
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD1 | NE_CR_STA);

	wrap = 0;
	for (q = p; q != NULL; q = q->next) {
		len = q->len;
		if (len > 0 && q->payload != NULL && len <= p->tot_len) {
			data = q->payload;

			// Finish the last word
			if (wrap) {
				save_byte[1] = *data;
				outpw((unsigned short)(ne->asic_addr +
						       NE_NOVELL_DATA),
				      *(unsigned short *)save_byte);
				data++;
				len--;
				wrap = 0;
			}
			// Output contiguous words
			if (len > 1) {
				outsw(ne->asic_addr + NE_NOVELL_DATA, data,
				      len >> 1);
				data += len & ~1;
				len &= 1;
			}
			// Save last byte if necessary
			if (len == 1) {
				save_byte[0] = *data;
				wrap = 1;
			}
		}
	}

	// Output last byte
	if (wrap) {
		outpw((unsigned short)(ne->asic_addr + NE_NOVELL_DATA),
		      *(unsigned short *)save_byte);
	}
	// Wait for remote DMA complete
	if (sys_arch_mbox_fetch(&ne->rdc, (void **)&mdata, NE2K_TXTIMEOUT) ==
	    SYS_ARCH_TIMEOUT) {
		printk("ne2000: timeout waiting for remote dma to complete\n");
		sys_sem_signal(&ne->txlock);
		return -EIO;
	}
	// Set TX buffer start page
	outp(ne->nic_addr + NE_P0_TPSR, (unsigned char)dst);

	// Set TX length (packets smaller than 64 bytes must be padded)
	if (p->tot_len > 64) {
		outp(ne->nic_addr + NE_P0_TBCR0, p->tot_len);
		outp(ne->nic_addr + NE_P0_TBCR1, p->tot_len >> 8);
	} else {
		outp(ne->nic_addr + NE_P0_TBCR0, 64);
		outp(ne->nic_addr + NE_P0_TBCR1, 0);
	}

	// Set page 0 registers, transmit packet, and start
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_TXP | NE_CR_STA);

	// Wait for packet transmitted
	if (sys_arch_mbox_fetch(&ne->ptx, (void **)&mdata, NE2K_TXTIMEOUT) ==
	    SYS_ARCH_TIMEOUT) {
		printk("ne2000: timeout waiting for packet transmit\n");
		sys_sem_signal(&ne->txlock);
		return -EIO;
	}

	printk("ne_transmit: packet transmitted\n");
	pbuf_free(p);
	sys_sem_signal(&ne->txlock);	// TODO: OS Specific.
	return 0;
}

int
ne_ioctl(struct dev *dev, int cmd, void *args, size_t size)
{
	return -ENOSYS;
}

int
ne_attach(struct dev *dev, struct eth_addr *hwaddr)
{
	struct ne *ne = dev->privdata;
	*hwaddr = ne->hwaddr;

	return 0;
}

int
dev_receive(dev_t devno, struct pbuf *p)
{
	struct dev *dev;

	if (devno < 0 || devno >= num_devs)
		return -ENODEV;	// TODO: OS Specific.
	dev = devtab[devno];
	if (!dev->receive)
		return -ENOSYS;
	dev->reads++;
	dev->input += p->tot_len;

	return dev->receive(dev->netif, p);	// RGDTODO: We have to see how he does it in http://www.jbox.dk/sanos
}

// RGDTODO: Attaches receive (ether_input) and should be called on start()
int
dev_attach(dev_t devno, struct netif *netif,
	   int (*receive) (struct netif * netif, struct pbuf * p))
{
	struct dev *dev;

	if (devno < 0 || devno >= num_devs)
		return -ENODEV;
	dev = devtab[devno];
	//if (!dev->driver->attach) return -ENOSYS;

	dev->netif = netif;
	dev->receive = receive;
	LWIP_ASSERT("netif != NULL", netif != NULL);
	return ne_attach(dev, &netif->hwaddr);	// RGD TODO: This may be ok, test.
}

int
ne_detach(struct dev *dev)
{
	return 0;
}

// RGDTODO: These are to be called from the ne2kif.c code.
#if 0
struct driver ne_driver = {
	"ne2000",
	DEV_TYPE_PACKET,
	ne_ioctl,
	NULL,
	NULL,
	ne_attach,
	ne_detach,
	ne_transmit
};
#endif

int
ne_setup(struct netif *netif,
	 unsigned short iobase, int irq,
	 unsigned short membase, unsigned short memsize, struct unit *unit)
{
	struct ne *ne;
	unsigned char romdata[16] = { 0 };
	int i;
	char str[20] = { 0 };

	// Allocate device structure
	ne = (struct ne *)kmalloc(sizeof(struct ne));
	if (!ne)
		return -ENOMEM;
	memset(ne, 0, sizeof(struct ne));

	// Setup NIC configuration
	ne->iobase = iobase;
	ne->irq = irq;
	ne->membase = membase;
	ne->memsize = memsize;

	ne->nic_addr = ne->iobase + NE_NOVELL_NIC_OFFSET;
	ne->asic_addr = ne->iobase + NE_NOVELL_ASIC_OFFSET;

	ne->rx_page_start = ne->membase / NE_PAGE_SIZE;
	ne->rx_page_stop =
	    ne->rx_page_start + (ne->memsize / NE_PAGE_SIZE) -
	    NE_TXBUF_SIZE * NE_TX_BUFERS;
	ne->next_pkt = ne->rx_page_start + 1;

	ne->rx_ring_start = ne->rx_page_start * NE_PAGE_SIZE;
	ne->rx_ring_end = ne->rx_page_stop * NE_PAGE_SIZE;

	// Probe for NE2000 card
	if (!ne_probe(ne))
		return 0;

	// Initialize network interface
	sys_mbox_new(&ne->ptx, 1);	// TODO OS Specific.
	sys_mbox_new(&ne->rdc, 1);
	sys_sem_new(&ne->txlock, 0);

	// FIXME Here is where we need to enable the driver
	ne2k_enable(netif, ne->irq);
	// Install interrupt handler
	// init_dpc(&ne->dpc); // TODO: This is low-level driver.
	// register_interrupt(&ne->intr, IRQ2INTR(ne->irq), ne_handler, ne); // TODO: Register our own irq.
	//enable_irq(ne->irq); // TODO: OS specific.

	// Set page 0 registers, abort remote DMA, stop NIC
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STP);

	// Set FIFO threshold to 8, no auto-init remote DMA, byte order=80x86, word-wide DMA transfers
	outp(ne->nic_addr + NE_P0_DCR, NE_DCR_FT1 | NE_DCR_WTS | NE_DCR_LS);

	// Get Ethernet MAC address
	ne_readmem(ne, 0, romdata, 16);
	for (i = 0; i < ETHER_ADDR_LEN; i++)
		ne->hwaddr.addr[i] = romdata[i * 2];

	// Set page 0 registers, abort remote DMA, stop NIC
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STP);

	// Clear remote byte count registers
	outp(ne->nic_addr + NE_P0_RBCR0, 0);
	outp(ne->nic_addr + NE_P0_RBCR1, 0);

	// Initialize receiver (ring-buffer) page stop and boundry
	outp(ne->nic_addr + NE_P0_PSTART, ne->rx_page_start);
	outp(ne->nic_addr + NE_P0_PSTOP, ne->rx_page_stop);
	outp(ne->nic_addr + NE_P0_BNRY, ne->rx_page_start);

	// Enable the following interrupts: receive/transmit complete, receive/transmit error, 
	// receiver overwrite and remote dma complete.
	outp(ne->nic_addr + NE_P0_IMR,
	     NE_IMR_PRXE | NE_IMR_PTXE | NE_IMR_RXEE | NE_IMR_TXEE | NE_IMR_OVWE
	     | NE_IMR_RDCE);

	// Set page 1 registers
	outp(ne->nic_addr + NE_P0_CR, NE_CR_PAGE_1 | NE_CR_RD2 | NE_CR_STP);

	// Copy out our station address
	for (i = 0; i < ETHER_ADDR_LEN; i++)
		outp(ne->nic_addr + NE_P1_PAR0 + i, ne->hwaddr.addr[i]);

	// Set current page pointer 
	outp(ne->nic_addr + NE_P1_CURR, ne->next_pkt);

	// Initialize multicast address hashing registers to not accept multicasts
	for (i = 0; i < 8; i++)
		outp(ne->nic_addr + NE_P1_MAR0 + i, 0);

	// Set page 0 registers
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STP);

	// Accept broadcast packets
	outp(ne->nic_addr + NE_P0_RCR, NE_RCR_AB);

	// Take NIC out of loopback
	outp(ne->nic_addr + NE_P0_TCR, 0);

	// Clear any pending interrupts
	outp(ne->nic_addr + NE_P0_ISR, 0xFF);

	// Start NIC
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);

	// Create packet device
	// ne->devno = dev_make("eth#", &ne_driver, unit, ne); // TODO: OS Specific
	ne->devno = 5;		// TODO: Arbitrary value for now.

	printk("%s: NE2000 iobase 0x%x irq %d mac %s devno %d\n",
	       "ne2000 driver", ne->iobase, ne->irq, ether2str(&ne->hwaddr,
							       str), ne->devno);
	// Install ne on dev structure.
	{
		struct dev *dev = devtab[5];
		dev->privdata = (void *)ne;
	}
	return 0;
}

int
install(struct netif *netif, struct unit *unit, char *opts)
{
	//unsigned short iobase = 0x280;
	unsigned short iobase = 0x300;
	int irq = 5;
	unsigned short membase = 16 * 1024;
	unsigned short memsize = 16 * 1024;
#if 0
	struct resource *memres;
	// TODO: OS Specific, we will hard code ours based on QEMU.
	if (unit) {
		iobase = (unsigned short)get_unit_iobase(unit);
		irq = get_unit_irq(unit);
		memres = get_unit_resource(unit, RESOURCE_MEM, 0);
		if (memres) {
			membase = (unsigned short)memres->start;
			memsize = (unsigned short)memres->len;
		}
	}

	iobase = get_num_option(opts, "iobase", iobase);
	irq = get_num_option(opts, "irq", irq);
	membase = get_num_option(opts, "membase", membase);
	memsize = get_num_option(opts, "memsize", memsize);
#endif

	return ne_setup(netif, iobase, irq, membase, memsize, unit);
}

//int start(hmodule_t hmod, int reason, void *reserved2) {
int
ne_start(struct netif *netif)
{
	int i = 0;
	printk("ne_start\n");
	for (; i < num_devs; ++i) {
		devtab[i] = &devtab_mem[i];
		memset(devtab[i], 0x0, sizeof(struct dev));
	}
	netstats = get_netstats();
	if (install(netif, 0, 0) < 0) {
		// Sets up the NIC card and the ne structure.
		printk("Error, failed to install ne2000 driver, halt...\n");
		for (;;)	/* WAIT */
			;
	}
	dev_attach(5, netif, ether_input);
	return 1;
}

struct dev *
ne_get_dev()
{
	return devtab[5];
}
