/* NE2000 network card driver
 *
 * Copyright (C) 2002 Michael Ringgaard. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * OX-specific adaptations
 */

#include <ox/types.h>
#include <ox/mm/malloc.h>
#include <ox/error_rpt.h>

#include <string.h>
#include <sys/errno.h>
#include <asm_core/io.h>

#include "ne2000.h"
#include "event.h"
#include "msleep.h"

#define ETHER_ADDR_LEN 6

typedef int (*ne2k_isr_t)(void *arg);
void oxinterrupt_install(int irq, ne2k_isr_t handler, void *arg);

#define insw	io_insw
#define inp	io_inb_p
#define outp	io_outb_p
#define outsw	io_outsw
#define outpw	io_outw_p

/*
 * Page 0 register offsets
 */

#define NE_P0_CR        0x00	/* Command Register */

#define NE_P0_CLDA0     0x01	/* Current Local DMA Addr low (read) */
#define NE_P0_PSTART    0x01	/* Page Start register (write) */

#define NE_P0_CLDA1     0x02	/* Current Local DMA Addr high (read) */
#define NE_P0_PSTOP     0x02	/* Page Stop register (write) */

#define NE_P0_BNRY      0x03	/* Boundary Pointer */

#define NE_P0_TSR       0x04	/* Transmit Status Register (read) */
#define NE_P0_TPSR      0x04	/* Transmit Page Start (write) */

#define NE_P0_NCR       0x05	/* Number of Collisions Reg (read) */
#define NE_P0_TBCR0     0x05	/* Transmit Byte count, low (write) */

#define NE_P0_FIFO      0x06	/* FIFO register (read) */
#define NE_P0_TBCR1     0x06	/* Transmit Byte count, high (write) */

#define NE_P0_ISR       0x07	/* Interrupt Status Register */

#define NE_P0_CRDA0     0x08	/* Current Remote DMA Addr low (read) */
#define NE_P0_RSAR0     0x08	/* Remote Start Address low (write) */

#define NE_P0_CRDA1     0x09	/* Current Remote DMA Addr high (read) */
#define NE_P0_RSAR1     0x09	/* Remote Start Address high (write) */

#define NE_P0_RBCR0     0x0A	/* Remote Byte Count low (write) */

#define NE_P0_RBCR1     0x0B	/* Remote Byte Count high (write) */

#define NE_P0_RSR       0x0C	/* Receive Status (read) */
#define NE_P0_RCR       0x0C	/* Receive Configuration Reg (write) */

#define NE_P0_CNTR0     0x0D	/* Frame alignment error counter (read) */
#define NE_P0_TCR       0x0D	/* Transmit Configuration Reg (write) */

#define NE_P0_CNTR1     0x0E	/* CRC error counter (read) */
#define NE_P0_DCR       0x0E	/* Data Configuration Reg (write) */

#define NE_P0_CNTR2     0x0F	/* Missed packet counter (read) */
#define NE_P0_IMR       0x0F	/* Interrupt Mask Register (write) */

/*
 * Page 1 register offsets
 */

#define NE_P1_CR        0x00	/* Command Register */
#define NE_P1_PAR0      0x01	/* Physical Address Register 0 */
#define NE_P1_PAR1      0x02	/* Physical Address Register 1 */
#define NE_P1_PAR2      0x03	/* Physical Address Register 2 */
#define NE_P1_PAR3      0x04	/* Physical Address Register 3 */
#define NE_P1_PAR4      0x05	/* Physical Address Register 4 */
#define NE_P1_PAR5      0x06	/* Physical Address Register 5 */
#define NE_P1_CURR      0x07	/* Current RX ring-buffer page */
#define NE_P1_MAR0      0x08	/* Multicast Address Register 0 */
#define NE_P1_MAR1      0x09	/* Multicast Address Register 1 */
#define NE_P1_MAR2      0x0A	/* Multicast Address Register 2 */
#define NE_P1_MAR3      0x0B	/* Multicast Address Register 3 */
#define NE_P1_MAR4      0x0C	/* Multicast Address Register 4 */
#define NE_P1_MAR5      0x0D	/* Multicast Address Register 5 */
#define NE_P1_MAR6      0x0E	/* Multicast Address Register 6 */
#define NE_P1_MAR7      0x0F	/* Multicast Address Register 7 */

/*
 * Page 2 register offsets
 */

#define NE_P2_CR        0x00	/* Command Register */
#define NE_P2_PSTART    0x01	/* Page Start (read) */
#define NE_P2_CLDA0     0x01	/* Current Local DMA Addr 0 (write) */
#define NE_P2_PSTOP     0x02	/* Page Stop (read) */
#define NE_P2_CLDA1     0x02	/* Current Local DMA Addr 1 (write) */
#define NE_P2_RNPP      0x03	/* Remote Next Packet Pointer */
#define NE_P2_TPSR      0x04	/* Transmit Page Start (read) */
#define NE_P2_LNPP      0x05	/* Local Next Packet Pointer */
#define NE_P2_ACU       0x06	/* Address Counter Upper */
#define NE_P2_ACL       0x07	/* Address Counter Lower */
#define NE_P2_RCR       0x0C	/* Receive Configuration Register (read) */
#define NE_P2_TCR       0x0D	/* Transmit Configuration Register (read) */
#define NE_P2_DCR       0x0E	/* Data Configuration Register (read) */
#define NE_P2_IMR       0x0F	/* Interrupt Mask Register (read) */

/*
 * Command Register (CR)
 */

#define NE_CR_STP       0x01	/* Stop */
#define NE_CR_STA       0x02	/* Start */
#define NE_CR_TXP       0x04	/* Transmit Packet */
#define NE_CR_RD0       0x08	/* Remote DMA Command 0 */
#define NE_CR_RD1       0x10	/* Remote DMA Command 1 */
#define NE_CR_RD2       0x20	/* Remote DMA Command 2 */
#define NE_CR_PS0       0x40	/* Page Select 0 */
#define NE_CR_PS1       0x80	/* Page Select 1 */

#define NE_CR_PAGE_0    0x00	/* Select Page 0 */
#define NE_CR_PAGE_1    0x40	/* Select Page 1 */
#define NE_CR_PAGE_2    0x80	/* Select Page 2 */

/*
 * Interrupt Status Register (ISR)
 */

#define NE_ISR_PRX      0x01	/* Packet Received */
#define NE_ISR_PTX      0x02	/* Packet Transmitted */
#define NE_ISR_RXE      0x04	/* Receive Error */
#define NE_ISR_TXE      0x08	/* Transmission Error */
#define NE_ISR_OVW      0x10	/* Overwrite */
#define NE_ISR_CNT      0x20	/* Counter Overflow */
#define NE_ISR_RDC      0x40	/* Remote Data Complete */
#define NE_ISR_RST      0x80	/* Reset status */

/*
 * Interrupt Mask Register (IMR) */

#define NE_IMR_PRXE     0x01	/* Packet Received Interrupt Enable */
#define NE_IMR_PTXE     0x02	/* Packet Transmit Interrupt Enable */
#define NE_IMR_RXEE     0x04	/* Receive Error Interrupt Enable */
#define NE_IMR_TXEE     0x08	/* Transmit Error Interrupt Enable */
#define NE_IMR_OVWE     0x10	/* Overwrite Error Interrupt Enable */
#define NE_IMR_CNTE     0x20	/* Counter Overflow Interrupt Enable */
#define NE_IMR_RDCE     0x40	/* Remote DMA Complete Interrupt Enable */

/*
 * Data Configuration Register (DCR) */

#define NE_DCR_WTS      0x01	/* Word Transfer Select */
#define NE_DCR_BOS      0x02	/* Byte Order Select */
#define NE_DCR_LAS      0x04	/* Long Address Select */
#define NE_DCR_LS       0x08	/* Loopback Select */
#define NE_DCR_AR       0x10	/* Auto-initialize Remote */
#define NE_DCR_FT0      0x20	/* FIFO Threshold Select 0 */
#define NE_DCR_FT1      0x40	/* FIFO Threshold Select 1 */

/*
 * Transmit Configuration Register (TCR) */

#define NE_TCR_CRC      0x01	/* Inhibit CRC */
#define NE_TCR_LB0      0x02	/* Loopback Control 0 */
#define NE_TCR_LB1      0x04	/* Loopback Control 1 */
#define NE_TCR_ATD      0x08	/* Auto Transmit Disable */
#define NE_TCR_OFST     0x10	/* Collision Offset Enable */

/*
 * Transmit Status Register (TSR) */

#define NE_TSR_PTX      0x01	/* Packet Transmitted */
#define NE_TSR_COL      0x04	/* Transmit Collided */
#define NE_TSR_ABT      0x08	/* Transmit Aborted */
#define NE_TSR_CRS      0x10	/* Carrier Sense Lost */
#define NE_TSR_FU       0x20	/* FIFO Underrun */
#define NE_TSR_CDH      0x40	/* CD Heartbeat */
#define NE_TSR_OWC      0x80	/* Out of Window Collision */

/*
 * Receiver Configuration Register (RCR)
 */

#define NE_RCR_SEP      0x01	/* Save Errored Packets */
#define NE_RCR_AR       0x02	/* Accept Runt packet */
#define NE_RCR_AB       0x04	/* Accept Broadcast */
#define NE_RCR_AM       0x08	/* Accept Multicast */
#define NE_RCR_PRO      0x10	/* Promiscuous Physical */
#define NE_RCR_MON      0x20	/* Monitor Mode */

/*
 * Receiver Status Register (RSR) */

#define NE_RSR_PRX      0x01	/* Packet Received Intact */
#define NE_RSR_CRC      0x02	/* CRC Error */
#define NE_RSR_FAE      0x04	/* Frame Alignment Error */
#define NE_RSR_FO       0x08	/* FIFO Overrun */
#define NE_RSR_MPA      0x10	/* Missed Packet */
#define NE_RSR_PHY      0x20	/* Physical Address */
#define NE_RSR_DIS      0x40	/* Receiver Disabled */
#define NE_RSR_DFR      0x80	/* Deferring */

/*
 * Novell NE2000
 */

#define NE_NOVELL_NIC_OFFSET    0x00
#define NE_NOVELL_ASIC_OFFSET   0x10

#define NE_NOVELL_DATA          0x00
#define NE_NOVELL_RESET         0x0F

#define NE_PAGE_SIZE            256	/* Size of RAM pages in bytes */
#define NE_TXBUF_SIZE           6	/* Size of TX buffer in pages */
#define NE_TX_BUFERS            2	/* Number of transmit buffers */

#define NE_TIMEOUT              10000
#define NE_TXTIMEOUT            30000

/*
 * Receive ring descriptor
 */

struct recv_ring_desc {
	u8_t rsr;		/* Receiver status */
	u8_t next_pkt;		/* Pointer to next packet */
	u16_t count;		/* Bytes in packet (length + 4) */
};

/*
 * NE2000 NIC status
 */

struct ne {
#if 0
	dev_t devno;		/* Device number */
#endif

	u8_t hwaddr[ETHER_ADDR_LEN];	/* MAC address */

	u16_t iobase;		/* Configured I/O base */
	u16_t irq;		/* Configured IRQ */
#if 0
	u16_t membase;		/* Configured memory base */
	u16_t memsize;		/* Configured memory size */
#endif
	u16_t asic_addr;	/* ASIC I/O bus address */
	u16_t nic_addr;		/* NIC (DP8390) I/O bus address */

#if 0
	struct interrupt intr;	/* Interrupt object for driver */
	struct dpc dpc;		/* DPC for driver */
#endif

	u16_t rx_ring_start;	/* Start address of receive ring */
	u16_t rx_ring_end;	/* End address of receive ring */

	u8_t rx_page_start;	/* Start of receive ring */
	u8_t rx_page_stop;	/* End of receive ring */
	u8_t next_pkt;		/* Next unread received packet */

	struct event rdma_completed;	/* Remote DMA completed event */
	struct event packet_tx;	/* Packet transmitted event */
};

static void
ne_readmem(struct ne *ne, u16_t src, void *dst, u16_t len)
{
	/* Word align length */
	len += len & 1;

	/* Abort any remote DMA already in progress */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);

	/* Setup DMA byte count */
	outp(ne->nic_addr + NE_P0_RBCR0, (u8_t) len);
	outp(ne->nic_addr + NE_P0_RBCR1, (u8_t) (len >> 8));

	/* Setup NIC memory source address */
	outp(ne->nic_addr + NE_P0_RSAR0, (u8_t) src);
	outp(ne->nic_addr + NE_P0_RSAR1, (u8_t) (src >> 8));

	/* Select remote DMA read */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD0 | NE_CR_STA);

	/* Read NIC memory */
	insw(ne->asic_addr + NE_NOVELL_DATA, dst, len >> 1);
}

static int
ne_probe(struct ne *ne)
{
	u8_t byte;

	/* Reset */
	byte = inp(ne->asic_addr + NE_NOVELL_RESET);
	outp(ne->asic_addr + NE_NOVELL_RESET, byte);
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STP);

	msleep(100);

	/* Test for a generic DP8390 NIC */
	byte = inp(ne->nic_addr + NE_P0_CR);
	byte &= NE_CR_RD2 | NE_CR_TXP | NE_CR_STA | NE_CR_STP;
	if (byte != (NE_CR_RD2 | NE_CR_STP))
		return 0;

	byte = inp(ne->nic_addr + NE_P0_ISR);
	byte &= NE_ISR_RST;
	if (byte != NE_ISR_RST)
		return 0;

	return 1;
}

static void
ne_get_packet(struct ne *ne, u16_t src, char *dst, u16_t len)
{
	if (src + len > ne->rx_ring_end) {
		u16_t split = ne->rx_ring_end - src;

		ne_readmem(ne, src, dst, split);
		len -= split;
		src = ne->rx_ring_start;
		dst += split;
	}

	ne_readmem(ne, src, dst, len);
}

void
ne_receive(struct ne *ne)
{
	struct recv_ring_desc packet_hdr;
	u16_t packet_ptr;
	u16_t len;
	u8_t bndry;
	int rc;

	/* Set page 1 registers */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_PAGE_1 | NE_CR_RD2 | NE_CR_STA);

	while (ne->next_pkt != inp(ne->nic_addr + NE_P1_CURR)) {
		/* Get pointer to buffer header structure */
		packet_ptr = ne->next_pkt * NE_PAGE_SIZE;

		/* Read receive ring descriptor */
		ne_readmem(ne, packet_ptr, &packet_hdr,
			   sizeof(struct recv_ring_desc));

		/* Allocate packet buffer */
		len = packet_hdr.count - sizeof(struct recv_ring_desc);
		sgbuf_t *p = sgbuf_alloc(len);

		/* Get packet from nic and send to upper layer */
		if (p != NULL) {
			packet_ptr += sizeof(struct recv_ring_desc);
			for (sgbuf_t *q = p; q != NULL; q = sgbuf_next(q)) {
				ne_get_packet(ne, packet_ptr,
					      sgbuf_base(q),
					      sgbuf_len(q));
				packet_ptr += sgbuf_len(q);
			}

			printk("ne2000: received packet, %d bytes\n", len);
			rc = relay_sgbuf_to_netstack(p);
			if (rc < 0) {
				printk("ne2000: error %d processing packet\n",
				       rc);
				sgbuf_free(p);
			}
		} else {
			/* Drop packet */
			printk("ne2000: packet dropped\n");
		}

		/* Update next packet pointer */
		ne->next_pkt = packet_hdr.next_pkt;

		/* Set page 0 registers */
		outp(ne->nic_addr + NE_P0_CR,
		     NE_CR_PAGE_0 | NE_CR_RD2 | NE_CR_STA);

		/* Update boundry pointer */
		bndry = ne->next_pkt - 1;
		if (bndry < ne->rx_page_start)
			bndry = ne->rx_page_stop - 1;
		outp(ne->nic_addr + NE_P0_BNRY, bndry);

		printk("start: %02x stop: %02x next: %02x bndry: %02x\n",
		       ne->rx_page_start, ne->rx_page_stop, ne->next_pkt,
		       bndry);

		/* Set page 1 registers */
		outp(ne->nic_addr + NE_P0_CR,
		     NE_CR_PAGE_1 | NE_CR_RD2 | NE_CR_STA);
	}
}

void
ne_dpc(void *arg)
{
	struct ne *ne = arg;
	u8_t isr;

	printk("ne2000: dpc\n");

	/* Select page 0 */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);

	/* Loop until there are no pending interrupts */
	while ((isr = inp(ne->nic_addr + NE_P0_ISR)) != 0) {
        printk("--| dpc checking interrupts |--\n");
		/* Reset bits for interrupts being acknowledged */
		outp(ne->nic_addr + NE_P0_ISR, isr);

		/* Packet received */
		if (isr & NE_ISR_PRX) {
			printk("ne2000: new packet arrived\n");
			ne_receive(ne);
		}
		/* Packet transmitted */
		if (isr & NE_ISR_PTX) {
			printk("ne2000: packet transmitted\n");
			set_event(&ne->packet_tx);
		}
		/* Remote DMA complete */
		if (isr & NE_ISR_RDC) {
			printk("ne2000: remote DMA complete\n");
			set_event(&ne->rdma_completed);
		}
		/* Select page 0 */
		outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);
	}

    printk("--> dpc done <--\n");
	// FIXME eoi(ne->irq);
}

static int
ne_handler(void *arg)
{
#if 0
	struct ne *ne = (struct ne *)arg;
	/* Queue DPC to service interrupt */
	queue_irq_dpc(&ne->dpc, ne_dpc, ne);
#else
	/* FIXME we must queue this */
	ne_dpc(arg);
#endif
	return 0;
}

/**
 * Transmit a raw packet.
 *
 * Not thread safe, must be protected by a lock.
 */
int
ne_transmit(void *privdata, sgbuf_t *p)
{
	struct ne *ne = privdata;
	u16_t dst;
	u8_t *data;
	int len;
	int wrap;
	u8_t save_byte[2];
	sgbuf_t *q;

	const u16_t tot_len = sgbuf_totlen(p);
	printk("ne_transmit: transmit packet len=%d\n", tot_len);

	/* We need to transfer a whole number of words */
	const u16_t dma_len = tot_len + (tot_len & 1);

	/* Clear packet transmitted and dma complete event */
	reset_event(&ne->packet_tx);
	reset_event(&ne->rdma_completed);

	/* Set page 0 registers */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);

	/* Reset remote DMA complete flag */
	outp(ne->nic_addr + NE_P0_ISR, NE_ISR_RDC);

	/* Set up DMA byte count */
	outp(ne->nic_addr + NE_P0_RBCR0, (u8_t) dma_len);
	outp(ne->nic_addr + NE_P0_RBCR1, (u8_t) (dma_len >> 8));

	/* Set up destination address in NIC memory */
	dst = ne->rx_page_stop;	/* for now we only use one tx buffer */
	outp(ne->nic_addr + NE_P0_RSAR0, (dst * NE_PAGE_SIZE));
	outp(ne->nic_addr + NE_P0_RSAR1, (dst * NE_PAGE_SIZE) >> 8);

	/* Set remote DMA write */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD1 | NE_CR_STA);

	wrap = 0;
	for (q = p; q != NULL; q = sgbuf_next(q)) {
		len = sgbuf_len(q);
		if (len > 0) {
			data = sgbuf_base(q);

			/* Finish the last word */
			if (wrap) {
				save_byte[1] = *data;
				outpw((u16_t) (ne->asic_addr +
					       NE_NOVELL_DATA),
				      *(u16_t *) save_byte);
				data++;
				len--;
				wrap = 0;
			}
			/* Output contiguous words */
			if (len > 1) {
				outsw(ne->asic_addr + NE_NOVELL_DATA, data,
				      len >> 1);
				data += len & ~1;
				len &= 1;
			}
			/* Save last byte if necessary */
			if (len == 1) {
				save_byte[0] = *data;
				wrap = 1;
			}
		}
	}

	/* Output last byte */
	if (wrap) {
		outpw((u16_t) (ne->asic_addr + NE_NOVELL_DATA),
		      *(u16_t *) save_byte);
	}
	/* Wait for remote DMA complete */
    printk("ne_transmit:: wait_for_object line %d file %s\n",__LINE__,__FILE__);
	if (wait_for_object(&ne->rdma_completed, NE_TIMEOUT) < 0) {
		printk(		/* KERN_WARNING */
			      "ne2000: timeout waiting for remote dma to complete\n");
		return -EIO;
	}
	/* Set TX buffer start page */
	outp(ne->nic_addr + NE_P0_TPSR, (u8_t) dst);

	/* Set TX length (packets smaller than 64 bytes must be padded) */
	if (p->tot_len > 64) {
		outp(ne->nic_addr + NE_P0_TBCR0, p->tot_len);
		outp(ne->nic_addr + NE_P0_TBCR1, p->tot_len >> 8);
	} else {
		outp(ne->nic_addr + NE_P0_TBCR0, 64);
		outp(ne->nic_addr + NE_P0_TBCR1, 0);
	}

	/* Set page 0 registers, transmit packet, and start */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_TXP | NE_CR_STA);

	/* Wait for packet transmitted */
    printk("ne_transmit:: wait_for_object line %d file %s\n",__LINE__,__FILE__);
	if (wait_for_object(&ne->packet_tx, NE_TIMEOUT) < 0) {
		printk(		/* KERN_WARNING */
			      "ne2000: timeout waiting for packet transmit\n");
		return -EIO;
	}
	printk("ne_transmit: packet transmitted\n");
	sgbuf_free(p);
	return 0;
}

int
ne_setup(void *privdata, u16_t iobase, u16_t irq, u16_t membase, u16_t memsize)
{
	struct ne *ne = privdata;
	u8_t romdata[16];
	int i;
	char str[20];

	/* Clean up device structure */
	memset(ne, 0, sizeof(struct ne));

	/* Setup NIC configuration */
	ne->iobase = iobase;
	ne->irq = irq;
#if 0
	ne->membase = membase;
	ne->memsize = memsize;
#endif
	ne->nic_addr = ne->iobase + NE_NOVELL_NIC_OFFSET;
	ne->asic_addr = ne->iobase + NE_NOVELL_ASIC_OFFSET;

	ne->rx_page_start = membase / NE_PAGE_SIZE;
	ne->rx_page_stop =
	    ne->rx_page_start + (memsize / NE_PAGE_SIZE) -
	    NE_TXBUF_SIZE * NE_TX_BUFERS;
	ne->next_pkt = ne->rx_page_start + 1;

	ne->rx_ring_start = ne->rx_page_start * NE_PAGE_SIZE;
	ne->rx_ring_end = ne->rx_page_stop * NE_PAGE_SIZE;

	/* Probe for NE2000 card */
	if (!ne_probe(ne))
		return -ENODEV;

	/* Initialize network interface */
	init_event(&ne->packet_tx, 0, 0);
	init_event(&ne->rdma_completed, 0, 0);

	/* Install interrupt handler */
	/* FIXME following code must be externalized */
#if 0
	init_dpc(&ne->dpc);
	register_interrupt(&ne->intr, IRQ2INTR(ne->irq), ne_handler, ne);
	enable_irq(ne->irq);
#else
	/* XXX OS-specific code */
	oxinterrupt_install(irq, ne_handler, ne);
#endif

	/* Set page 0 registers, abort remote DMA, stop NIC */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STP);

	/* Set FIFO threshold to 8, no auto-init remote DMA, byte order=80x86, word-wide DMA transfers */
	outp(ne->nic_addr + NE_P0_DCR, NE_DCR_FT1 | NE_DCR_WTS | NE_DCR_LS);

	/* Get Ethernet MAC address */
	ne_readmem(ne, 0, romdata, 16);
	for (i = 0; i < ETHER_ADDR_LEN; i++)
		ne->hwaddr[i] = romdata[i * 2];

	/* Set page 0 registers, abort remote DMA, stop NIC */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STP);

	/* Clear remote byte count registers */
	outp(ne->nic_addr + NE_P0_RBCR0, 0);
	outp(ne->nic_addr + NE_P0_RBCR1, 0);

	/* Initialize receiver (ring-buffer) page stop and boundry */
	outp(ne->nic_addr + NE_P0_PSTART, ne->rx_page_start);
	outp(ne->nic_addr + NE_P0_PSTOP, ne->rx_page_stop);
	outp(ne->nic_addr + NE_P0_BNRY, ne->rx_page_start);

	/* Enable the following interrupts: receive/transmit complete,
	 * receive/transmit error, receiver overwrite and remote dma complete.
	 */
	outp(ne->nic_addr + NE_P0_IMR,
	     NE_IMR_PRXE | NE_IMR_PTXE | NE_IMR_RXEE | NE_IMR_TXEE | NE_IMR_OVWE
	     | NE_IMR_RDCE);

	/* Set page 1 registers */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_PAGE_1 | NE_CR_RD2 | NE_CR_STP);

	/* Copy out our station address */
	for (i = 0; i < ETHER_ADDR_LEN; i++)
		outp(ne->nic_addr + NE_P1_PAR0 + i, ne->hwaddr[i]);

	/* Set current page pointer  */
	outp(ne->nic_addr + NE_P1_CURR, ne->next_pkt);

	/* Initialize multicast address hashing registers to not accept multicasts */
	for (i = 0; i < 8; i++)
		outp(ne->nic_addr + NE_P1_MAR0 + i, 0);

	/* Set page 0 registers */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STP);

	/* Accept broadcast packets */
	outp(ne->nic_addr + NE_P0_RCR, NE_RCR_AB);

	/* Take NIC out of loopback */
	outp(ne->nic_addr + NE_P0_TCR, 0);

	/* Clear any pending interrupts */
	outp(ne->nic_addr + NE_P0_ISR, 0xFF);

	/* Start NIC */
	outp(ne->nic_addr + NE_P0_CR, NE_CR_RD2 | NE_CR_STA);

	return 0;
}


/*
 * OX-specific interface code
 */
size_t
ne_privdata_size(void)
{
	return sizeof(struct ne);
}

const u8_t *
ne_hwaddr_get(void *arg)
{
	struct ne *ne = arg;
	return ne->hwaddr;
}
