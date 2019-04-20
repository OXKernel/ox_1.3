#ifndef _NE_2000_H
#define _NE_2000_H

#include <sys/types.h>
#include "netif/etharp.h"
#include "netif/ne2kif.h"

#define DEVNAMELEN 32

struct dev {
  char name[DEVNAMELEN];
  //struct driver *driver;
  //struct unit *unit;
  void *privdata;
  int refcnt;
  uid_t uid;
  gid_t gid;
  int mode;
  //struct devfile *files;
  
  int reads;
  int writes;
  int input;
  int output;

  struct netif *netif;
  int (*receive)(struct netif *netif, struct pbuf *p);
};

#define ETH_ALEN ETHER_ADDR_LEN
#define ETHER_ADDR_LEN 6

struct ne {
  dev_t devno;                          // Device number
  struct eth_addr hwaddr;               // MAC address

  unsigned short iobase;                // Configured I/O base
  unsigned short irq;                   // Configured IRQ
  unsigned short membase;               // Configured memory base
  unsigned short memsize;               // Configured memory size

  unsigned short asic_addr;             // ASIC I/O bus address
  unsigned short nic_addr;              // NIC (DP8390) I/O bus address

  // struct interrupt intr;             // Interrupt object for driver // TODO: Should be our own.
  // struct dpc dpc;                    // DPC for driver // TODO: Low-level driver.

  unsigned short rx_ring_start;         // Start address of receive ring
  unsigned short rx_ring_end;           // End address of receive ring

  unsigned char rx_page_start;          // Start of receive ring
  unsigned char rx_page_stop;           // End of receive ring
  unsigned char next_pkt;               // Next unread received packet

  sys_mbox_t rdc;                        // Remote DMA completed event // TODO: OS Specific.
  sys_mbox_t ptx;                        // Packet transmitted event
  sys_sem_t txlock;                     // Transmit lock
};

struct ne2k_if {
  struct   eth_addr      *ethaddr; //MAC Address 
  unsigned long           mem_end;
  unsigned long           mem_start;
  unsigned long           base_addr;
  int					  mtu;
  int                     irq;
  unsigned char           *dev_addr;
};

// Called be ne2kif to init the device once netif is initialized.
int ne_start(struct netif *netif);
// Called by ne2kif to do low-level send (write).
int ne_transmit(struct dev *dev, struct pbuf *p);
// Called by ne2kif to do low-level read.
void ne_receive(struct ne *ne);
// Called by ne2kif.c in the ISR handling code.
int ne_handler();
// Called by ne2000.c after a pbuf is read from the ethernet.
void netif_rx(struct pbuf *buf);

struct dev *ne_get_dev();

#endif
