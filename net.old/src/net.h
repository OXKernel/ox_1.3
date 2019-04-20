#ifndef _OX_NET_H
#define _OX_NET_H

#define NETIF_MSG_DRV 0x0001
#define NETDEV_TX_BUSY    -1
#define NETDEV_TX_OK	   0
#define ETH_ALEN           6 /* Octets in one ethernet addr   */
#define ETH_ZLEN		  60
#define ETH_DATA_LEN    1500
#define ETH_FRAME_LEN   1514
#define ETH_FCS_LEN        4

#include "string.h" // memset, memcpy, etc...
#include "arch/cc.h"
#include "arch/sys_arch.h"
#include "lwip/pbuf.h"
#include <ox/bool_t.h>

typedef int irqreturn_t;
typedef int netdev_tx_t;
typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;

#define IRQ_NONE (-1)
#define spinlock_t sys_sem_t
#define __initdata
#define __iomem
#define sk_buff pbuf /* RGD: Use LWIP data structure */

struct net_device_stats {
	unsigned long   rx_packets;
	unsigned long   tx_packets;
	unsigned long   rx_bytes;
	unsigned long   tx_bytes;
	unsigned long   rx_errors;
	unsigned long   tx_errors;
	unsigned long   rx_dropped;
	unsigned long   tx_dropped;
	unsigned long   multicast;
	unsigned long   collisions;
	unsigned long   rx_length_errors;
	unsigned long   rx_over_errors;
	unsigned long   rx_crc_errors;
	unsigned long   rx_frame_errors;
	unsigned long   rx_fifo_errors;
	unsigned long   rx_missed_errors;
	unsigned long   tx_aborted_errors;
	unsigned long   tx_carrier_errors;
	unsigned long   tx_fifo_errors;
	unsigned long   tx_heartbeat_errors;
	unsigned long   tx_window_errors;
	unsigned long   rx_compressed;
	unsigned long   tx_compressed;
};

struct ne2k_if;
#define net_device ne2k_if

struct net_device_ops {
	int 		 			 (*ndo_open)(struct net_device *dev);
	int 	 	 			 (*ndo_stop)(struct net_device *dev);
	netdev_tx_t  			 (*ndo_start_xmit)(struct sk_buff *skb,
											   struct net_device *dev);
	void         			 (*ndo_tx_timeout) (struct net_device *dev);
	struct net_device_stats* (*ndo_get_stats)(struct net_device *dev);
	void                     (*ndo_set_rx_mode)(struct net_device *dev);
	int                      (*ndo_validate_addr)(struct net_device *dev);
	int                      (*ndo_set_mac_address)(struct net_device *dev,
                                                    void *addr); 
    int                      (*ndo_change_mtu)(struct net_device *dev,
											   int new_mtu);
    void                     (*ndo_poll_controller)(struct net_device *dev);
};

struct ne2k_if {
  struct   eth_addr      *ethaddr; //MAC Address 
  unsigned long           mem_end;
  unsigned long           mem_start;
  unsigned long           base_addr;
  int					  mtu;
  int                     irq;
  unsigned char           *dev_addr;
  struct net_device_stats stats;
  struct net_device_ops   *netdev_ops; // API for the network device.
  unsigned long			  watchdog_timeo;
  void                    *priv;
};

#define netdev_priv(dev) dev->priv

#define netdev_warn printk
#define netdev_info printk
#define netdev_err  printk
#define netdev_notice printk
#define netdev_dbg  printk
#define pr_cont     printk
#define netif_err	printk
#define netif_warning printk
#define netdev_warn printk
#define netif_dbg	printk
#define pr_err		printk

#include "8390.h"

unsigned long asm_get_eflags ( void );
void asm_set_eflags ( unsigned long flags );

#ifndef NULL
#define NULL ((void *)0)
#endif
#define EXPORT_SYMBOL(x)
#define ARRAY_SIZE(x) (sizeof((x)) / sizeof(x[0]))

#define time_after(a,b)         \
    (((long)((b) - (a)) < 0))

#endif
