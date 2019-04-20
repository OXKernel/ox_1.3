// Driver based on ne2000.c from sanos.
/*
*********************************************************************************************************
*                                              lwIP TCP/IP Stack
*                                    	 port for uC/OS-II RTOS on TIC6711 DSK
*
*********************************************************************************************************
*/
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/timers.h"
#include <lwip/stats.h>

#include "netif/etharp.h"

#include "netif/ne2kif.h"
#include <asm_core/io.h>
#include <sys/errno.h>
#include <ox/mm/malloc.h>
#include <ox/lib/printk.h>
#include <ox/sleep.h>
#include <lwip/sys.h>
typedef unsigned   int     u32_t;
#include <arch/sys_arch.h> // For mbox.
#include "ne2000.h"

// RGDTODO: Fix this to use ne2000 code.
static struct dev *dev; // Our network device.

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 't'

struct netif *ne2k_if_netif;   

/*----------------------------------------------------------------------------------------
  ****************************************************************************************
  ----------------------------------------------------------------------------------------*/
/*
 * ethernetif_init():
 *
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 */
volatile int ne2k_started=0;
err_t ne2k_init(struct netif *netif)
{
  struct ne2k_if *ne2k_if;

  if(ne2k_started) {
  	return ERR_OK;
  } else {
  	ne2k_started = 1; // Need pthread_once semantic.
  }

  ne_start(netif); // Call ne2000 driver start.
  dev = ne_get_dev(); // Call ne2000 driver for the dev strucure.

  ne2k_if = mem_malloc(sizeof(struct ne2k_if));//MAC Address
  if (ne2k_if == NULL) {
  	LWIP_DEBUGF(NETIF_DEBUG,("ne2k_init: out of memory!\n"));
  	return ERR_MEM;
  }

  LWIP_ASSERT("netif != NULL", netif != NULL);
  netif->state = ne2k_if;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  netif->output = etharp_output; // This is ok, its from LWIP.
  netif->linkoutput = low_level_output;

  ne2k_if->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);
  ne2k_if->base_addr = 0x300; // Base_ADDR;

  TRACE("low_level_init");
  low_level_init(netif);
  
  TRACE("sys_timeout");
  sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
  
  TRACE("done!");
  //netif_set_up(ne2k_if); // Must be done in order to connect, done in test_main.c
  return ERR_OK;
}

/**
 *  arp_timer.
 */
static void arp_timer(void *arg)
{
  etharp_tmr();
  sys_timeout(ARP_TMR_INTERVAL, (sys_timeout_handler)arp_timer, NULL);
}

/**
 * Initialize the ne2k ethernet chip, resetting the interface and getting the ethernet
 * address.
 */
void low_level_init(struct netif * netif)
{
	u16_t i;
	struct ne2k_if *ne2k_if;

	TRACE("start");
	LWIP_ASSERT("netif != NULL", netif != NULL);

	ne2k_if = netif->state;
	// the meaning of "netif->state" can be defined in drivers, here for MAC address!
	
	netif->hwaddr_len=6;
	netif->mtu = 1500;	
	netif->flags = NETIF_FLAG_BROADCAST;
    // FIXME we need a proper way to sleep here
    for (i=0;i<DELAY_MS;i++); //wait

    // MAC Address read in the probe, see RGD: I would guess... comment in ne2kx86.c
    struct ne *ne = dev->privdata;
    for(i =0; i < ETH_ALEN; i++) {
  	    ne2k_if->ethaddr->addr[i] = (u8_t)ne->hwaddr.addr[i];
    }
    
    ne2k_if_netif = netif;
}


/*----------------------------------------------------------------------------------------
  ****************************************************************************************
  ----------------------------------------------------------------------------------------*/

/*
 * low_level_output():
 *
 * Should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 */
static err_t low_level_output(struct netif * netif, struct pbuf *p)
{
	struct pbuf *q;
	u16_t packetLength,remote_Addr,Count;
	u8_t *buf;

	printk("low_level_output start\n");
	/*
	 * Write packet to ring buffers.
	 * NOTE: In this version we output the entire pbuf 
	 *       inside the ne2000.c driver. This is not the
	 *       case in the ne2kifax.c.disabled code which
	 *       actually dumps out the raw char * payload.
	 */
    ne_transmit(dev,p);

	return ERR_OK;
}

/*----------------------------------------------------------------------------------------
  ****************************************************************************************
  ----------------------------------------------------------------------------------------*/

/*
 * ethernetif_input():
 *
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface.
 *
 */
static void 
ne2k_input(struct netif *netif, struct pbuf *p)
{
  struct ne2k_if *ne2k_if;
  struct eth_hdr *ethhdr;

  ne2k_if = netif->state;
  
  /* move received packet into a new pbuf */
  /* no packet could be read, silently ignore this */
  if (p == NULL) return;
  /* points to packet payload, which starts with an Ethernet header */
  ethhdr = p->payload;

#if LINK_STATS
  lwip_stats.link.recv++;
#endif /* LINK_STATS */  

  switch(htons(ethhdr->type)) {
  /* IP packet? */
	case ETHTYPE_IP:
	case ETHTYPE_ARP:
    	if(netif->input(p, netif)!= ERR_OK){
    		pbuf_free(p);
    		p = NULL;
    	}
    	break;
  default:
		pbuf_free(p);
		p = NULL;
		break;
  }
}

/**
 *  ne2k_rx.
 */
void ne2k_rx(struct pbuf *p)
{
	u8_t  curr,bnry,loopCnt = 0;
		
	printk("ne2k_rx: start\n");
	while(loopCnt < 10) { // RGD: TODO: Why 10 what's that about. We should recieve what we do.
		ne2k_input(ne2k_if_netif,p);
		loopCnt++;
	}
}

// Entry point from ISR in lib8390.c
void netif_rx(struct pbuf *buf)
{
    ne2k_rx(buf); // Process incoming packet.
}

/*---*---*---*---*---*---*---*
 *     void ne2k_isr(void)
 *    can be int 4 5 6 or 7 
 *---*---*---*---*---*---*---*/
void ne2k_isr(int irq)
{
    ne_handler(); // Call ne2000 driver.
}
