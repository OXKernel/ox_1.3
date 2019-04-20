#include <ox/lib/printk.h>
#include <ox/mm/malloc.h>
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"
#include "ne2000.h"

/**
 * Transmit raw packet.
 */
static
err_t
lwip_raw_xmit(struct netif *netif, struct pbuf *p)
{
	/* FIXME ne_transmit must be called with a lock held */
	/* XXX struct pbuf == sgbuf_t */
	ne_transmit(netif->state, p);
}

/**
 * Initialize the card. Called from netif_add.
 */
err_t
lwip_ne2k_init(struct netif *netif)
{
	// FIXME
	netif->linkoutput = lwip_raw_xmit;
	netif->output = etharp_output;

	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

	// netif->state contains the driver's internal state (privdata)
	// netif->hwaddr_len, my_netif->hwaddr ethernet address FIXME
	// netif->mtu = FIXME
	// netif->name[2], used by netif_find
	//
	// Set when link is established (cable connected, right?)
	netif->flags |= NETIF_FLAG_LINK_UP;
	// FIXME define LWIP_NETIF_LINK_CALLBACK instead and call
	// netif_set_link_down/up
}

static char *
macaddrfmt(char *dst, const u8_t *hwaddr)
{
	sprintk(dst, "%02x:%02x:%02x:%02x:%02x:%02x",
		hwaddr[0], hwaddr[1], hwaddr[2],
		hwaddr[3], hwaddr[4], hwaddr[5]);
	return dst;
}

static struct netif *netif;

err_t
lwip_ne2k_input(struct pbuf *p)
{
	return netif->input(p, netif);
}

void
lwip_ne2k_add_interface(u16_t iobase, u16_t irq, u16_t membase, u16_t memsize)
{
	void *state = NULL;

	netif = kmalloc(sizeof(*netif));
	if (!netif)
		goto oom;

	state = kmalloc(ne_privdata_size());
	if (!state)
		goto oom;

	if (ne_setup(state, iobase, irq, membase, memsize) != 0) {
		printk("ne2k: no interface found");
		goto error;
	} else {
		char tmp[18];
		printk("ne2000: iobase 0x%x irq %d mac %s\n",
		       iobase, irq, macaddrfmt(tmp, ne_hwaddr_get(state)));
	}
	netif_add(netif, IP_ADDR_ANY, IP_ADDR_ANY, IP_ADDR_ANY, state,
		  lwip_ne2k_init, ethernet_input);

	return;
oom:
	printk("ne2k: out of memory");
error:
	kfree(netif);
	kfree(state);
}
