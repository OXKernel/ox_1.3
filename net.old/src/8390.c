
/* 8390 core for usual drivers */

static const char version[] =
    "8390.c:v1.10cvs 9/23/94 Donald Becker (becker@cesdis.gsfc.nasa.gov)\n";

#define ei_inb(_p)	io_inb(_p)
#define ei_outb(_v, _p)	io_outb(_p, _v)
#define ei_inb_p(_p)	io_inb_p(_p)
#define ei_outb_p(_v, _p) io_outb_p(_p, _v)

#include "lib8390.c"

int ei_open(struct net_device *dev)
{
	return __ei_open(dev);
}
EXPORT_SYMBOL(ei_open);

int ei_close(struct net_device *dev)
{
	return __ei_close(dev);
}
EXPORT_SYMBOL(ei_close);

netdev_tx_t ei_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	return __ei_start_xmit(skb, dev);
}
EXPORT_SYMBOL(ei_start_xmit);

struct net_device_stats *ei_get_stats(struct net_device *dev)
{
	return __ei_get_stats(dev);
}
EXPORT_SYMBOL(ei_get_stats);

void ei_set_multicast_list(struct net_device *dev)
{
	//__ei_set_multicast_list(dev); // RGD: TODO:
}
EXPORT_SYMBOL(ei_set_multicast_list);

void ei_tx_timeout(struct net_device *dev)
{
	__ei_tx_timeout(dev);
}
EXPORT_SYMBOL(ei_tx_timeout);

irqreturn_t ei_interrupt(int irq, void *dev_id)
{
	return __ei_interrupt(irq, dev_id);
}
EXPORT_SYMBOL(ei_interrupt);

#ifdef CONFIG_NET_POLL_CONTROLLER
void ei_poll(struct net_device *dev)
{
	__ei_poll(dev);
}
EXPORT_SYMBOL(ei_poll);
#endif

static inline bool is_multicast_ether_addr(const u8 *addr)
{
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	u32 a = *(const u32 *)addr;
#else
	u16 a = *(const u16 *)addr;
#endif
#ifdef __BIG_ENDIAN
	return 0x01 & (a >> ((sizeof(a) * 8) - 8));
#else
	return 0x01 & a;
#endif
}

/**
 * is_zero_ether_addr - Determine if give Ethernet address is all zeros.
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is all zeroes.
 *
 * Please note: addr must be aligned to u16.
 */
static inline bool is_zero_ether_addr(const u8 *addr)
{
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	return ((*(const u32 *)addr) | (*(const u16 *)(addr + 4))) == 0;
#else
	return (*(const u16 *)(addr + 0) |
			*(const u16 *)(addr + 2) |
			*(const u16 *)(addr + 4)) == 0;
#endif
}

int eth_validate_addr(struct net_device *dev)
{
	return !is_multicast_ether_addr(dev->dev_addr) && !is_zero_ether_addr(dev->dev_addr);
}

int eth_mac_addr(struct net_device *dev)
{
	// RGD: TODO:
	// Supposed to change the MAC address of the device.
	// See:
	//http://lxr.free-electrons.com/source/net/ethernet/eth.c#L332
	return 0;
}

int eth_change_mtu(struct net_device *dev, int new_mtu)
{
	if (new_mtu < 68 || new_mtu > ETH_DATA_LEN)
		return -EINVAL;
	dev->mtu = new_mtu;
	return 0;
}

// API...
const struct net_device_ops ei_netdev_ops = {
	.ndo_open		= ei_open,
	.ndo_stop		= ei_close,
	.ndo_start_xmit		= ei_start_xmit,
	.ndo_tx_timeout		= ei_tx_timeout,
	.ndo_get_stats		= ei_get_stats,
	.ndo_set_rx_mode	= ei_set_multicast_list,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address 	= eth_mac_addr,
	.ndo_change_mtu		= eth_change_mtu,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= ei_poll,
#endif
};

EXPORT_SYMBOL(ei_netdev_ops);

// CTOR
struct net_device *__alloc_ei_netdev(int size)
{
	struct net_device *dev = ____alloc_ei_netdev(size);
	if (dev)
		dev->netdev_ops = &ei_netdev_ops; // Setup API.
	return dev;
}
EXPORT_SYMBOL(__alloc_ei_netdev);

void NS8390_init(struct net_device *dev, int startp)
{
	__NS8390_init(dev, startp);
}
EXPORT_SYMBOL(NS8390_init);

#if defined(MODULE)

static int __init ns8390_module_init(void)
{
	return 0;
}

static void __exit ns8390_module_exit(void)
{
}

module_init(ns8390_module_init);
module_exit(ns8390_module_exit);
MODULE_LICENSE("GPL");
#endif
