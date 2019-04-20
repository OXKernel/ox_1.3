#include <ox/lib/printk.h>
#include <drivers/chara/ne2k.h>
#include <netif/ne2kif.h>

#include <lwip/netif.h>
#include <lwip/tcpip.h>
#include <lwip/ip_addr.h>

void ping_init(void);
void ping_send_now(void);

void network_init(void);


typedef char ip4str[16];

inline static void
ip4fmt(char *buf, u32_t addr)
{
	sprintk(buf, "%d.%d.%d.%d",
		255 & (addr),
		255 & (addr >> 8),
		255 & (addr >> 16),
		(addr >> 24));
}

struct netif netif;

static void
tcpip_init_done(void *arg)
{
	sys_sem_t *ready = arg;

	printk("tcpip_init_done:: start initializing interface\n");
	netif_add(&netif, IP_ADDR_ANY, IP_ADDR_ANY, IP_ADDR_ANY, NULL, ne2k_init, tcpip_input);
	printk("tcpip_init_done:: done initializing interface\n");
	
	TRACE("enable_irq");
	enable_irq(5);

	TRACE("dhcp_start");
	dhcp_start(&netif); // FIXME not working!

	ip4str addr, mask, gw;
	ip4fmt(addr, netif.ip_addr.addr);
	ip4fmt(mask, netif.netmask.addr);
	ip4fmt(gw,   netif.gw.addr);
	printk("tcpip_init_done:: Address: %s  Mask: %s  Gateway: %s\n", addr, mask, gw);

	TRACE("netif_set_default");
	netif_set_default(&netif);
	TRACE("netif_set_up");
	netif_set_up(&netif);

	printk("tcpip_init_done:: start call sem_signal\n");
	sys_sem_signal(ready);
	printk("tcpip_init_done:: done call sem_signal\n");
}

static
void network_test(void)
{
	ping_init();
	ping_send_now();
}

void
network_init(void)
{
	//ne2k_enable(ETHER_IRQ);
	TRACE("asm_enable_interrupt");
	asm_enable_interrupt();

	memset(&netif, 0, sizeof(netif));

	sys_sem_t ready;
	if(sys_sem_new(&ready, 0) != ERR_OK)
		LWIP_ASSERT("failed to create semaphore", 0);
	printk("network_init: tcpip_init\n");
	tcpip_init(tcpip_init_done, &ready);
	printk("network_init: tcpip_init done\n");
	sys_sem_wait(&ready);
	sys_sem_free(&ready);

	network_test();

	ne2k_disable();
	asm_disable_interrupt();
}
