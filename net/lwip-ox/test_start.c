#include "lwipopts.h"
#include "posix/sys/socket.h"
#include "lwip/netdb.h"
#include <string.h>
#include "lwip/init.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/stats.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "netif/etharp.h"

int wait = 1;

static struct netif netif;

extern
err_t
lwip_ne2k_init(struct netif *netif);

static void
tcpip_init_done(void *arg)
{
  ip_addr_t ipaddr, netmask, gateway;
  sys_sem_t *sem;
  sem = arg;

  printk("tcpip_init_done:: start initializing interface\n");
  /*
    CHANGE THESE to suit your own network configuration:
    This should match QEMU user network settings.

  IP4_ADDR(&gateway, 10,0,2,2); // Comcast network.
  //IP4_ADDR(&ipaddr, 192,168,124,1);
  //IP4_ADDR(&ipaddr, 10,0,0,107);
  IP4_ADDR(&ipaddr, 10,0,2,15);
  IP4_ADDR(&netmask, 255,255,255,0);

  */

  IP4_ADDR(&gateway, 10,0,0,1); // Comcast network.
  IP4_ADDR(&ipaddr, 10,0,0,109);
  IP4_ADDR(&netmask, 255,255,255,0);

  printk("addr=%d mask=%d gw=%d\n",netif.ip_addr.addr,
    netif.netmask.addr,
    netif.gw.addr);
    // See net/lwip/test/unit/etharp/test_etharp.c
   netif.hwaddr_len = ETHARP_HWADDR_LEN;
  netif_set_default(netif_add(&netif, &ipaddr, &netmask, &gateway, NULL, lwip_ne2k_init,
			      tcpip_input));
  printk("tcpip_init_done:: done initializing interface\n");
  printk("addr=%d mask=%d gw=%d\n",netif.ip_addr.addr,
    netif.netmask.addr,
    netif.gw.addr);
  netif_set_up(&netif); // Sets up that our interface is up. This must be here.
  dhcp_start(&netif);
  printk("addr=%d mask=%d gw=%d\n",netif.ip_addr.addr,
    netif.netmask.addr,
    netif.gw.addr);
  printk("tcpip_init_done:: start call sem_signal\n");
  //enable_irq(5);
  sys_sem_signal(sem);
  printk("tcpip_init_done:: done call sem_signal\n");
}

void test_init(void) {
  sys_sem_t sem;

  if(sys_sem_new(&sem, 0) != ERR_OK) {
    LWIP_ASSERT("failed to create semaphore", 0);
  }
  printk("test_init: calling<0> tcpip_init\n");
  tcpip_init(tcpip_init_done, &sem);
  printk("test_init: calling<1> tcpip_init\n");
  sys_sem_wait(&sem);
  printk("test_init: calling<2> tcpip_init\n");
  sys_sem_free(&sem);
}

void socket_test()
{
	int s;
    int len;
	struct sockaddr_in addr={0};
    struct hostent *server=NULL;
	char buffer[256];
	test_init();
    //for(;;); // WAIT // RGD TODO
    enable_irq(0); // PIT
    enable_irq(5); // NET
	printk("socket_test: calling socket()\n");
	// See include/lwip/socket define which is lwip_socket.
        //dns_init();
	s=socket(AF_INET,SOCK_STREAM,0);
	printk("socket: [%d]\n",s);
	if(s < 0) return;
	printk("socket_test: start calling gethostbyname()\n");
        // Trying to connect to google causes us to go haywire. I wonder
        // if we are actually connected...
        //server=gethostbyname("www.google.com"); // Was 192.168.1.105
        server=gethostbyname("www.objectdigital.com"); 
            server=gethostbyname("10.0.2.15");
        //server=gethostbyname("10.0.0.107");
        //server=gethostbyname("127.0.0.1");
        //server=gethostbyname("10.0.2.15");
        //server=gethostbyname("10.0.0.1");
        //server=gethostbyname("192.168.1.1");
        //server=gethostbyname("192.168.43.47"); // Me on Verizon hotspot.
	printk("socket_test: done calling gethostbyname()\n");
	printk("server [%d] h_addr [%d] h_length [%d]\n",server,server->h_addr,server->h_length);
	if(!server) {
		printk("error gethostbyname\n");
		return;
	} else {
		printk("server [%d]\n",server);
	}
	memset(&addr,0,sizeof(addr)); // RGD: This could be that the stack is corrupt.
	addr.sin_family=AF_INET;
	memcpy(&addr.sin_addr.s_addr, server->h_addr, server->h_length);
	addr.sin_port = htons(80);
	//addr.sin_port = htons(7777);
	printk("connect start\n");
    //ne2k_enable(5);
    len = connect(s,&addr,sizeof(addr));
    printk("connect=%d\n",len);
    printk("sleeping\n");
    local_sleep(10);
	strcpy(buffer,"GET / HTTP/1.0\r\n\r\n");
	printk("send start\n");
    // I think this is in tcp_out.c, tcp_output
	len = send(s,buffer,strlen(buffer),0);
    printk("sent len=%d\n",len);
	printk("recieve start\n");
    memset(buffer,0x0,256);
	len = recv(s,buffer,sizeof(buffer),0);
    printk("recieved len=%d buffer[%s]\n",len,buffer);
    for(;;); // WAIT
	return;
}
