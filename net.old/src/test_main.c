#include <sys/socket.h>
#include <netdb.h>
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
#include "netif/ne2kif.h"
#include "lwip/netdb.h"

int wait = 1;


void socket_test()
{
	int s;
    int len;
	struct sockaddr_in addr;
        struct hostent *server=NULL;
	char buffer[256];
	test_init();
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
        //server=gethostbyname("10.0.2.15");
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
	memset(&addr,0,sizeof(addr));
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
