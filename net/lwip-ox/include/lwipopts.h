#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define NO_SYS                          0
#define SYS_LIGHTWEIGHT_PROT		    1

/* APIs */
#define LWIP_RAW                        1
#define LWIP_NETCONN                    1
#define LWIP_SOCKET                     1
#define LWIP_IPV4                       1
#define LWIP_DNS                        1
#define LWIP_DHCP                       1
#define LWIP_DNS_API_DECLARE_STRUCTS    1

#define LWIP_PROVIDE_ERRNO		        1
#define MEM_LIBC_MALLOC                 1

#endif /* __LWIPOPTS_H__ */
