#
# Copyright (c) 2001, 2002 Swedish Institute of Computer Science.
# All rights reserved. 
# 
# Redistribution and use in source and binary forms, with or without modification, 
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission. 
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
# SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
# OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
# OF SUCH DAMAGE.
#
# This file is part of the lwIP TCP/IP stack.
# 
# Author: Adam Dunkels <adam@sics.se>
#
CFLAGS=-I./include
LWIPDIR=.
# COREFILES, CORE4FILES: The minimum set of files needed for lwIP.
COREFILES=$(LWIPDIR)/core/init.o \
	$(LWIPDIR)/core/def.o \
	$(LWIPDIR)/core/dns.o \
	$(LWIPDIR)/core/inet_chksum.o \
	$(LWIPDIR)/core/ip.o \
	$(LWIPDIR)/core/mem.o \
	$(LWIPDIR)/core/memp.o \
	$(LWIPDIR)/core/netif.o \
	$(LWIPDIR)/core/pbuf.o \
	$(LWIPDIR)/core/raw.o \
	$(LWIPDIR)/core/stats.o \
	$(LWIPDIR)/core/sys.o \
	$(LWIPDIR)/core/tcp.o \
	$(LWIPDIR)/core/tcp_in.o \
	$(LWIPDIR)/core/tcp_out.o \
	$(LWIPDIR)/core/timeouts.o \
	$(LWIPDIR)/core/udp.o

CORE4FILES=$(LWIPDIR)/core/ipv4/autoip.o \
	$(LWIPDIR)/core/ipv4/dhcp.o \
	$(LWIPDIR)/core/ipv4/etharp.o \
	$(LWIPDIR)/core/ipv4/icmp.o \
	$(LWIPDIR)/core/ipv4/igmp.o \
	$(LWIPDIR)/core/ipv4/ip4_frag.o \
	$(LWIPDIR)/core/ipv4/ip4.o \
	$(LWIPDIR)/core/ipv4/ip4_addr.o

CORE6FILES=$(LWIPDIR)/core/ipv6/dhcp6.o \
	$(LWIPDIR)/core/ipv6/ethip6.o \
	$(LWIPDIR)/core/ipv6/icmp6.o \
	$(LWIPDIR)/core/ipv6/inet6.o \
	$(LWIPDIR)/core/ipv6/ip6.o \
	$(LWIPDIR)/core/ipv6/ip6_addr.o \
	$(LWIPDIR)/core/ipv6/ip6_frag.o \
	$(LWIPDIR)/core/ipv6/mld6.o \
	$(LWIPDIR)/core/ipv6/nd6.o

# APIFILES: The files which implement the sequential and socket APIs.
APIFILES=$(LWIPDIR)/api/api_lib.o \
	$(LWIPDIR)/api/api_msg.o \
	$(LWIPDIR)/api/err.o \
	$(LWIPDIR)/api/netbuf.o \
	$(LWIPDIR)/api/netdb.o \
	$(LWIPDIR)/api/netifapi.o \
	$(LWIPDIR)/api/sockets.o \
	$(LWIPDIR)/api/tcpip.o

# NETIFFILES: Files implementing various generic network interface functions
NETIFFILES=$(LWIPDIR)/netif/ethernet.o \
	$(LWIPDIR)/netif/slipif.o

# SIXLOWPAN: 6LoWPAN
SIXLOWPAN=$(LWIPDIR)/netif/lowpan6.o \

# PPPFILES: PPP
PPPFILES=$(LWIPDIR)/netif/ppp/auth.o \
	$(LWIPDIR)/netif/ppp/ccp.o \
	$(LWIPDIR)/netif/ppp/chap-md5.o \
	$(LWIPDIR)/netif/ppp/chap_ms.o \
	$(LWIPDIR)/netif/ppp/chap-new.o \
	$(LWIPDIR)/netif/ppp/demand.o \
	$(LWIPDIR)/netif/ppp/eap.o \
	$(LWIPDIR)/netif/ppp/ecp.o \
	$(LWIPDIR)/netif/ppp/eui64.o \
	$(LWIPDIR)/netif/ppp/fsm.o \
	$(LWIPDIR)/netif/ppp/ipcp.o \
	$(LWIPDIR)/netif/ppp/ipv6cp.o \
	$(LWIPDIR)/netif/ppp/lcp.o \
	$(LWIPDIR)/netif/ppp/magic.o \
	$(LWIPDIR)/netif/ppp/mppe.o \
	$(LWIPDIR)/netif/ppp/multilink.o \
	$(LWIPDIR)/netif/ppp/ppp.o \
	$(LWIPDIR)/netif/ppp/pppapi.o \
	$(LWIPDIR)/netif/ppp/pppcrypt.o \
	$(LWIPDIR)/netif/ppp/pppoe.o \
	$(LWIPDIR)/netif/ppp/pppol2tp.o \
	$(LWIPDIR)/netif/ppp/pppos.o \
	$(LWIPDIR)/netif/ppp/upap.o \
	$(LWIPDIR)/netif/ppp/utils.o \
	$(LWIPDIR)/netif/ppp/vj.o \
	$(LWIPDIR)/netif/ppp/polarssl/arc4.o \
	$(LWIPDIR)/netif/ppp/polarssl/des.o \
	$(LWIPDIR)/netif/ppp/polarssl/md4.o \
	$(LWIPDIR)/netif/ppp/polarssl/md5.o \
	$(LWIPDIR)/netif/ppp/polarssl/sha1.o

# LWIPNOAPPSFILES: All LWIP files without apps
LWIPNOAPPSFILES=$(COREFILES) \
	$(CORE4FILES) \
	$(CORE6FILES) \
	$(APIFILES) \
	$(NETIFFILES) \
	$(PPPFILES) \
	$(SIXLOWPAN)

# SNMPFILES: SNMPv2c agent
SNMPFILES=$(LWIPDIR)/apps/snmp/snmp_asn1.o \
	$(LWIPDIR)/apps/snmp/snmp_core.o \
	$(LWIPDIR)/apps/snmp/snmp_mib2.o \
	$(LWIPDIR)/apps/snmp/snmp_mib2_icmp.o \
	$(LWIPDIR)/apps/snmp/snmp_mib2_interfaces.o \
	$(LWIPDIR)/apps/snmp/snmp_mib2_ip.o \
	$(LWIPDIR)/apps/snmp/snmp_mib2_snmp.o \
	$(LWIPDIR)/apps/snmp/snmp_mib2_system.o \
	$(LWIPDIR)/apps/snmp/snmp_mib2_tcp.o \
	$(LWIPDIR)/apps/snmp/snmp_mib2_udp.o \
	$(LWIPDIR)/apps/snmp/snmp_msg.o \
	$(LWIPDIR)/apps/snmp/snmpv3.o \
	$(LWIPDIR)/apps/snmp/snmp_netconn.o \
	$(LWIPDIR)/apps/snmp/snmp_pbuf_stream.o \
	$(LWIPDIR)/apps/snmp/snmp_raw.o \
	$(LWIPDIR)/apps/snmp/snmp_scalar.o \
	$(LWIPDIR)/apps/snmp/snmp_table.o \
	$(LWIPDIR)/apps/snmp/snmp_threadsync.o \
	$(LWIPDIR)/apps/snmp/snmp_traps.o \
	$(LWIPDIR)/apps/snmp/snmpv3_mbedtls.o \
	$(LWIPDIR)/apps/snmp/snmpv3_dummy.o

# HTTPDFILES: HTTP server
HTTPDFILES=$(LWIPDIR)/apps/httpd/fs.o \
	$(LWIPDIR)/apps/httpd/httpd.o

# LWIPERFFILES: IPERF server
LWIPERFFILES=$(LWIPDIR)/apps/lwiperf/lwiperf.o

# SNTPFILES: SNTP client
SNTPFILES=$(LWIPDIR)/apps/sntp/sntp.o

# NETBIOSNSFILES: NetBIOS name server
NETBIOSNSFILES=$(LWIPDIR)/apps/netbiosns/netbiosns.o

# LWIPAPPFILES: All LWIP APPs
LWIPAPPFILES=$(SNMPFILES) \
	$(HTTPDFILES) \
	$(LWIPERFFILES) \
	$(SNTPFILES) \
	$(NETBIOSNSFILES)

ALL: $(LWIPAPPFILES)
	ar rv liblwip.a $(LWIPAPPFILES) 
