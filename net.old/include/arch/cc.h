/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __ARCH_CC_H__
#define __ARCH_CC_H__


/* Define platform endianness */
#ifndef BYTE_ORDER
#define BYTE_ORDER LITTLE_ENDIAN
#endif /* BYTE_ORDER */

#define LWIP_CHKSUM_ALGORITHM 2
#if 0 /* FIXME You should provide your own */
u16_t my_chksum(void *dataptr, u16_t len);
#define LWIP_CHKSUM my_chksum
#endif

/* Define generic types used in lwIP */
typedef __SIZE_TYPE__ mem_ptr_t;
typedef unsigned   char    u8_t;
typedef __UINT16_TYPE__    u16_t;
typedef __UINT32_TYPE__    u32_t;
typedef signed     char    s8_t;
typedef __INT16_TYPE__     s16_t;
typedef __INT32_TYPE__     s32_t;

/* Define (sn)printf formatters for these lwIP types */
#define X8_F  "02x"
#define U16_F "hu"
#define S16_F "hd"
#define X16_F "hx"
#define U32_F "u"
#define S32_F "d"
#define X32_F "x"

/* Compiler hints for packing structures */
#define PACK_STRUCT_STRUCT __attribute__((packed))

#if 0
#define SYS_ARCH_PROTECT(x) FIXME
#define SYS_ARCH_UNPROTECT(x) FIXME
#define SYS_ARCH_DECL_PROTECT(x) FIXME
#endif

#include <ox/error_rpt.h>
#define LWIP_PLATFORM_DIAG(x) do { printk x; } while(0)
#define LWIP_PLATFORM_ASSERT(x) do {\
	panic("Assertion \"%s\" failed at %s:%d\n", \
		(x), __FILE__, __LINE__); \
} while(0)

#endif /* __ARCH_CC_H__ */
