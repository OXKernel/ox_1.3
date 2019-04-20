#ifndef __ARCH_CC_H__
#define __ARCH_CC_H__

#define BYTE_ORDER LITTLE_ENDIAN /* FIXME we need to get it from OX */

#define LWIP_CHKSUM_ALGORITHM 2
#if 0 /* FIXME You should provide your own */
u16_t my_chksum(void *dataptr, u16_t len);
#define LWIP_CHKSUM my_chksum
#endif


/* Define generic types used in lwIP */
#define LWIP_NO_STDINT_H 1 /* We do not provide stdint.h */
typedef __SIZE_TYPE__	size_t;
typedef __SIZE_TYPE__	mem_ptr_t;
typedef unsigned char	u8_t;
typedef __UINT16_TYPE__	u16_t;
typedef __UINT32_TYPE__	u32_t;
typedef signed char	s8_t;
typedef __INT16_TYPE__	s16_t;
typedef __INT32_TYPE__	s32_t;

/* Define (sn)printf formatters for these lwIP types */
#define LWIP_NO_INTTYPES_H 1 /* We do not provide inttypes.h */
#define X8_F  "02x"
#define U16_F "hu"
#define S16_F "hd"
#define X16_F "hx"
#define U32_F "u"
#define S32_F "d"
#define X32_F "x"

/* Compiler hints for packing structures */
#define PACK_STRUCT_STRUCT __attribute__((packed))

#include <ox/error_rpt.h>
#define LWIP_PLATFORM_DIAG(x) do { printk x; } while(0)
#define LWIP_PLATFORM_ASSERT(x) do {\
	panic("Assertion \"%s\" failed at %s:%d\n", \
		(x), __FILE__, __LINE__); \
} while(0)

#endif /* __ARCH_CC_H__ */
