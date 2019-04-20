#ifndef _ISA_PNP_H
#define _ISA_PNP_H

typedef unsigned long kernel_ulong_t;

#define ISAPNP_ANY_ID           0xffff

#define ISAPNP_VENDOR(a,b,c)    (((((a)-'A'+1)&0x3f)<<2)|\
                                    ((((b)-'A'+1)&0x18)>>3)|((((b)-'A'+1)&7)<<13)|\
                                    ((((c)-'A'+1)&0x1f)<<8))
    #define ISAPNP_DEVICE(x)        ((((x)&0xf000)>>8)|\
                                     (((x)&0x0f00)>>8)|\
                                     (((x)&0x00f0)<<8)|\
                                     (((x)&0x000f)<<8))
    #define ISAPNP_FUNCTION(x)      ISAPNP_DEVICE(x)
    
    /*
     *
     */
    
#if 0
    #include <linux/mod_devicetable.h>
#endif
    
    #define DEVICE_COUNT_COMPATIBLE 4
    
    #define ISAPNP_CARD_DEVS        8
    
    #define ISAPNP_CARD_ID(_va, _vb, _vc, _device) \
                    .card_vendor = ISAPNP_VENDOR(_va, _vb, _vc), .card_device = ISAPNP_DEVICE(_device)
    #define ISAPNP_CARD_END \
                    .card_vendor = 0, .card_device = 0
    #define ISAPNP_DEVICE_ID(_va, _vb, _vc, _function) \
                    { .vendor = ISAPNP_VENDOR(_va, _vb, _vc), .function = ISAPNP_FUNCTION(_function) }
    
    struct isapnp_card_id {
            unsigned long driver_data;      /* data private to the driver */
            unsigned short card_vendor, card_device;
            struct {
                    unsigned short vendor, function;
            } devs[ISAPNP_CARD_DEVS];       /* logical devices */
    };
    
 #define ISAPNP_DEVICE_SINGLE(_cva, _cvb, _cvc, _cdevice, _dva, _dvb, _dvc, _dfunction) \
                    .card_vendor = ISAPNP_VENDOR(_cva, _cvb, _cvc), .card_device =  ISAPNP_DEVICE(_cdevice), \
                    .vendor = ISAPNP_VENDOR(_dva, _dvb, _dvc), .function = ISAPNP_FUNCTION(_dfunction)
 #define ISAPNP_DEVICE_SINGLE_END \
                 .card_vendor = 0, .card_device = 0

struct isapnp_device_id {
	unsigned short card_vendor, card_device;
	unsigned short vendor, function;
	kernel_ulong_t driver_data;     /* data private to the driver */
};

#endif
