#include "sgbuf.h"

int ne_transmit(void *privdata, sgbuf_t *p);
int ne_setup(void *privdata, u16_t iobase, u16_t irq, u16_t membase, u16_t memsize);
size_t ne_privdata_size(void);
const u8_t *ne_hwaddr_get(void *arg);
