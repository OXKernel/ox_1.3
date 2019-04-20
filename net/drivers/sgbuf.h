/* Scatter-gather buffer abstraction, for portability between network stacks.
 * This one implements the interface with LwIP.
 */

#include "lwip/pbuf.h"

typedef struct pbuf sgbuf_t;

static inline
sgbuf_t *
sgbuf_alloc(u16_t len)
{
	return pbuf_alloc(PBUF_RAW, len, PBUF_RAM);
}

static inline
void
sgbuf_free(sgbuf_t *buf)
{
	pbuf_free(buf);
}

__attribute__((const))
static inline
sgbuf_t *
sgbuf_next(sgbuf_t *buf)
{
	return buf->next;
}

__attribute__((const))
static inline
void *
sgbuf_base(sgbuf_t *buf)
{
	return buf->payload;
}

__attribute__((const))
static inline
u16_t
sgbuf_len(sgbuf_t *buf)
{
	return buf->len;
}

__attribute__((const))
static inline
u16_t
sgbuf_totlen(sgbuf_t *buf)
{
	return buf->tot_len;
}

err_t lwip_ne2k_input(struct pbuf *p);

static inline
int
relay_sgbuf_to_netstack(sgbuf_t *p)
{
	return lwip_ne2k_input(p);
}
