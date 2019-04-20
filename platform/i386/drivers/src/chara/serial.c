/* Copyright 2006, 2015, 2016 Ismael Luceno <ismael@linux.com>
 *
 * FIXME the file is autocontained because it came from another project,
 *       should be fixed to use primitives provided by OX.
 */

#define min(a, b) \
({ \
	__typeof__(a) _a = (a);	\
	__typeof__(b) _b = (b); \
	(_a < _b) ? _a : _b;	\
})

typedef unsigned int uint;
typedef unsigned char u8;
/* If not provided by the compiler the buildsystem should take care */
typedef __UINT16_TYPE__ u16;
typedef __UINT32_TYPE__ u32;
typedef __SIZE_TYPE__ size_t;

#define BDA_IOBASE_TTYS(x) (((const u16 *)0x400)[(x)])
/* Normally:
 * ttyS0 at 0x3f8 / IRQ 4
 * ttyS1 at 0x2f8 / IRQ 3
 * ttyS2 at 0x3e8 / IRQ 4
 * ttyS3 at 0x2e8 / IRQ 3
 */

#define UART_FCR	2	/* FIFO Control Register */
#define UART_LCR	3	/* Line Control Register */
#define UART_MCR	4	/* Modem Control Register */
#define UART_LSR	5	/* Line Status Register */

#define WAIT_RD		1
#define WAIT_WR		0x20


inline static
void outb(u8 v, u16 port)
{
	__asm__ __volatile__ ("outb %0, %1\n" : : "a" (v), "dN" (port));
}

inline static
u8 inb(u16 port)
{
	u8 v;
	__asm__ __volatile__ ("inb %1, %0\n" : "=a" (v) : "dN" (port));
	return v;
}

inline static
void swait(u16 iobase, u8 mask)
{
	int ready;
	do { ready = inb(iobase + UART_LSR) & mask; } while (!ready);
}

size_t
serial_readln(uint port, char *buf, size_t len)
{
	if (port > 3 || !len)
		return 0;

	const u16 iobase = BDA_IOBASE_TTYS(port);
	char *p = buf;

	/* XXX: The double loop prevents reading empty lines */
	char a;
	do {
		swait(iobase, WAIT_RD);
		*p++ = a = inb(iobase);
	} while (--len && a < 32);
	do {
		if (!len--)
			break;
		swait(iobase, WAIT_RD);
		*p++ = a = inb(iobase);
	} while (a >= 32);
	return p - buf;
}

void
serial_write(uint port, const char *buf, size_t len)
{
	if (port > 3 || !len)
		return;

	const u16 iobase = BDA_IOBASE_TTYS(port);

	while (len--) {
		/* FIXME FIFO is enabled, could send in batches */
		swait(iobase, WAIT_WR);
		outb(*buf++, iobase);
	}
}

void
serial_puts(uint port, const char *buf)
{
	if (port > 3)
		return;

	const u16 iobase = BDA_IOBASE_TTYS(port);

	while (*buf) {
		/* FIXME FIFO is enabled, could send in batches */
		swait(iobase, WAIT_WR);
		if (*buf == '\n')
			outb('\r', iobase);
		outb(*buf++, iobase);
	}
}

static const
struct {
	u8 rport;
	u8 val;
} pinit[] = {
	/* Set DLAB */
	{ UART_LCR, 0x80 },

	/* Set clock divisor (1 for 115200) */
	{ 0, 1 },		/* Low byte */
	{ 1, 0 },		/* High byte */

	/* Clear DLAB and set comm. parameters */
	{ UART_LCR, 0x03 },	/* 8N1 */

	/* Enable FIFO, cleart RX/TX flags */
	{ UART_FCR, 0xC7},

	/* HW flow control, no interrupts */
	{ UART_MCR, 0x0F },
};

void serial_init(void)
{
	/* FIXME: Register available ports somewhere */
	for (int i = 0; i < 4; i++) {
		u16 iobase = BDA_IOBASE_TTYS(i);
		if (!iobase)
			continue;

		for (int j = 0; j < sizeof(pinit) / sizeof(pinit[0]); j++)
			outb(pinit[j].val, iobase + pinit[j].rport);
	}
}
